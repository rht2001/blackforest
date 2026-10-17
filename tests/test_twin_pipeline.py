"""Actual localhost HTTP integration, explicit simulation, no hardware IO."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from urllib.request import ProxyHandler, build_opener
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "digital_twin"))
from interface.robot_control import RobotHandController
from logic.robot_dataset import RobotDataset
from logic.robot_outputs import robot_outputs
from logic.twin_publisher import TwinPublisher, twin_payload
from main import AcquisitionLoop
import bridge_server as bridge


class TwinPipelineTests(unittest.TestCase):
    def test_partial_regions_and_invalid_pose_clear_old_values(self):
        robot = RobotHandController(simulate=True)
        outputs = robot.capture_telemetry()
        outputs["tactile_valid"][0] = False
        outputs["angles"] = None
        payload = twin_payload(outputs)
        self.assertNotIn("little_tip", payload["tactile_regions"])
        self.assertEqual(len(payload["tactile_regions"]), 16)
        self.assertIsNone(payload["angles_act"])
        self.assertIsNone(payload["angles_set"])
        # Palm's container retains acquisition order, not image-space order.
        outputs["tactile"][0][16] = 321
        self.assertEqual(twin_payload(outputs)["tactile_regions"]["palm"][0][0], 321)
        bridge.normalize(payload)

    def test_csv_and_http_from_same_acquisition(self):
        server = bridge.ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        url = f"http://127.0.0.1:{server.server_port}"
        publisher = TwinPublisher(url + "/api/state")
        robot = RobotHandController(simulate=True)
        robot.send_angles([100, 200, 300, 400, 500, 600])
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "dataset.csv"
                dataset = RobotDataset(robot_outputs, path)
                loop = AcquisitionLoop(Mock(), robot, dataset, publisher=publisher)
                loop.tick()
                loop.on_close()
                self.assertEqual(publisher.sent_count, 1, publisher.last_error)
                self.assertFalse(publisher._worker.is_alive())
                opener = build_opener(ProxyHandler({}))
                with opener.open(url + "/api/state", timeout=2) as response:
                    state = json.load(response)
                with path.open(newline='', encoding='utf-8') as file:
                    rows = list(csv.DictReader(file, delimiter=';'))
                self.assertEqual(state["angles_act"], json.loads(rows[0]["angles"]))
                self.assertEqual(state["force_act"], json.loads(rows[0]["forces"]))
                self.assertTrue(state["simulated"])
                self.assertEqual(len(state["tactile_regions"]), 17)
                self.assertEqual(sum(len(row) for matrix in state["tactile_regions"].values()
                                     for row in matrix), 1062)
                for route in ("/", "/tactile"):
                    with opener.open(url + route, timeout=2) as response:
                        self.assertEqual(response.status, 200)
                        self.assertIn(b'Simulation aus Python', response.read())
        finally:
            publisher.close()
            robot.disconnect()
            server.shutdown()
            server.server_close()

    def test_bridge_offline_does_not_stop_csv(self):
        # Reserve and release a port, then verify failed HTTP cannot interrupt CSV.
        server = bridge.ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        port = server.server_port
        server.server_close()
        publisher = TwinPublisher(f"http://127.0.0.1:{port}/api/state")
        robot = RobotHandController(simulate=True)
        try:
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "dataset.csv"
                dataset = RobotDataset(robot_outputs, path)
                loop = AcquisitionLoop(Mock(), robot, dataset, publisher=publisher)
                loop.tick()
                loop.on_close()
                self.assertIsNotNone(publisher.last_error)
                with path.open(newline='', encoding='utf-8') as file:
                    self.assertEqual(len(list(csv.DictReader(file, delimiter=';'))), 1)
        finally:
            publisher.close()
            robot.disconnect()


if __name__ == "__main__":
    unittest.main()
