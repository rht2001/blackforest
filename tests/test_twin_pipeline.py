import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from urllib.request import ProxyHandler, build_opener

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "digital_twin"))
from logic.twin_publisher import REGIONS, TwinPublisher, robot_snapshot, twin_payload
import bridge_server as bridge
import main as app_main


class TwinTests(unittest.TestCase):
    def snapshot(self):
        groups = [(key, 3000, height, width, key, (0, 0))
                  for key, (height, width) in REGIONS]
        robot = SimpleNamespace(
            sensor_groups=groups, is_connected=False,
            last_sensor_data={key: np.arange(height * width)
                              for key, (height, width) in REGIONS},
            safe_read=Mock(), send_command=Mock())
        results = {"angles": np.array([100, 200, 300, 400, 500, 600]),
                   "forces": np.array([65535, 0, 1, 2, 3, 4])}
        return robot, results

    def test_cache_adapter_without_hardware_io(self):
        robot, results = self.snapshot()
        snapshot = robot_snapshot(robot, results)
        payload = twin_payload(snapshot)
        self.assertEqual(payload["angles_act"], results["angles"].tolist())
        self.assertEqual(payload["force_act"], [-1, 0, 1, 2, 3, 4])
        self.assertEqual(len(payload["tactile_regions"]), 17)
        self.assertEqual(sum(len(row) for matrix in payload["tactile_regions"].values()
                             for row in matrix), 1062)
        self.assertIsNone(snapshot["tactile"][9][0])
        self.assertTrue(payload["simulated"])
        robot.safe_read.assert_not_called()
        robot.send_command.assert_not_called()
        self.assertEqual(results["forces"][0], 65535)

    def test_missing_incomplete_and_invalid_regions(self):
        robot, results = self.snapshot()
        del robot.last_sensor_data["little_tip"]
        robot.last_sensor_data["ring_tip"] = [1]
        robot.last_sensor_data["middle_tip"] = [5000] * 9
        payload = twin_payload(robot_snapshot(robot, results))
        self.assertEqual(len(payload["tactile_regions"]), 14)
        for key in ("little_tip", "ring_tip", "middle_tip"):
            self.assertNotIn(key, payload["tactile_regions"])

    def test_invalid_pose_clears_previous_values(self):
        robot, results = self.snapshot()
        results["angles"] = np.array([1001] * 6)
        self.assertIsNone(twin_payload(robot_snapshot(robot, results))["angles_act"])

    def test_http_pipeline_and_both_viewers(self):
        server = bridge.ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base_url = f"http://127.0.0.1:{server.server_port}"
        publisher = TwinPublisher(base_url + "/api/state")
        try:
            robot, results = self.snapshot()
            publisher.submit(robot_snapshot(robot, results))
            publisher.close()
            self.assertEqual(publisher.sent_count, 1, publisher.last_error)
            opener = build_opener(ProxyHandler({}))
            with opener.open(base_url + "/api/state", timeout=2) as response:
                state = json.load(response)
            self.assertEqual(state["angles_act"], results["angles"].tolist())
            self.assertEqual(len(state["tactile_regions"]), 17)
            for route in ("/", "/tactile"):
                with opener.open(base_url + route, timeout=2) as response:
                    self.assertEqual(response.status, 200)
                    self.assertIn(b"Simulation aus Python", response.read())
        finally:
            publisher.close()
            server.shutdown()
            server.server_close()

    def test_unreachable_bridge_does_not_raise(self):
        server = bridge.ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        port = server.server_port
        server.server_close()
        publisher = TwinPublisher(f"http://127.0.0.1:{port}/api/state")
        try:
            robot, results = self.snapshot()
            publisher.submit(robot_snapshot(robot, results))
            publisher.close()

            self.assertIsNotNone(publisher.last_error)
            self.assertFalse(publisher._worker.is_alive())
        finally:
            publisher.close()

    def run_app(self, arguments):
        robot, results = self.snapshot()
        robot.main = Mock(return_value=results)
        with patch.object(sys, "argv", ["blackforest", *arguments]), \
                patch.object(app_main, "RobotHandController", return_value=robot), \
                patch.object(app_main, "HandApp") as app, \
                patch.object(app_main, "RobotDatabase") as database, \
                patch.object(app_main, "TwinPublisher") as publisher, \
                patch.object(app_main.time, "sleep"):
            app.return_value.main.side_effect = [None, app_main.tk.TclError()]
            app_main.main()
            database.return_value.main.assert_called_once_with(data=results)
            robot.main.assert_called_once()
            robot.safe_read.assert_not_called()
            return publisher

    def test_main_without_twin_keeps_database_and_no_publisher(self):
        self.run_app([]).assert_not_called()

    def test_main_with_twin_publishes_once_and_closes(self):
        publisher = self.run_app(["--twin"])
        publisher.assert_called_once_with("http://127.0.0.1:18765/api/state")
        publisher.return_value.submit.assert_called_once()
        publisher.return_value.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
