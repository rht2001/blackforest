"""Exercise acquisition and lifecycle with fake IO, never instantiate hardware."""
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from interface import robot_control as controller
from ui.appInterface import HandApp
import main as gui


class IntegrationTests(unittest.TestCase):
    def robot(self):
        robot = controller.RobotHandController(mock=True)
        robot.client = Mock()
        robot.is_connected = True
        robot.receive_addresses = {"ANGLES": 1546, "FORCES": 1582, "POSITION": 1534}
        robot.sensor_groups = [("test", 3000, 1, 2, "test", (0, 0))]
        robot.auto_mode_active = False
        return robot

    def test_simulation_never_connects_and_is_marked(self):
        with patch.object(controller, "ModbusTcpClient") as client:
            robot = controller.RobotHandController(simulate=True)
            client.assert_not_called()
            robot.send_angles([12] * 6)
            self.assertEqual(robot.receive_data(), [12] * 6)
            outputs = robot.capture_telemetry()
            self.assertTrue(outputs["simulated"])
            self.assertEqual(outputs["angles"], [12] * 6)
            self.assertEqual(outputs["tactile_valid"], [True] * 17)
            robot.disconnect()

    def test_disconnected_fallback_is_preserved(self):
        with patch.object(controller, "ModbusTcpClient") as client:
            client.return_value.connect.return_value = False
            robot = controller.RobotHandController()
            client.return_value.write_registers.assert_not_called()
            self.assertTrue(robot.capture_telemetry()["simulated"])
            robot.disconnect()
            client.return_value.close.assert_called_once()

    def test_display_uses_snapshot_without_sensor_reads(self):
        app = Mock()
        app.robot.sensor_groups = [("test", 3000, 1, 2, "test", (0, 0))]
        app.robot.sensor_offsets = {}
        app.offsets = {"test": (0, 0)}
        app.pixel = 6
        HandApp.draw(app, {"tactile": [[0], [0]], "tactile_valid": [True]})
        app.robot.safe_read.assert_not_called()
        self.assertEqual(app.canvas.create_rectangle.call_count, 2)

    def test_bad_reads_and_signed_forces(self):
        robot = self.robot()
        robot.client.read_holding_registers.return_value = Mock(
            isError=lambda: False, registers=[1])
        self.assertIsNone(robot.safe_read(1534, 6))
        robot.client.read_holding_registers.side_effect = OSError("offline")
        self.assertIsNone(robot.safe_read(1534, 6))
        robot.safe_read = Mock(return_value=[0, 32767, 32768, 65535, 1, 2])
        self.assertEqual(robot.read_command("FORCES"), [0, 32767, -32768, -1, 1, 2])

    def test_one_acquisition_and_validity(self):
        robot = self.robot()
        robot.safe_read = Mock(side_effect=[None, [500]*6, [65535]*6, [3]*6])
        outputs = robot.capture_telemetry()
        self.assertEqual(outputs["tactile_valid"], [False])
        self.assertIsNone(outputs["tactile"][0][0])
        self.assertEqual(outputs["forces"], [-1]*6)
        self.assertEqual(outputs["positions"], [3]*6)
        self.assertEqual([call.args[0] for call in robot.safe_read.call_args_list],
                         [3000, 1546, 1582, 1534])

    def test_tactile_padding(self):
        robot = self.robot()
        robot.safe_read = Mock(return_value=[4, 5])
        robot.read_tactile_data()
        self.assertEqual(robot.last_tactile_valid, [True])
        self.assertEqual(robot.last_sensor_data["test"].tolist(), [4, 5])
        robot.safe_read = Mock(side_effect=[[4, 5], [500]*6, [0]*6, [0]*6])
        outputs = robot.capture_telemetry()
        self.assertEqual(outputs["tactile"][:3], [[4], [5], [None]])

    def test_gui_passes_same_snapshot_and_flushes_before_destroy(self):
        app, robot, dataset = Mock(), Mock(), Mock()
        loop = gui.AcquisitionLoop(app, robot, dataset)
        loop.tick()
        robot.capture_telemetry.assert_called_once_with()
        dataset.loop.assert_called_once_with(robot.capture_telemetry.return_value)
        app.main.assert_called_once_with(robot.capture_telemetry.return_value)
        order = Mock()
        order.attach_mock(dataset.close, "flush")
        order.attach_mock(app.destroy, "destroy")
        loop.on_close()
        self.assertEqual([call[0] for call in order.mock_calls], ["flush", "destroy"])
        app.after_cancel.assert_called_once()
        loop.tick()
        robot.capture_telemetry.assert_called_once()

    def test_upstream_datadict_is_populated_without_duplicate_reads(self):
        robot = self.robot()
        robot.safe_read = Mock(side_effect=[[4, 5], [500]*6, [65535]*6, [3]*6])
        robot.sensor_offsets = {"test": controller.np.array([1, 2])}
        outputs = robot.capture_telemetry()
        self.assertEqual(set(robot.results), set(controller.DataDict.__annotations__))
        self.assertEqual(robot.results["angles"].tolist(), outputs["angles"])
        self.assertEqual(robot.results["position"].tolist(), outputs["positions"])
        self.assertEqual(robot.results["tactile_matrix"][:2, 0].tolist(), [3, 3])
        self.assertEqual(outputs["tactile"][:2], [[4], [5]])
        self.assertEqual(robot.safe_read.call_count, 4)

    def test_real_connection_is_created_only_once(self):
        with patch.object(controller, "ModbusTcpClient") as client:
            client.return_value.connect.return_value = True
            with patch.object(controller.RobotHandController, "startup") as startup:
                robot = controller.RobotHandController()
            client.assert_called_once()
            client.return_value.connect.assert_called_once()
            startup.assert_called_once()
            robot.disconnect()

    def test_reset_rearms_contact_detection_from_fixing(self):
        robot = self.robot()
        robot.safe_read = Mock(side_effect=[[0, 0], [0]*6, [0]*6, [0]*6])
        with patch.object(controller.time, "sleep"):
            outputs = robot.capture_telemetry()
        self.assertTrue(robot.auto_mode_active)
        self.assertEqual(robot.state, "resetting")
        robot.client.write_registers.assert_called_once_with(
            address=1486, values=[1000]*6, device_id=1)
        self.assertEqual(outputs["angles"], [0]*6)

    def test_save_failure_keeps_window_for_retry(self):
        app, robot, dataset = Mock(), Mock(), Mock()
        loop = gui.AcquisitionLoop(app, robot, dataset)
        dataset.close.side_effect = OSError("disk full")
        with self.assertRaises(OSError):
            loop.on_close()
        app.destroy.assert_not_called()
        self.assertFalse(loop.running)


if __name__ == "__main__":
    unittest.main()
