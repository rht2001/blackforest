"""Read-only hardware contract, using a fake transport and no real hand."""
import ast
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from interface.robot_control import RobotHandController


class ReadOnlyTests(unittest.TestCase):
    def test_connect_poll_and_calibration_only_read_registers(self):
        client = Mock(spec=['connect', 'read_holding_registers', 'close'])
        client.connect.return_value = True
        def read(*, address, count, device_id):
            response = Mock()
            response.isError.return_value = False
            response.registers = [321 if address == 1546 else 0] * count
            return response
        client.read_holding_registers.side_effect = read
        with patch('interface.robot_control.ModbusTcpClient', return_value=client) as factory:
            robot = RobotHandController()
            factory.assert_called_once()
            self.assertEqual(client.mock_calls, [unittest.mock.call.connect()])
            result = robot.poll()
            self.assertEqual(result['angles'].tolist(), [321]*6)
            self.assertEqual(robot.current_angles, [321]*6)
            with patch('interface.robot_control.time.sleep'):
                robot.calibrate_offsets()
            robot.read_error()
            addresses = [c.kwargs['address'] for c in client.read_holding_registers.call_args_list]
            self.assertIn(1534, addresses)
            self.assertIn(4900, addresses)
            self.assertTrue(all(c[0] in ('connect', 'read_holding_registers') for c in client.mock_calls))

    def test_mock_mode_never_constructs_network_client(self):
        with patch('interface.robot_control.ModbusTcpClient') as factory:
            robot = RobotHandController(mock=True)
            robot.poll()
        factory.assert_not_called()
        self.assertFalse(robot.is_connected)

    def test_angle_read_error_is_invalid_not_closed_pose(self):
        robot = RobotHandController(mock=True)
        robot.client = Mock()
        robot.is_connected = True
        robot.client.read_holding_registers.return_value.isError.return_value = True
        self.assertEqual(robot.get_angles(), [-1]*6)

    def test_source_has_no_hardware_write_or_movement_calls(self):
        forbidden = {'write_register', 'write_registers', 'write_coil', 'write_coils',
                     'send_command', 'write_data', 'set_angles', 'set_forces',
                     'set_default_speed', 'set_max_force', 'open_hand', 'close_hand',
                     'close_on_contact'}
        for path in (ROOT / 'src').rglob('*.py'):
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                    self.assertNotIn(node.func.attr, forbidden, str(path))
        for name in forbidden:
            self.assertFalse(hasattr(RobotHandController, name), name)


if __name__ == '__main__':
    unittest.main()
