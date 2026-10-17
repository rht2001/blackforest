import json
import time
from pymodbus.client import ModbusTcpClient
import numpy as np
from typing import Literal, Dict

DEFAULT_MOVEMENT_SPEED = [400] * 6
CLOSE_THRESHOLD = 400
STATES = Literal["starting", "initialised", "moving", "steady_state", "opening"]

DataTypes = Literal["ANGLES", "FORCES", "POSITION"]
data_packet: tuple[int, int, int, int, int, int]
from config.config_parameters import REGISTERS
from logic.robot_outputs import robot_outputs
from pymodbus.exceptions import ModbusException


class RobotHandController:
    def __init__(self, ip="192.168.123.211", port=6000, simulate=False):
        """ Initialize a RobotHandController instance. Lightweight python wrapper for the Modbus TCP API to control the robot hand with descriptive function names.

        Parameters
        ----------
        ip : str, optional
            The IP address of the robot hand, by default "192.168.123.211"
        port : int, optional
            The port of the robot hand, by default 6000
        """
        self.ip = ip
        self.port = port
        self.sliders = []
        self.auto_mode_active = True
        self.auto_contact_trigerred = False
        self.is_connected = False
        self.mock_angles = [1000, 1000, 1000, 1000, 1000, 1000]

        self.client = None
        if not simulate:
            try:
                self.client = ModbusTcpClient(self.ip, port=self.port, timeout=1.0)
                self.is_connected = bool(self.client.connect())
            except (ModbusException, OSError) as error:
                print(f"[RobotHandController] Connection error: {error}")
        if self.is_connected:
            print(f"[RobotHandController] Connected to {self.ip}:{self.port}")
        else:
            print("[RobotHandController] Simulation mode; telemetry marked simulated.")

        self.send_addresses = {
            "ANGLES": 1486,
            "FORCES": 1498,
            "POSITION": 1474,
        }

        self.receive_addresses = {
            "ANGLES": 1546,
            "FORCES": 1582,
            "POSITION": REGISTERS["POS_ACT"],
        }
        self.sensor_groups = [
            ("Little Tip", 3000, 3, 3, "Little", (8, 10)),
            ("Little Nail", 3018, 12, 8, "Little", (-2, 40)),
            ("Little Pad", 3210, 10, 8, "Little", (4, 190)),

            ("Ring Tip", 3370, 3, 3, "Ring", (-9, -20)),
            ("Ring Nail", 3388, 12, 8, "Ring", (-23, 30)),
            ("Ring Pad", 3580, 10, 8, "Ring", (-18, 200)),

            ("Middle Tip", 3740, 3, 3, "Middle", (-21, -25)),
            ("Middle Nail", 3758, 12, 8, "Middle", (-39, 22)),
            ("Middle Pad", 3950, 10, 8, "Middle", (-35, 210)),

            ("Index Tip", 4110, 3, 3, "Index", (-38, -19)),
            ("Index Nail", 4128, 12, 8, "Index", (-49, 17)),
            ("Index Pad", 4320, 10, 8, "Index", (-54, 200)),

            ("Thumb Tip", 4480, 3, 3, "Thumb", (185, 140)),
            ("Thumb Nail", 4498, 12, 8, "Thumb", (145, 170)),
            ("Thumb Mid", 4690, 3, 3, "Thumb", (90, 240)),
            ("Thumb Pad", 4708, 12, 8, "Thumb", (-30, 240)),

            ("Palm", 4900, 8, 14, "Palm", (-15, 0))
        ]
        self.sensor_offsets = {}
        self.state: STATES = "starting"
        if self.is_connected:
            self.startup()
        else:
            self.state = "initialised"
            self.sliders = [1000] * 6

    def startup(self):
        self.set_default_speed()
        # With this, we always start at fully open
        self.send_command(value=[1000] * 6)
        print("opened")
        self.state = "initialised"
        self.sliders = [1000] * 6

    def set_default_speed(self):
        if not self.is_connected:
            return
        self.send_all_commands("DEFAULT_SPEED_SET", value=DEFAULT_MOVEMENT_SPEED)

    def safe_read(self, addr, length):
        if not self.is_connected:
            return [0] * length
        values = []
        try:
            for offset in range(0, length, 120):
                count = min(120, length - offset)
                response = self.client.read_holding_registers(
                    address=addr + offset, count=count, device_id=1)
                if response.isError() or len(response.registers) != count:
                    return None
                values.extend(response.registers)
        except (ModbusException, OSError):
            return None
        return values

    def close(self):
        """ Close the robot hand completely. """
        vals = [0, 0, 0, 0, 234, 670]
        self.send_angles(vals)

    def calibrate_offsets(self):
        print("Calibrating sensor offsets...")
        time.sleep(1)

        for name, addr, h, w, key, rel in self.sensor_groups:
            data = self.safe_read(addr, h * w)
            self.sensor_offsets[name] = np.array(data) if data else np.zeros(h * w)

    def send_angles(self, angles):
        self.send_command(value=angles, type="ANGLES")

    def close_on_contact(self, matrix):
        if not self.auto_mode_active:
            return False

        # At least 3 sensor pixels must exceed CLOSE_THRESHOLD
        if np.count_nonzero(matrix > CLOSE_THRESHOLD) >= 3:
            self.auto_mode_active = False  # Prevent further triggers
            self.send_command(value=[0, 0, 0, 0, 1000, 1000])  # Execute existing close function
            self.auto_contact_trigerred = True
            print(f"Contact detected at {matrix[matrix > 0]}")
            return True

        return False

    def read_error(self):
        if not self.is_connected:
            return None
        return self.client.read_holding_registers(address=1606, count=6, device_id=1)

    def send_all_commands(self, register_name, value):
        if not self.is_connected:
            return
        self.client.write_registers(
            address=REGISTERS[register_name],
            values=value,
            device_id=1
        )

    def send_command(self, value: list[int], type: DataTypes = "ANGLES"):
        if type == "ANGLES":
            self.mock_angles = list(value)
        if not self.is_connected:
            return
        self.client.write_registers(
            address=self.send_addresses[type],
            values=value,
            device_id=1
        )

    def read_command(self, type: DataTypes = "ANGLES"):
        if not self.is_connected:
            return list(self.mock_angles) if type == "ANGLES" else [0] * 6
        values = self.safe_read(self.receive_addresses[type], 6)
        if values is not None and type == "FORCES":
            values = [v - 65536 if v >= 32768 else v for v in values]
        return values

    def receive_data(self, type: DataTypes = "ANGLES"):
        """Backward compatibility alias for read_command."""
        return self.read_command(type=type)

    def read_current_angles(self):
        return self.read_command("ANGLES")

    def read_tactile_data(self):
        # Preserve region order: 112 cells x 17 regions; None for padding/errors.
        points = [[None] * len(self.sensor_groups) for _ in range(112)]
        valid = []
        for column, (_, addr, height, width, _, _) in enumerate(self.sensor_groups):
            values = self.safe_read(addr, height * width)
            valid.append(values is not None)
            if values is not None:
                for row, value in enumerate(values):
                    points[row][column] = value
        return points, valid

    def main(self) -> robot_outputs:
        timestamp = time.time_ns()
        tactile, valid = self.read_tactile_data()
        measured = [value for row in tactile for value in row if value is not None]
        self.close_on_contact(np.asarray(measured))
        return {
            "simulated": not self.is_connected,
            "timestamp_ns": timestamp,
            "angles": self.read_command("ANGLES"),
            "forces": self.read_command("FORCES"),
            "positions": self.read_command("POSITION"),
            "tactile": tactile,
            "tactile_valid": valid,
        }

    def disconnect(self):
        if self.client is not None:
            self.client.close()
        self.is_connected = False
