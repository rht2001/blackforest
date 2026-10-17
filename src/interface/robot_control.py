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


class RobotHandController:
    def __init__(self, ip: str = "192.168.123.211", port: int = 6000, mock: bool = False):
        """
        Lightweight Python wrapper for Modbus TCP control of the RH56DFTP dexterous hand.
        """
        self.ip = ip
        self.port = port
        self.client: ModbusTcpClient | None = None
        self.auto_mode_active = True
        self.auto_contact_trigerred = False
        self.is_connected = False
        self.current_angles = [1000] * 6
        self.last_sensor_data: dict[str, np.ndarray] = {}
        self.last_all_points = np.zeros((112, 17))

        self.send_addresses = {
            "ANGLES": 1486,
            "FORCES": 1498,
            "POSITION": 1474,
        }

        self.receive_addresses = {
            "ANGLES": 1546,
            "FORCES": 1582,
            "POSITION": 1594,
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

        if not mock:
            try:
                self.client = ModbusTcpClient(self.ip, port=self.port, timeout=1.0)
                if self.client.connect():
                    self.is_connected = True
                    print(f"[RobotHandController] Connected to hand at {self.ip}:{self.port}")
                else:
                    print(f"[RobotHandController] Could not reach {self.ip}:{self.port}. Running in simulation/disconnected mode.")
            except Exception as e:
                print(f"[RobotHandController] Connection error ({e}). Running in simulation/disconnected mode.")
                self.client = None
        else:
            self.client = None
            print("[RobotHandController] Explicitly initialized in mock mode.")

        if self.is_connected:
            self.startup()
        else:
            self.state = "initialised"

    # ==========================================
    # Unified Hardware Access Points
    # ==========================================

    def write_data(self, target: DataTypes | str, values: list[int]) -> bool:
        """
        Unified write access point for commands and register settings.
        Target can be a standard data category ('ANGLES', 'FORCES', 'POSITION')
        or a specific register key from REGISTERS.
        """
        if target in self.send_addresses:
            address = self.send_addresses[target]
        elif target in REGISTERS:
            address = REGISTERS[target]
        else:
            raise ValueError(f"Unknown write target: {target}")

        if target in ("ANGLES", "ANGLE_SET"):
            self.current_angles = list(values)

        if not self.is_connected or self.client is None:
            return True

        res = self.client.write_registers(address=address, values=values, device_id=1)
        return not res.isError()

    def read_data(self, target: DataTypes | str, count: int = 6) -> list[int] | None:
        """
        Unified read access point for actuator telemetry and register inspection.
        Target can be a standard data category ('ANGLES', 'FORCES', 'POSITION')
        or a specific register key from REGISTERS.
        """
        if target in self.receive_addresses:
            address = self.receive_addresses[target]
        elif target in REGISTERS:
            address = REGISTERS[target]
        else:
            raise ValueError(f"Unknown read target: {target}")

        if not self.is_connected or self.client is None:
            if target in ("ANGLES", "ANGLE_ACT"):
                return list(self.current_angles)
            return [0] * count

        res = self.client.read_holding_registers(address=address, count=count, device_id=1)
        if not res.isError():
            if target in ("ANGLES", "ANGLE_ACT"):
                self.current_angles = list(res.registers)
            return res.registers

        print(f"[RobotHandController] Failed to read {target} from address {address}")
        return None

    def safe_read(self, addr: int, length: int) -> list[int]:
        """Reads arbitrary holding registers in safe chunks of up to 120 registers."""
        if not self.is_connected or self.client is None:
            return [0] * length

        values = []
        for i in range(0, length, 120):
            res = self.client.read_holding_registers(
                address=addr + i,
                count=min(120, length - i),
                device_id=1
            )
            if not res.isError():
                values.extend(res.registers)
        return values

    # ==========================================
    # High-Level Domain Helpers
    # ==========================================

    def startup(self):
        """Initializes speed and opens hand on startup."""
        self.set_default_speed()
        self.open_hand()
        print("[RobotHandController] Hand opened and ready.")
        self.state = "initialised"

    def set_default_speed(self):
        """Sets the default speed for all 6 DOFs."""
        self.write_data("DEFAULT_SPEED_SET", DEFAULT_MOVEMENT_SPEED)

    def set_angles(self, angles: list[int]) -> bool:
        """Sets target angles for all 6 DOFs (0=closed, 1000=open)."""
        return self.write_data("ANGLES", angles)

    def get_angles(self) -> list[int]:
        """Reads actual joint angles for all 6 DOFs."""
        res = self.read_data("ANGLES", count=6)
        return res if res is not None else list(self.current_angles)

    def set_forces(self, forces: list[int]) -> bool:
        """Sets the force control threshold for all 6 DOFs (0-3000g)."""
        return self.write_data("FORCES", forces)

    def get_forces(self) -> list[int]:
        """Reads actual force values for all 6 DOFs."""
        res = self.read_data("FORCES", count=6)
        return res if res is not None else [0] * 6

    def open_hand(self):
        """Opens all fingers fully (angle 1000)."""
        self.set_angles([1000] * 6)

    def close_hand(self):
        """Closes hand to standard grasp position."""
        self.set_angles([0, 0, 0, 0, 234, 670])

    def calibrate_offsets(self):
        """Calibrates baseline tactile sensor readings."""
        print("Calibrating sensor offsets...")
        time.sleep(1)
        for name, addr, h, w, key, rel in self.sensor_groups:
            data = self.safe_read(addr, h * w)
            self.sensor_offsets[name] = np.array(data) if data else np.zeros(h * w)

    def close_on_contact(self, matrix: np.ndarray) -> bool:
        """Triggers grip closure if tactile pressure exceeds contact threshold."""
        if not self.auto_mode_active:
            return False

        if np.count_nonzero(matrix > CLOSE_THRESHOLD) >= 3:
            self.auto_mode_active = False
            self.write_data("ANGLES", [0, 0, 0, 0, 1000, 1000])
            self.auto_contact_trigerred = True
            print(f"Contact detected at {max(matrix[matrix > 0])}")
            return True

        return False

    def read_error(self):
        """Reads error code bitfields from actuators."""
        if not self.is_connected or self.client is None:
            return None
        return self.client.read_holding_registers(address=1606, count=6, device_id=1)

    def read_tactile_data(self) -> tuple[np.ndarray, np.ndarray]:
        """Reads tactile matrix data across all 17 sensor groups and caches results."""
        data_matrix = np.zeros((1, 17))
        all_points = np.zeros((112, 17))
        raw_cache = {}

        for index, (name, addr, h, w, key, rel) in enumerate(self.sensor_groups):
            data = self.safe_read(addr, h * w)
            if not data:
                continue

            raw = np.array(data)
            data_matrix[0, index] = np.mean(raw)
            all_points[0:len(raw), index] = raw
            raw_cache[name] = raw

        self.last_sensor_data = raw_cache
        self.last_all_points = all_points
        return data_matrix, all_points

    def main(self):
        """Periodic control step."""
        _, all_points = self.read_tactile_data()
        self.close_on_contact(all_points)
        self.read_data("ANGLES")

    # ==========================================
    # Backward Compatibility Aliases
    # ==========================================
    def close(self):
        self.close_hand()

    def send_angles(self, angles: list[int]):
        return self.set_angles(angles)

    def send_command(self, value: list[int], type: DataTypes = "ANGLES"):
        return self.write_data(type, value)

    def send_all_commands(self, register_name: str, value: list[int]):
        return self.write_data(register_name, value)

    def read_command(self, type: DataTypes = "ANGLES") -> list[int]:
        res = self.read_data(type)
        return res if res is not None else [0] * 6

    def receive_data(self, type: DataTypes = "ANGLES") -> list[int]:
        return self.read_command(type=type)

    def read_current_angles(self) -> list[int] | None:
        return self.read_data("ANGLES", count=6)
