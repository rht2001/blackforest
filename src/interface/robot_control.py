import time
from pymodbus.client import ModbusTcpClient
import numpy as np
from typing import Literal
from config.config_parameters import DataDict, default_results, STATES, REGISTERS
DataTypes = Literal["ANGLES", "FORCES", "POSITION"]


class RobotHandController:
    def __init__(self, ip: str = "192.168.123.211", port: int = 6000, mock: bool = False):
        """
        Read-only Modbus telemetry receiver for the RH56DFTP dexterous hand.
        """
        self.ip = ip
        self.port = port
        self.client: ModbusTcpClient | None = None
        self.is_connected = False
        self.current_angles = [1000] * 6
        self.angle_array = np.array(self.current_angles)
        self.last_sensor_data: dict[str, np.ndarray] = {}
        self.last_all_points = np.zeros((112, 17))
        self.offsets = {
                "Little": (150, 120),
                "Ring":   (230, 110),
                "Middle": (310, 100),
                "Index":  (390, 110),
                "Thumb":  (420, 360),
                "Palm":   (240, 520)
            }

        self.results: DataDict = default_results

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

            ("Palm", 4900, 8, 14, "Palm", (-15, 0)),
        ]
        self.sensor_offsets: dict[str, np.ndarray] = {}
        self.state: STATES = "starting"
        self.state_change: bool = False
        self.mean_matrix: np.ndarray = np.zeros((1, 17))
        self.all_points: np.ndarray = np.zeros((112, 17))

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

        self.state = "initialised"

    # ==========================================
    # Read-only telemetry access
    # ==========================================


    def read_command(self, type: DataTypes | str = "ANGLES", count: int = 6) -> list[int]:
        """
        Unified read access point for actuator telemetry and register inspection.
        `type` can be a standard data category ('ANGLES', 'FORCES', 'POSITION')
        or a specific register key from REGISTERS.
        """
        if type in self.receive_addresses:
            address = self.receive_addresses[type]
        elif type in REGISTERS:
            address = REGISTERS[type]
        else:
            raise ValueError(f"Unknown read target: {type}")

        if not self.is_connected or self.client is None:
            if type in ("ANGLES", "ANGLE_ACT"):
                return list(self.current_angles)
            return [0] * count

        res = self.client.read_holding_registers(address=address, count=count, device_id=1)
        if not res.isError():
            if type in ("ANGLES", "ANGLE_ACT"):
                self.current_angles = list(res.registers)
            return list(res.registers)

        print(f"[RobotHandController] Failed to read {type} from address {address}")
        return [-1] * count if type in ("ANGLES", "ANGLE_ACT") else [0] * count

    def read_data(self, target: DataTypes | str, count: int = 6) -> list[int]:
        """Compatibility wrapper for read_command."""
        return self.read_command(type=target, count=count)

    def safe_read(self, addr: int, length: int) -> list[int]:
        """Reads arbitrary holding registers in safe chunks of up to 120 registers."""
        if not self.is_connected or self.client is None:
            return [0] * length

        values = []
        for i in range(0, length, 120):
            res = self.client.read_holding_registers(
                address=addr + i,
                count=min(120, length - i),
                device_id=1,
            )
            if not res.isError():
                values.extend(res.registers)
        return values

    # ==========================================
    # Telemetry and local tactile calibration
    # ==========================================


    def get_angles(self) -> list[int]:
        """Reads actual joint angles for all 6 DOFs."""
        return self.read_command("ANGLES")


    def get_forces(self) -> list[int]:
        """Reads actual force values for all 6 DOFs."""
        return self.read_command("FORCES")


    def calibrate_offsets(self):
        """Calibrates baseline tactile sensor readings."""
        print("Calibrating sensor offsets...")
        time.sleep(1)
        for name, addr, h, w, key, rel in self.sensor_groups:
            data = self.safe_read(addr, h * w)
            self.sensor_offsets[name] = np.array(data) if data else np.zeros(h * w)


    def read_error(self):
        """Reads error code bitfields from actuators."""
        if not self.is_connected or self.client is None:
            return None
        return self.client.read_holding_registers(address=REGISTERS["ERROR"], count=6, device_id=1)

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
            offset = self.sensor_offsets.get(name, np.zeros(len(raw)))
            matrix = np.maximum(0, raw - offset)

            data_matrix[0, index] = np.mean(matrix)
            all_points[0:len(raw), index] = matrix
            raw_cache[name] = raw

        self.last_sensor_data = raw_cache
        self.last_all_points = all_points
        return data_matrix, all_points

    def build_result_data(self) -> DataDict:
        """Packages polled kinematics, telemetry, and tactile data into DataDict."""
        self.results = {
            "angles": np.array(self.read_command("ANGLES")),
            "forces": np.array(self.read_command("FORCES")),
            "position": np.array(self.read_command("POSITION")),
            "tactile_matrix": self.all_points,
            "tactile_mean_matrix": self.mean_matrix,
            "state": self.state,
            "state_change": self.state_change,
        }
        return self.results

    def poll(self) -> DataDict:
        """
        Pure hardware polling method.
        Acquires tactile matrices and joint telemetry without decision logic.
        """
        self.mean_matrix, self.all_points = self.read_tactile_data()
        current_pos = np.array(self.read_command())
        self.angle_array = current_pos
        return self.build_result_data()

    def main(self) -> DataDict:
        """Hardware acquisition entry point (alias for poll)."""
        return self.poll()


