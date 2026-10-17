import json
import time
from pymodbus.client import ModbusTcpClient
import numpy as np
from typing import Literal
from config.config_parameters import REGISTERS, DataDict, default_results, STATES
DEFAULT_MOVEMENT_SPEED = [400] * 6


DataTypes = Literal["ANGLES", "FORCES", "POSITION"]


CLOSE_THRESHOLD = 50
IMPULSE_REJECTION_WAIT_TIME = 0.25
GRIP_ANGLE_THRESHOLD = 10  # Threshold for detecting if the hand is in a gripping position (0-1000 scale)

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
        self.contact_start_time = 0.0
        self.is_connected = False
        self.current_angles = [1000] * 6
        self.angle_array = np.array(self.current_angles)
        self.mock_angles = [1000] * 6
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

            ("Palm", 4900, 8, 14, "Palm", (-15, 0)),
        ]
        self.sensor_offsets: dict[str, np.ndarray] = {}
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
    # Unified Command Bus (read_command / send_command)
    # ==========================================

    def send_command(self, value: list[int], type: DataTypes | str = "ANGLES") -> bool:
        """
        Unified write access point for commands and register settings.
        `type` can be a standard data category ('ANGLES', 'FORCES', 'POSITION')
        or a specific register key from REGISTERS.
        """
        if type in self.send_addresses:
            address = self.send_addresses[type]
        elif type in REGISTERS:
            address = REGISTERS[type]
        else:
            raise ValueError(f"Unknown command target: {type}")

        if type in ("ANGLES", "ANGLE_SET"):
            self.current_angles = list(value)
            self.mock_angles = list(value)

        if not self.is_connected or self.client is None:
            return True

        res = self.client.write_registers(address=address, values=value, device_id=1)
        return not res.isError()

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
        return [0] * count

    def write_data(self, target: DataTypes | str, values: list[int]) -> bool:
        """Compatibility wrapper for send_command."""
        return self.send_command(value=values, type=target)

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
    # High-Level Semantic Facades
    # ==========================================

    def startup(self):
        """Initializes speed, opens hand, and calibrates offsets on startup."""
        self.set_default_speed()
        self.open_hand()
        if self.is_connected:
            self.calibrate_offsets()
        print("[RobotHandController] Hand opened and ready.")
        self.state = "initialised"

    def set_default_speed(self):
        """Sets the default speed for all 6 DOFs."""
        return self.send_command(DEFAULT_MOVEMENT_SPEED, "DEFAULT_SPEED_SET")

    def set_angles(self, angles: list[int]) -> bool:
        """Sets target angles for all 6 DOFs (0=closed, 1000=open)."""
        return self.send_command(angles, "ANGLES")

    def get_angles(self) -> list[int]:
        """Reads actual joint angles for all 6 DOFs."""
        return self.read_command("ANGLES")

    def set_forces(self, forces: list[int]) -> bool:
        """Sets the force control threshold for all 6 DOFs (0-3000g)."""
        return self.send_command(forces, "FORCES")

    def get_forces(self) -> list[int]:
        """Reads actual force values for all 6 DOFs."""
        return self.read_command("FORCES")

    def open_hand(self):
        """Opens all fingers fully (angle 1000) and re-arms automatic grip detection."""
        self.set_angles([1000] * 6)
        self.auto_mode_active = True
        self.auto_contact_trigerred = False
        self.contact_start_time = 0.0

    def close_hand(self):
        """Closes hand to standard grasp position and disarms automatic grip detection."""
        self.set_angles([0, 0, 0, 0, 234, 670])
        self.auto_mode_active = False
        self.auto_contact_trigerred = False
        self.contact_start_time = 0.0

    def calibrate_offsets(self):
        """Calibrates baseline tactile sensor readings."""
        print("Calibrating sensor offsets...")
        time.sleep(1)
        for name, addr, h, w, key, rel in self.sensor_groups:
            data = self.safe_read(addr, h * w)
            self.sensor_offsets[name] = np.array(data) if data else np.zeros(h * w)

    def close_on_contact(self, matrix: np.ndarray) -> bool:
        """Triggers grip closure if tactile pressure exceeds contact threshold with impulse rejection."""
        if not self.auto_mode_active:
            return False

        contact_present = np.count_nonzero(matrix > CLOSE_THRESHOLD) >= 10

        # Step 1: Contact detected for the first time -> start debounce timer
        if contact_present and not self.auto_contact_trigerred:
            self.auto_contact_trigerred = True
            self.contact_start_time = time.time()
            print("Found contact once, waiting for impulse rejection...")
            return False

        # Step 2: Contact lost before wait time elapsed -> reset (impulse rejected)
        if not contact_present and self.auto_contact_trigerred:
            self.auto_contact_trigerred = False
            return False

        # Step 3: Contact persisted longer than the rejection window -> confirm trigger
        if contact_present and self.auto_contact_trigerred:
            if time.time() - self.contact_start_time >= IMPULSE_REJECTION_WAIT_TIME:
                self.auto_mode_active = False
                self.auto_contact_trigerred = False
                self.set_angles([0, 0, 0, 0, 1000, 1000])
                print("Contact confirmed and sustained. Grip closed.")
                return True

        return False

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

    def build_result_data(self, mean_matrix: np.ndarray, all_points: np.ndarray) -> DataDict:
        """Packages polled kinematics, telemetry, and tactile data into DataDict."""
        self.results = {
            "angles": np.array(self.read_command("ANGLES")),
            "forces": np.array(self.read_command("FORCES")),
            "position": np.array(self.read_command("POSITION")),
            "tactile_matrix": all_points,
            "tactile_mean_matrix": mean_matrix,
            "state": self.state
        }


    def confirm_grip(self,valid_positions:np.ndarray, valid_tactile:np.ndarray,mean_matrix, angle_tolerance: int = 100, min_contact_fingers: int = 3) -> tuple[bool, str]:

        finger_angles = self.angle_array[0:4]  # Little, Ring, Middle, Index

        if np.any(finger_angles<50):
            return False, "Grasp missed: One or more fingers closed completely (empty grasp)."

        if all(a > 950 for a in finger_angles):
            return False, "Hand still open: No contact detected."

        spread = max(finger_angles) - min(finger_angles)
        if spread > angle_tolerance:
            return False, f"Non-homogeneous grasp: Angle spread {spread} exceeds tolerance {angle_tolerance}."



        mask_positions = np.abs(valid_positions - finger_angles) <= 0.05 * np.abs(valid_positions)
        mask_tactile = np.abs(valid_tactile - mean_matrix) <= 0.05 * np.abs(valid_tactile)

        all_within_tolerance = mask_positions.all() and mask_tactile.all()
        if all_within_tolerance:
            return True, "Stable cylindrical grasp confirmed."

        return False, "Stable grip not confirmed"

    def main(self)-> DataDict:
        mean_matrix, all_points = self.read_tactile_data()
        auto_contact = self.close_on_contact(all_points)
        current_pos = np.array(self.read_command())

        if auto_contact:
            self.state = "moving"

        if self.state == "moving":
        # TODO: Steady state detection only polls between two consecutive readings, which may not be sufficient for robust detection. Consider implementing a more sophisticated method that tracks the last N readings and checks for stability over that window.

            diff = np.abs(self.angle_array - np.array(current_pos))
            if np.all(diff < 2):
                self.state = "steady_state"
                print("steady state is achieved, trying to check position")

                # TODO: READ valid_positions and valid_tactile here after steady state is achieved.


                #self.confirm_grip(valid_positions, valid_tactile, mean_matrix=mean_matrix)

        self.angle_array = current_pos
        return self.results

