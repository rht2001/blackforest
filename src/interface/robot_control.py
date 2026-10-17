import json
import time
from pymodbus.client import ModbusTcpClient
import numpy as np
from typing import Literal, Dict

DEFAULT_MOVEMENT_SPEED = [400] * 6
CLOSE_THRESHOLD = 400
STATES = Literal["starting", "initialised", "moving", "steady_state", "opening"]
IMPULSE_REJECTION_WAIT_TIME = 0.5
IMPULSE_REJECTION_WAIT_TIME = 0.5
DataTypes = Literal["ANGLES", "FORCES", "POSITION"]
data_packet: tuple[int, int, int, int, int, int]
from config.config_parameters import REGISTERS


class RobotHandController:
    def __init__(self, ip="192.168.123.211", port=6000):
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
        for i in range(0, length, 120):
            res = self.client.read_holding_registers(
                address=addr + i,
                count=min(120, length - i),
                device_id=1
            )
            if not res.isError():
                values.extend(res.registers)
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

        contact_present = np.count_nonzero(matrix > CLOSE_THRESHOLD) >= 10

        # Step 1: Contact detected for the first time -> start debounce timer
        if contact_present and not self.auto_contact_trigerred:
            self.auto_contact_trigerred = True
            self.contact_start_time = time.time()
            print("Found contact once, waiting for impulse rejection...")
            time.sleep(0.5)
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
                self.send_command(value=[0, 0, 0, 0, 1000, 1000])
                print("Contact confirmed and sustained. Grip closed.")
                return True

        return False


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
            if type == "ANGLES":
                return list(self.mock_angles)
            return [0] * 6
        res = self.client.read_holding_registers(address=self.receive_addresses[type], count=6, device_id=1)
        if not res.isError():
            return res.registers
        print("Couldnt grab the data")
        return self.read_error()

    def receive_data(self, type: DataTypes = "ANGLES"):
        """Backward compatibility alias for read_command."""
        return self.read_command(type=type)

    def read_current_angles(self):
        if not self.is_connected:
            return list(self.mock_angles)
        res = self.client.read_holding_registers(address=1486, count=6, device_id=1)
        if not res.isError():
            return res.registers
        return None

    def read_tactile_data(self):
        # Max cells is 112 (Palm: 8 * 14 = 112)
        data_matrix = np.zeros((1, 17))
        all_points = np.zeros((112, 17))

        for index, (_, addr, h, w, key, rel) in enumerate(self.sensor_groups):
            data = self.safe_read(addr, h * w)
            if not data:
                continue

            raw = np.array(data)
            data_matrix[0, index] = np.mean(raw)
            all_points[0:len(raw), index] = raw
            offset = self.sensor_offsets.get(name, np.zeros(len(raw)))

            matrix = (raw - offset).reshape(h, w)
            matrix[matrix < 0] = 0

        return data_matrix, all_points

    def main(self):
        _, all_points = self.read_tactile_data()
        auto_contact = self.close_on_contact(all_points)
        current_pos = self.read_command()
