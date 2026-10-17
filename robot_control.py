
import json
import time
from pymodbus.client import ModbusTcpClient
import numpy as np
from typing import Literal, Dict

DEFAULT_MOVEMENT_SPEED = [100, 100, 100, 100, 100, 100]
CLOSE_THRESHOLD = 100

DataTypes = Literal["ANGLES", "FORCES", "POSITION"]
data_packet: tuple[int, int, int, int, int, int]
from config_parameters import REGISTERS
class RobotHandController:
    def __init__(self, ip="192.168.123.211", port=6000):
        self.ip = ip
        self.port = port
        self.client = ModbusTcpClient(self.ip, port=self.port)
        self.client.connect()
        self.sliders = []
        self.send_addresses = {
            "ANGLES": 1486,
            "FORCES": 1498,
            "POSITION": 1474,
        }

        self.auto_mode_active = True

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
        self.set_default_speed()
    def set_default_speed(self):
        self.send_all_commands("DEFAULT_SPEED_SET", value=DEFAULT_MOVEMENT_SPEED)
        pass
    def safe_read(self, addr, length):
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
        vals = [0, 0, 0, 0, 234, 670]

        self.send_angles(vals)

    def calibrate_offsets(self):
        print("Kalibriere Sensoren...")
        time.sleep(1)

        for name, addr, h, w, key, rel in self.sensor_groups:
            data = self.safe_read(addr, h * w)
            self.sensor_offsets[name] = np.array(data) if data else np.zeros(h * w)

    def send_angles(self, angles):
        self.client.write_registers(
            address=1486,
            values=angles,
            device_id=1
        )
    def close_on_contact(self, matrix):

        if not self.auto_mode_active:
            return False

        # At least 3 sensor pixels must exceed 100
        if np.count_nonzero(matrix > CLOSE_THRESHOLD) >= 3:
            self.auto_mode_active = False  # Prevent further triggers
            self.send_command(value=[0, 0, 0, 0, 1000, 1000])                   # Execute existing close function

            print(f"Contact detected at")
            return True

        return False
    def read_error(self):
            error = self.client.read_holding_registers(address=1606, count=6, device_id=1)
            return error
    def send_all_commands(self,register_name, value):
        self.client.write_registers(
                address=REGISTERS[register_name],
                values=value,
                device_id=1
            )
    def send_command(self, value:list[int], type:DataTypes= "ANGLES"):

        self.client.write_registers(
                    address=self.send_addresses[type],
                    values=value,
                    device_id=1
                )
    #TODO: check if the count works for reading the forces
    def receive_data(self, type:DataTypes= "ANGLES"):
        res = self.client.read_holding_registers(address=self.receive_addresses[type], count=6, device_id=1)
        if not res.isError():
            return res.registers

        print("Couldnt grab the data")
        return self.read_error()

    def read_current_angles(self):
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

        return data_matrix, all_points

    def main(self):
        _, all_points = self.read_tactile_data()
        self.close_on_contact(all_points)






