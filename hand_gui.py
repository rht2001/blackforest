import tkinter as tk
from pymodbus.client import ModbusTcpClient
import numpy as np
from PIL import Image, ImageTk
import os
import json
import time


class RobotHandController:
    def __init__(self, ip="192.168.123.211", port=6000):
        self.ip = ip
        self.port = port
        self.client = ModbusTcpClient(self.ip, port=self.port)
        self.client.connect()

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

    def read_current_angles(self):
        res = self.client.read_holding_registers(address=1486, count=6, device_id=1)
        if not res.isError():
            return res.registers
        return None


class HandApp:
    def __init__(self, root, robot: RobotHandController):
        self.root = root
        self.robot = robot

        self.SAVE_FILE = "positions.json"
        self.pixel = 6
        self.positions = {}
        self.sliders = []

        self.offsets = {
            "Little": (150, 120),
            "Ring":   (230, 110),
            "Middle": (310, 100),
            "Index":  (390, 110),
            "Thumb":  (420, 360),
            "Palm":   (240, 520)
        }

        self.setup_window()
        self.setup_ui()
        self.load_positions()

        self.root.after(500, self.update_angles_from_robot)
        self.root.after(1000, self.robot.calibrate_offsets)
        self.loop()

    def setup_window(self):
        self.root.title("Greifhand – mit Positionsspeicher")
        self.root.geometry("1200x900")

        self.left_frame = tk.Frame(self.root)
        self.left_frame.pack(side="left", fill="y", padx=10)

        self.right_frame = tk.Frame(self.root)
        self.right_frame.pack(side="right", fill="both", expand=True)

        img = Image.open(os.path.join(os.path.dirname(__file__), "hand.png"))
        img = img.resize((700, 900))
        self.hand_img = ImageTk.PhotoImage(img)

        self.canvas = tk.Canvas(self.right_frame, width=700, height=900)
        self.canvas.pack()

    def setup_ui(self):
        names = ["Kleiner Finger", "Ringfinger", "Mittelfinger", "Zeigefinger", "Daumen B", "Daumen R"]

        for i in range(6):
            f = tk.Frame(self.left_frame)
            f.pack()

            tk.Label(f, text=names[i], width=15).pack(side="left")

            s = tk.Scale(f, from_=0, to=1000, orient="horizontal", length=250)
            s.pack(side="left")
            s.bind("<ButtonRelease-1>", lambda e: self.send_angles())

            self.sliders.append(s)

        bf = tk.Frame(self.left_frame)
        bf.pack(pady=10)

        tk.Button(bf, text="Öffnen", command=self.öffnen).pack(side="left")
        tk.Button(bf, text="Schließen", command=self.schließen).pack(side="left")
        tk.Button(bf, text="Step +", command=self.step_plus).pack(side="left")
        tk.Button(bf, text="Step -", command=self.step_minus).pack(side="left")

        self.entry = tk.Entry(self.left_frame)
        self.entry.pack(fill="x", pady=5)

        tk.Button(self.left_frame, text="Position speichern", command=self.save_position).pack(fill="x")

        self.saved_frame = tk.Frame(self.left_frame)
        self.saved_frame.pack(fill="both", expand=True)

    def send_angles(self):
        self.robot.send_angles([s.get() for s in self.sliders])

    def update_angles_from_robot(self):
        registers = self.robot.read_current_angles()
        if registers:
            for i in range(6):
                self.sliders[i].set(registers[i])

    def öffnen(self):
        for s in self.sliders:
            s.set(1000)
        self.send_angles()

    def schließen(self):
        vals = [0, 0, 0, 0, 234, 670]
        for i in range(6):
            self.sliders[i].set(vals[i])
        self.send_angles()

    def step_plus(self):
        for s in self.sliders:
            s.set(min(1000, s.get() + 20))
        self.send_angles()

    def step_minus(self):
        for s in self.sliders:
            s.set(max(0, s.get() - 20))
        self.send_angles()

    def save_position(self):
        name = self.entry.get()
        if not name:
            return

        values = [s.get() for s in self.sliders]
        self.positions[name] = values
        self.save_to_file()
        self.create_button(name, values)

    def create_button(self, name, values):
        def apply():
            for i in range(6):
                self.sliders[i].set(values[i])
            self.send_angles()

        tk.Button(self.saved_frame, text=name, command=apply).pack(fill="x")

    def save_to_file(self):
        with open(self.SAVE_FILE, "w") as f:
            json.dump(self.positions, f)

    def load_positions(self):
        if not os.path.exists(self.SAVE_FILE):
            return

        with open(self.SAVE_FILE, "r") as f:
            data = json.load(f)

        for name, values in data.items():
            self.positions[name] = values
            self.create_button(name, values)

    def draw(self):
        self.canvas.delete("all")
        self.canvas.create_image(350, 450, image=self.hand_img)

        for name, addr, h, w, key, rel in self.robot.sensor_groups:
            data = self.robot.safe_read(addr, h * w)
            if not data:
                continue

            raw = np.array(data)
            offset = self.robot.sensor_offsets.get(name, np.zeros(len(raw)))

            matrix = (raw - offset).reshape(h, w)
            matrix[matrix < 0] = 0

            x0 = self.offsets[key][0] + rel[0]
            y0 = self.offsets[key][1] + rel[1]

            # Heatmap
            for i in range(h):
                for j in range(w):
                    v = matrix[i, j] / 4000

                    if v < 0.2:
                        color = "#00aa00"
                    elif v < 0.5:
                        color = "#cccc00"
                    elif v < 0.8:
                        color = "#ff8800"
                    else:
                        color = "#ff0000"

                    self.canvas.create_rectangle(
                        x0 + j * self.pixel, y0 + i * self.pixel,
                        x0 + (j + 1) * self.pixel, y0 + (i + 1) * self.pixel,
                        fill=color, width=0, stipple="gray25"
                    )

            # Robust Contact Point
            active = matrix > 50
            if np.sum(active) < 2:
                continue

            y_idx, x_idx = np.where(active)

            cx = int(np.mean(x_idx))
            cy = int(np.mean(y_idx))

            force = np.mean(matrix[active])
            size = int(3 + force / 200)

            self.canvas.create_oval(
                x0 + cx * self.pixel,
                y0 + cy * self.pixel,
                x0 + cx * self.pixel + size,
                y0 + cy * self.pixel + size,
                fill="blue"
            )

    def loop(self):
        self.draw()
        self.root.after(120, self.loop)


if __name__ == "__main__":
    robot = RobotHandController(ip="192.168.123.211", port=6000)
    root = tk.Tk()
    app = HandApp(root, robot)
    root.mainloop()