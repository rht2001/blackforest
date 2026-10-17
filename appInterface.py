import tkinter as tk
from pymodbus.client import ModbusTcpClient
import numpy as np
from PIL import Image, ImageTk
import os
import json
import time
from robot_control import RobotHandController
#TODO : provide a simulation environment to run it without the robot being connected
class HandApp(tk.Tk):
    def __init__(self, robot: RobotHandController):
        super().__init__()
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

    def setup_window(self):
        # 'self' is now the Tk window
        self.title("Greifhand – mit Positionsspeicher")
        self.geometry("1200x900")

        self.left_frame = tk.Frame(self)
        self.left_frame.pack(side="left", fill="y", padx=10)

        self.right_frame = tk.Frame(self)
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

            self.robot.sliders.append(s)

        bf = tk.Frame(self.left_frame)
        bf.pack(pady=10)

        tk.Button(bf, text="Öffnen", command=self.öffnen).pack(side="left")
        tk.Button(bf, text="Schließen", command=self.robot.close).pack(side="left")
        tk.Button(bf, text="Step +", command=self.step_plus).pack(side="left")
        tk.Button(bf, text="Step -", command=self.step_minus).pack(side="left")

        self.entry = tk.Entry(self.left_frame)
        self.entry.pack(fill="x", pady=5)

        tk.Button(self.left_frame, text="Position speichern", command=self.save_position).pack(fill="x")

        self.saved_frame = tk.Frame(self.left_frame)
        self.saved_frame.pack(fill="both", expand=True)

    def send_angles(self):
        self.robot.send_angles([s.get() for s in self.robot.sliders])

    def update_angles_from_robot(self):
        registers = self.robot.read_current_angles()
        if registers:
            for i in range(6):
                self.robot.sliders[i].set(registers[i])

    def öffnen(self):
        for s in self.robot.sliders:
            s.set(1000)
        self.send_angles()


    def step_plus(self):
        for s in self.robot.sliders:
            s.set(min(1000, s.get() + 20))
        self.send_angles()

    def step_minus(self):
        for s in self.robot.sliders:
            s.set(max(0, s.get() - 20))
        self.send_angles()

    def save_position(self):
        name = self.entry.get()
        if not name:
            return

        values = [s.get() for s in self.robot.sliders]
        self.positions[name] = values
        self.save_to_file()
        self.create_button(name, values)

    def create_button(self, name, values):
        def apply():
            for i in range(6):
                self.robot.sliders[i].set(values[i])
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

    def main(self):
        self.draw()
