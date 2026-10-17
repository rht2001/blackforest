import tkinter as tk
import numpy as np
from PIL import Image, ImageTk
import os
import json
import time
from typing import Any
from interface.robot_control import RobotHandController


class HandApp(tk.Tk):
    def __init__(self, robot: RobotHandController, grasp_manager: Any = None):
        super().__init__()
        self.robot = robot
        self.grasp_manager = grasp_manager

        self.SAVE_FILE = "positions.json"
        self.pixel = 6
        self.positions = {}
        self.sliders: list[tk.Scale] = []

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
        self.title("Dexterous Hand Controller & Tactile Visualizer")
        self.geometry("1200x900")

        self.left_frame = tk.Frame(self)
        self.left_frame.pack(side="left", fill="y", padx=10)

        self.right_frame = tk.Frame(self)
        self.right_frame.pack(side="right", fill="both", expand=True)

        img_path = os.path.join(os.path.dirname(__file__), "hand.png")
        if not os.path.exists(img_path):
            img_path = os.path.join(os.getcwd(), "hand.png")
        img = Image.open(img_path)
        img = img.resize((700, 900))
        self.hand_img = ImageTk.PhotoImage(img)

        self.canvas = tk.Canvas(self.right_frame, width=700, height=900)
        self.canvas.pack()

    def setup_ui(self):
        self.state_label = tk.Label(
            self.left_frame,
            text="STATE: INITIALISED",
            font=("Helvetica", 13, "bold"),
            fg="#0055ff",
            relief="groove",
            padx=10,
            pady=4,
        )
        self.state_label.pack(fill="x", pady=(5, 2))

        self.slip_banner = tk.Label(
            self.left_frame,
            text="Grip: Stable",
            font=("Helvetica", 11, "bold"),
            fg="#2e7d32",
            bg="#f0f0f0",
            relief="groove",
            padx=10,
            pady=3,
        )
        self.slip_banner.pack(fill="x", pady=(0, 6))
        self.current_slipping_contacts: list[str] = []

        names = ["Little Finger", "Ring Finger", "Middle Finger", "Index Finger", "Thumb Bend", "Thumb Rot"]

        for i in range(6):
            f = tk.Frame(self.left_frame)
            f.pack()

            tk.Label(f, text=names[i], width=15).pack(side="left")

            s = tk.Scale(f, from_=0, to=1000, orient="horizontal", length=250)
            s.set(1000)
            s.pack(side="left")
            s.bind("<ButtonRelease-1>", lambda e: self.send_angles())

            self.sliders.append(s)

        bf = tk.Frame(self.left_frame)
        bf.pack(pady=10)

        tk.Button(bf, text="Open", command=self.open_hand).pack(side="left", padx=2)
        tk.Button(bf, text="Close", command=self.close_hand).pack(side="left", padx=2)
        tk.Button(bf, text="Probe Thumb", command=self.restart_probing).pack(side="left", padx=2)
        tk.Button(bf, text="Calibrate", command=self.calibrate_sensors).pack(side="left", padx=2)
        tk.Button(bf, text="Step +", command=self.step_plus).pack(side="left", padx=2)
        tk.Button(bf, text="Step -", command=self.step_minus).pack(side="left", padx=2)

        self.entry = tk.Entry(self.left_frame)
        self.entry.pack(fill="x", pady=5)

        tk.Button(self.left_frame, text="Save Position", command=self.save_position).pack(fill="x")

        self.saved_frame = tk.Frame(self.left_frame)
        self.saved_frame.pack(fill="both", expand=True)

    def send_angles(self):
        angles = [s.get() for s in self.sliders]
        self.robot.set_angles(angles)

    def update_angles_from_robot(self):
        registers = self.robot.get_angles()
        if registers:
            for i in range(min(6, len(registers))):
                self.sliders[i].set(registers[i])

    def open_hand(self):
        for s in self.sliders:
            s.set(1000)
        self.robot.open_hand()
        if self.grasp_manager is not None:
            self.grasp_manager.reset()
        self.update_slip_display(False, [])

    def calibrate_sensors(self):
        self.robot.calibrate_offsets()

    def close_hand(self):
        vals = [0, 0, 0, 0, 234, 670]
        for i in range(6):
            self.sliders[i].set(vals[i])
        self.robot.close_hand()

    def restart_probing(self):
        """Triggers or restarts thumb probing on demand."""
        if self.grasp_manager is not None:
            self.grasp_manager.restart_probing(self.robot)

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
            # Prefer cached tactile data from robot loop to avoid redundant Modbus traffic
            if name in self.robot.last_sensor_data:
                raw = self.robot.last_sensor_data[name]
            else:
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

            is_slipping = name in getattr(self, "current_slipping_contacts", [])
            circle_color = "#ff2222" if is_slipping else "#0055ff"
            outline_color = "#ffcc00" if is_slipping else ""

            self.canvas.create_oval(
                x0 + cx * self.pixel,
                y0 + cy * self.pixel,
                x0 + cx * self.pixel + size,
                y0 + cy * self.pixel + size,
                fill=circle_color,
                outline=outline_color,
                width=2 if is_slipping else 0,
            )

    def update_state_display(self, state: str):
        if hasattr(self, "state_label"):
            state_lower = state.lower()
            fg_color = "#0055ff"
            if state_lower == "cylinder_confirmed":
                fg_color = "#008800"
            elif state_lower == "thumb_probing":
                fg_color = "#8b5cf6"
            elif state_lower == "probing_complete":
                fg_color = "#059669"
            elif state_lower == "resetting":
                fg_color = "#e11d48"

            self.state_label.config(text=f"STATE: {state.upper()}", fg=fg_color)
            self.title(f"Hand Controller - [{state.upper()}]")

    def update_slip_display(self, slip_detected: bool, slipping_contacts: list[str]):
        if not hasattr(self, "slip_banner"):
            return
        self.current_slipping_contacts = slipping_contacts
        if slip_detected and slipping_contacts:
            contact_str = ", ".join(slipping_contacts)
            self.slip_banner.config(
                text=f"⚠️ SLIP DETECTED: {contact_str}",
                fg="#ffffff",
                bg="#d32f2f",
            )
        else:
            self.slip_banner.config(
                text="Grip: Stable",
                fg="#2e7d32",
                bg="#f0f0f0",
            )

    def main(self, state: str | None = None, data: Any = None):
        if data is not None:
            if state is None:
                state = data.get("state")
            self.update_slip_display(
                data.get("slip_detected", False),
                data.get("slipping_contacts", []),
            )
        if state:
            self.update_state_display(state)
        self.draw()
        try:
            self.update_idletasks()
            self.update()
        except tk.TclError:
            pass

