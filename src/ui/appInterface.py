import tkinter as tk
import numpy as np
from PIL import Image, ImageTk
import os
from interface.robot_control import RobotHandController


class HandApp(tk.Tk):
    def __init__(self, robot: RobotHandController):
        super().__init__()
        self.robot = robot

        self.pixel = 6
        self.angle_labels: list[tk.Label] = []

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

    def setup_window(self):
        self.title("Hand Telemetry Receiver & Tactile Visualizer")
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
        self.state_label.pack(fill="x", pady=5)

        names = ["Little Finger", "Ring Finger", "Middle Finger", "Index Finger", "Thumb Bend", "Thumb Rot"]

        for i in range(6):
            f = tk.Frame(self.left_frame)
            f.pack()

            tk.Label(f, text=names[i], width=15).pack(side="left")

            label = tk.Label(f, text="—", width=10)
            label.pack(side="left")
            self.angle_labels.append(label)

        tk.Label(self.left_frame, text="Receive only · movement controlled externally").pack(pady=10)

    def update_angles_from_robot(self):
        for label, value in zip(self.angle_labels, self.robot.current_angles):
            label.config(text=str(value))

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

            self.canvas.create_oval(
                x0 + cx * self.pixel,
                y0 + cy * self.pixel,
                x0 + cx * self.pixel + size,
                y0 + cy * self.pixel + size,
                fill="blue"
            )

    def update_state_display(self, state: str):
        if hasattr(self, "state_label"):
            self.state_label.config(text=f"STATE: {state.upper()}")
            self.title(f"Hand Receiver - [{state.upper()}]")

    def main(self, state: str | None = None):
        if state:
            self.update_state_display(state)
        self.update_angles_from_robot()
        self.draw()
        try:
            self.update_idletasks()
            self.update()
        except tk.TclError:
            pass

