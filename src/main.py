import time
import tkinter as tk
from interface.robot_control import RobotHandController
from ui.appInterface import HandApp


def main():
    robot = RobotHandController(ip="192.168.123.211", port=6000)
    app = HandApp(robot)

    is_running = True

    def on_close():
        nonlocal is_running
        is_running = False
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", on_close)

    while is_running:
        try:
            app.main()
            results_data = robot.main()
            time.sleep(0.03)  # ~30 FPS
        except tk.TclError:
            break


if __name__ == "__main__":
    main()