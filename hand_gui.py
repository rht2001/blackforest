import time
from robot_control import RobotHandController
from appInterface import HandApp
import tkinter as tk
def on_close():
        global is_running
        is_running = False
        app.destroy()

if __name__ == "__main__":

    robot = RobotHandController(ip="192.168.123.211", port=6000)
    app = HandApp(robot)

    is_running = True
    app.protocol("WM_DELETE_WINDOW", on_close)

    while is_running:
        app.main()
        robot.main()

        time.sleep(0.03)  # ~30 FPS