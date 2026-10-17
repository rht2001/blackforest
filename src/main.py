import time
import datetime
import tkinter as tk
from interface.robot_control import RobotHandController
from interface.robot_dataset import RobotDatabase
from ui.appInterface import HandApp
STEP_TIME = 0.03
def main():
    robot = RobotHandController(ip="192.168.123.211", port=6000)
    app = HandApp(robot)
    db = RobotDatabase()

    is_running = True

    def on_close():
        nonlocal is_running
        is_running = False
        app.destroy()

    app.protocol("WM_DELETE_WINDOW", on_close)

    while is_running:
        try:
            start = time.monotonic()

            app.main()
            results_data = robot.main()
            db.main(data=results_data)

            elapsed = time.monotonic() - start
            time.sleep(max(STEP_TIME - elapsed, 0))
        except tk.TclError:
            break


if __name__ == "__main__":
    main()