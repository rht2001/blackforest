import time
import tkinter as tk
from interface.robot_control import RobotHandController
from ui.appInterface import HandApp


def main():
    """
    Main function to run the Dexterous Hand Controller & Tactile Visualizer application.
    Initializes the robot hand controller as access point of the TCP protocol and starts the Tkinter GUI application.
    Runs continously for continuous polling of GUI.
    """
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