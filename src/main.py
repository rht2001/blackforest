import time
import datetime
import tkinter as tk
from interface.robot_control import RobotHandController
from interface.robot_dataset import RobotDatabase
from logic.grasp_manager import GraspManager
from ui.appInterface import HandApp
STEP_TIME = 0.03
def main():
    """
    Main function to run the Dexterous Hand Controller & Tactile Visualizer application.
    Initializes the robot hand controller as access point of the TCP protocol and starts the Tkinter GUI application.
    Runs continously for continuous polling of GUI.
    """
    robot = RobotHandController(ip="192.168.123.210", port=6000)
    grasp_manager = GraspManager()
    app = HandApp(robot, grasp_manager=grasp_manager)
    db = RobotDatabase()

    is_running = True

    def on_close():
        nonlocal is_running
        is_running = False
        app.destroy()
        db.on_close()

    app.protocol("WM_DELETE_WINDOW", on_close)

    while is_running:
        try:
            start = time.monotonic()

            results_data = robot.poll()
            results_data = grasp_manager.step(robot, results_data)
            app.main(state=results_data.get("state"))
            db.main(data=results_data)


            elapsed = time.monotonic() - start
            time.sleep(max(STEP_TIME - elapsed, 0))
        except tk.TclError:
            break


if __name__ == "__main__":
    main()