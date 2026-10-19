import argparse
import time
import datetime
import tkinter as tk
from interface.robot_control import RobotHandController
from interface.robot_dataset import RobotDatabase
from logic.grasp_manager import GraspManager
from logic.grasp_capture import GraspCapture
from ui.appInterface import HandApp
from logic.twin_publisher import TwinPublisher, robot_snapshot
STEP_TIME = 0.03
def main():
    """
    Main function to run the Hand Telemetry Receiver & Tactile Visualizer application.
    Initializes the robot hand controller as access point of the TCP protocol and starts the Tkinter GUI application.
    Runs continously for continuous polling of GUI.
    """

    robot = RobotHandController(ip="192.168.123.210", port=6000)
    capture = GraspCapture()
    grasp_manager = GraspManager(on_grasp_closed=capture.submit)
    app = HandApp(robot)
    db = RobotDatabase()
    publisher = TwinPublisher("http://127.0.0.1:18765/api/state")

    is_running = True

    def on_close():
        nonlocal is_running
        is_running = False
        app.destroy()
        db.on_close()
        publisher.close()

    app.protocol("WM_DELETE_WINDOW", on_close)


    try:
        while is_running:
            try:
                start = time.monotonic()

                results_data = robot.poll()
                results_data = grasp_manager.step(robot, results_data)
                app.main(state=results_data.get("state"))
                db.main(data=results_data)
                if publisher is not None:
                    publisher.submit(robot_snapshot(robot, results_data))


                elapsed = time.monotonic() - start
                time.sleep(max(STEP_TIME - elapsed, 0))
            except tk.TclError:
                break
    finally:
        capture.close()
        if robot.client is not None:
            robot.client.close()

if __name__ == "__main__":
    main()
