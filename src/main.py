import argparse
import time
import datetime
import tkinter as tk
from interface.robot_control import RobotHandController
from interface.robot_dataset import RobotDatabase
from logic.grasp_manager import GraspManager
from ui.appInterface import HandApp
from logic.twin_publisher import TwinPublisher, robot_snapshot
from logic.orientation_logic import TactilePoseAndSensorEstimator
from digital_twin.extract_tectile import reframe_to_palm_taxel
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
    geometric_tafel_positions = reframe_to_palm_taxel()
    orientation_check = TactilePoseAndSensorEstimator(robot)
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


    while is_running:
        try:
            start = time.monotonic()

            results_data = robot.poll()
            grasp_results = grasp_manager.step(robot, results_data)
            robot.results = grasp_results
            app.main(state=grasp_results.get("state"), data=grasp_results)
            geometric_tafel_positions = reframe_to_palm_taxel()
            orientation_check.main(kinematics=geometric_tafel_positions)
            db.main(data=grasp_results)
            if publisher is not None:
                publisher.submit(robot_snapshot(robot, grasp_results))


            elapsed = time.monotonic() - start
            time.sleep(max(STEP_TIME - elapsed, 0))
        except tk.TclError:
            break

if __name__ == "__main__":
    main()
