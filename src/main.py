import argparse
import time
import datetime
import tkinter as tk
from interface.robot_control import RobotHandController
from interface.robot_dataset import RobotDatabase
from logic.grasp_manager import GraspManager
from logic.grasp_capture import GraspCapture
from ui.appInterface import HandApp
from ui.dashboardInterface import FullscreenHandApp
from logic.twin_publisher import TwinPublisher, robot_snapshot
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
    dashboard = FullscreenHandApp(
        robot,
        grasp_manager=grasp_manager,
        master=app,
        companion_app=app,
    )
    db = RobotDatabase()
    publisher = TwinPublisher("http://127.0.0.1:18765/api/state")

    is_running = True
    app_open = True
    dashboard_open = True

    def shutdown():
        nonlocal is_running
        if not is_running:
            return
        is_running = False
        try:
            if dashboard.winfo_exists():
                dashboard.destroy()
        except tk.TclError:
            pass
        try:
            app.destroy()
        except tk.TclError:
            pass
        db.on_close()
        publisher.close()

    def on_close_app():
        nonlocal app_open
        app_open = False
        if dashboard_open and dashboard.winfo_exists():
            app.withdraw()
        else:
            shutdown()

    def on_close_dashboard():
        nonlocal dashboard_open
        dashboard_open = False
        try:
            dashboard.destroy()
        except tk.TclError:
            pass
        if not app_open:
            shutdown()

    app.protocol("WM_DELETE_WINDOW", on_close_app)
    dashboard.protocol("WM_DELETE_WINDOW", on_close_dashboard)

    while is_running:
        try:
            start = time.monotonic()

            results_data = robot.poll()
            grasp_results = grasp_manager.step(robot, results_data)
            if app_open:
                app.main(state=grasp_results.get("state"), data=grasp_results)
                # Sync grasp_results in case a UI button callback (e.g. OPEN) ran inside app.update()
                grasp_results["state"] = grasp_manager.state
                grasp_results["probe_state"] = (
                    "probing"
                    if grasp_manager.state == "thumb_probing"
                    else grasp_manager.probe_state
                )
                grasp_results["probe_summary"] = grasp_manager.probe_summary
                grasp_results["slip_detected"] = (
                    grasp_manager.slip_detector.is_slipping
                    if grasp_manager.state == "cylinder_confirmed"
                    else False
                )
                grasp_results["slipping_contacts"] = (
                    grasp_manager.slip_detector.slipping_contacts
                    if grasp_manager.state == "cylinder_confirmed"
                    else []
                )
            if dashboard_open and dashboard.winfo_exists():
                dashboard.main(state=grasp_results.get("state"), data=grasp_results)
            db.main(data=grasp_results)
            if publisher is not None:
                publisher.submit(robot_snapshot(robot, grasp_results))

            elapsed = time.monotonic() - start
            time.sleep(max(STEP_TIME - elapsed, 0))
        except tk.TclError:
            break

if __name__ == "__main__":
    main()

