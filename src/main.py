"""Receive hand telemetry and publish it to the browser viewer without a desktop GUI."""
from contextlib import ExitStack
import time

from interface.robot_control import RobotHandController
from interface.robot_dataset import RobotDatabase
from logic.grasp_manager import GraspManager
from logic.grasp_capture import GraspCapture
from logic.twin_publisher import TwinPublisher, robot_snapshot

STEP_TIME = 0.03


def main():
    """Keep logging and browser telemetry running until the receiver is stopped."""
    with ExitStack() as resources:
        robot = RobotHandController(ip="192.168.123.210", port=6000)
        if robot.client is not None:
            resources.callback(robot.client.close)
        capture = GraspCapture()
        resources.callback(capture.close)
        grasp_manager = GraspManager(on_grasp_closed=capture.submit)
        db = RobotDatabase()
        resources.callback(db.on_close)
        publisher = TwinPublisher("http://127.0.0.1:18765/api/state")
        resources.callback(publisher.close)

        try:
            while True:
                start = time.monotonic()
                results_data = robot.poll()
                results_data = grasp_manager.step(robot, results_data)
                db.main(data=results_data)
                publisher.submit(robot_snapshot(robot, results_data))
                elapsed = time.monotonic() - start
                time.sleep(max(STEP_TIME - elapsed, 0))
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
