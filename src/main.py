"""Tk acquisition loop with CSV persistence and orderly shutdown."""
import argparse
import logging
from pathlib import Path
from interface.robot_control import RobotHandController
from ui.appInterface import HandApp
from logic.robot_dataset import RobotDataset
from logic.robot_outputs import robot_outputs


class AcquisitionLoop:
    def __init__(self, app, robot, dataset, interval_ms=30):
        self.app, self.robot, self.dataset = app, robot, dataset
        self.interval_ms = interval_ms
        self.running = True
        self.callback = None
        app.protocol("WM_DELETE_WINDOW", self.on_close)

    def tick(self):
        self.callback = None
        if not self.running:
            return
        try:
            outputs = self.robot.main()
            self.dataset.loop(outputs)
            self.app.main(outputs)
        except Exception:
            logging.exception("Acquisition stopped; flushing recorded data")
            self.on_close()
            return
        self.callback = self.app.after(self.interval_ms, self.tick)

    def on_close(self):
        self.running = False
        if self.callback is not None:
            self.app.after_cancel(self.callback)
            self.callback = None
        # Keep window available for a close retry if saving fails.
        self.dataset.close()
        self.app.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.123.211")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--simulate", action="store_true",
                        help="Use mock data without attempting a hardware connection")
    parser.add_argument("--dataset", type=Path,
                        default=Path.cwd() / "main_dataset.csv")
    args = parser.parse_args()
    robot = None
    with RobotDataset(robot_outputs, args.dataset) as dataset:
        try:
            robot = RobotHandController(ip=args.ip, port=args.port, simulate=args.simulate)
            app = HandApp(robot)
            acquisition = AcquisitionLoop(app, robot, dataset)
            app.after(0, acquisition.tick)
            app.mainloop()
        finally:
            if robot is not None:
                robot.disconnect()


if __name__ == "__main__":
    main()
