"""Tk acquisition loop with CSV persistence and orderly shutdown."""
import argparse
import logging
from pathlib import Path
from interface.robot_control import RobotHandController
from ui.appInterface import HandApp
from logic.robot_dataset import RobotDataset
from logic.robot_outputs import robot_outputs
from logic.twin_publisher import TwinPublisher


class AcquisitionLoop:
    def __init__(self, app, robot, dataset, interval_ms=30, publisher=None, state_dataset=None):
        self.app, self.robot, self.dataset = app, robot, dataset
        self.publisher = publisher
        self.state_dataset = state_dataset
        self.interval_ms = interval_ms
        self.running = True
        self.callback = None
        app.protocol("WM_DELETE_WINDOW", self.on_close)

    def tick(self):
        self.callback = None
        if not self.running:
            return
        try:
            outputs = self.robot.capture_telemetry()
            self.dataset.loop(outputs)
            if (self.state_dataset is not None and outputs["state_change"]
                    and outputs["state"] == "steady_state"):
                self.state_dataset.loop(outputs)
            if self.publisher is not None:
                self.publisher.submit(outputs)
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
        if self.state_dataset is not None:
            self.state_dataset.close()
        if self.publisher is not None:
            self.publisher.close()
        self.app.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="192.168.123.211")
    parser.add_argument("--port", type=int, default=6000)
    parser.add_argument("--simulate", action="store_true",
                        help="Use mock data without attempting a hardware connection")
    parser.add_argument("--dataset", type=Path,
                        default=Path.cwd() / "main_dataset.csv")
    parser.add_argument("--twin", action="store_true", help="Publish snapshots to the local digital twin")
    parser.add_argument("--state-changes", type=Path,
                        help="CSV for transitions into steady_state (default: next to dataset)")
    parser.add_argument("--twin-url", default="http://127.0.0.1:18765/api/state")
    args = parser.parse_args()
    robot = None
    publisher = None
    state_path = args.state_changes or args.dataset.with_name("state_changes.csv")
    if args.dataset.resolve() == state_path.resolve():
        parser.error("--dataset and --state-changes must use different files")
    with RobotDataset(robot_outputs, args.dataset) as dataset, RobotDataset(
            robot_outputs, state_path) as state_dataset:
        try:
            if args.twin:
                publisher = TwinPublisher(args.twin_url)
            robot = RobotHandController(ip=args.ip, port=args.port, simulate=args.simulate)
            app = HandApp(robot)
            acquisition = AcquisitionLoop(app, robot, dataset, publisher=publisher,
                                          state_dataset=state_dataset)
            app.after(0, acquisition.tick)
            app.mainloop()
        finally:
            if publisher is not None:
                publisher.close()
            if robot is not None:
                robot.disconnect()


if __name__ == "__main__":
    main()
