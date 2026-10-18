"""Save one fresh digital-twin taxel snapshot per completed automatic closure."""
import csv
from datetime import datetime, timezone
from pathlib import Path
from queue import Queue
import threading
import time
from uuid import uuid4
from urllib.error import HTTPError

from taxel_coordinates import get_taxel_positions


class GraspCapture:
    def __init__(self, directory=None, base_url="http://127.0.0.1:18765", *, wait_timeout=3.0):
        self.directory = (Path(directory) if directory is not None else
                          Path(__file__).resolve().parents[2] / "grasp_captures")
        self.base_url = base_url
        self.wait_timeout = wait_timeout
        self._queue = Queue()
        self._worker = threading.Thread(target=self._run, name="GraspCapture", daemon=True)
        self._worker.start()

    def submit(self):
        """Called once after thumb motion settles; HTTP and disk IO stay off the robot loop."""
        self._queue.put(time.time())

    def close(self):
        self._queue.put(None)
        self._worker.join(timeout=self.wait_timeout + 1)

    def _run(self):
        while True:
            captured_after = self._queue.get()
            try:
                if captured_after is None:
                    return
                try:
                    path = self.save_snapshot(captured_after)
                    print(f"[GraspCapture] Saved {path}")
                except Exception as exc:
                    print(f"[GraspCapture] CSV not saved: {exc}")
            finally:
                self._queue.task_done()

    def save_snapshot(self, captured_after):
        """Require a viewer update received after closure; never save an older cached pose."""
        deadline = time.monotonic() + self.wait_timeout
        points = []
        error = "no fresh viewer taxels (keep the bridge and 3D viewer open)"
        while time.monotonic() < deadline:
            try:
                points = get_taxel_positions(self.base_url, timeout=min(0.5, self.wait_timeout),
                                             after=captured_after)
                if points:
                    break
            except HTTPError as exc:
                exc.close()
                if exc.code == 404:
                    raise RuntimeError("bridge has no /api/taxels?after endpoint; restart the updated "
                                       "digital_twin/bridge_server.py, then reload the 3D viewer") from exc
                error = str(exc)
            except (OSError, ValueError) as exc:
                error = str(exc)
            time.sleep(0.1)
        if not points:
            raise RuntimeError(error)

        self.directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.fromtimestamp(captured_after, timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        path = self.directory / f"grasp_{stamp}_{uuid4().hex[:8]}.csv"
        temporary = path.with_suffix(".tmp")
        try:
            with temporary.open("x", newline="", encoding="utf-8") as stream:
                writer = csv.DictWriter(stream, fieldnames=["id", "x", "y", "z", "pressure"])
                writer.writeheader()
                writer.writerows(points)
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
        return path
