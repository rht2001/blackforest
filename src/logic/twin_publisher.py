"""Read-only telemetry export; never imports or opens a hardware controller."""
import json
import math
import threading
from urllib.request import ProxyHandler, Request, build_opener


REGIONS = [(f"{finger}_{zone}", shape)
           for finger in ("little", "ring", "middle", "index")
           for zone, shape in (("tip", (3, 3)), ("nail", (12, 8)), ("pad", (10, 8)))]
REGIONS += [(f"thumb_{zone}", shape) for zone, shape in
            (("tip", (3, 3)), ("nail", (12, 8)), ("mid", (3, 3)), ("pad", (12, 8)))]
REGIONS += [("palm", (8, 14))]


def number(value, low, high):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and low <= value <= high)


def vector(value, low, high):
    return (list(value) if isinstance(value, (list, tuple)) and len(value) == 6
            and all(number(v, low, high) for v in value) else None)


def robot_snapshot(robot, results):
    """Adapt main's DataDict and raw cache without hardware IO."""
    tactile = [[None] * len(REGIONS) for _ in range(112)]
    valid = [False] * len(REGIONS)
    for column, (_, (height, width)) in enumerate(REGIONS):
        if column >= len(robot.sensor_groups):
            continue
        name, _, source_height, source_width, *_rest = robot.sensor_groups[column]
        if (source_height, source_width) != (height, width):
            continue
        values = robot.last_sensor_data.get(name)
        if hasattr(values, "tolist"):
            values = values.tolist()
        if not isinstance(values, (list, tuple)) or len(values) != height * width:
            continue
        valid[column] = True
        for row, value in enumerate(values):
            tactile[row][column] = value
    angles = results.get("angles")
    forces = results.get("forces")
    if hasattr(angles, "tolist"):
        angles = angles.tolist()
    if hasattr(forces, "tolist"):
        forces = forces.tolist()
    if isinstance(forces, (list, tuple)):
        forces = [value - 65536 if number(value, 32768, 65535) else value
                  for value in forces]
    return {"simulated": not robot.is_connected, "angles": angles, "forces": forces,
            "tactile": tactile, "tactile_valid": valid}


def twin_payload(outputs):
    """Convert only valid regions; preserve palm wire order for the viewer."""
    points = outputs["tactile"]
    flags = outputs["tactile_valid"]
    if len(points) != 112 or any(len(row) != 17 for row in points) or len(flags) != 17:
        raise ValueError("Expected 112 x 17 tactile cells and 17 validity flags")
    regions, fingers = {}, {}
    for column, (key, (height, width)) in enumerate(REGIONS):
        if flags[column] is not True:
            continue
        values = [points[i][column] for i in range(height * width)]
        if not all(number(value, 0, 4095) for value in values):
            continue
        regions[key] = [values[i * width:(i + 1) * width] for i in range(height)]
        fingers.setdefault(key.split("_")[0], []).extend(values)
    return {
        "simulated": bool(outputs["simulated"]),
        # Explicit null clears previous pose values on a failed read.
        "angles_set": None,
        "angles_act": vector(outputs["angles"], 0, 1000),
        "force_act": vector(outputs["forces"], -4000, 4000),
        "tactile_regions": regions,
        "tactile": {key: {"max": max(values), "mean": sum(values) / len(values)}
                    for key, values in fingers.items()},
    }


class TwinPublisher:
    """One background worker, one pending frame; CSV still keeps every sample."""
    def __init__(self, url="http://127.0.0.1:18765/api/state"):
        self.url = url
        self._lock = threading.Lock()
        self._signal = threading.Event()
        self._pending = None
        self._closed = False
        self.sent_count = 0
        self.last_error = None
        self._opener = build_opener(ProxyHandler({}))
        self._worker = threading.Thread(target=self._run, daemon=True, name="TwinPublisher")
        self._worker.start()

    def submit(self, outputs):
        payload = json.dumps(twin_payload(outputs), allow_nan=False).encode("utf-8")
        with self._lock:
            if self._closed:
                return
            self._pending = payload
        self._signal.set()

    def _run(self):
        while True:
            self._signal.wait()
            with self._lock:
                self._signal.clear()
                payload, self._pending = self._pending, None
                closed = self._closed
            if payload is None:
                if closed:
                    return
                continue
            try:
                request = Request(self.url, payload, {"Content-Type": "application/json"}, method="POST")
                with self._opener.open(request, timeout=0.35) as response:
                    response.read()
                self.sent_count += 1
                self.last_error = None
            except Exception as error:
                self.last_error = str(error)
            with self._lock:
                if self._closed and self._pending is None:
                    return

    def close(self):
        with self._lock:
            self._closed = True
        self._signal.set()
        self._worker.join(timeout=1.0)
