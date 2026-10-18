import time
import threading
import numpy as np
from typing import Any, Callable


THUMB_SENSORS = {
    "Thumb Pad": {"col": 15, "shape": (12, 8)},
    "Thumb Tip": {"col": 12, "shape": (3, 3)},
    "Thumb Mid": {"col": 14, "shape": (3, 3)},
}


class ThumbProber:
    """
    Crystal-clear thumb probing protocol:
    - Outer loop: scans thumb rotation steps.
    - Inner loop: smoothly flexes thumb inward step-by-step.
    - Senses both tactile matrix increases and thumb actuator force delta.
    - Completely retracts to 1000 before every rotation move.
    """

    def __init__(
        self,
        rotation_steps: list[int] | None = None,
        retract_bend: int = 995,
        min_bend: int = 200,
        flex_step: int = 10,
        step_interval: float = 0.04,
        retract_wait: float = 0.5,
        low_force_limit: int = 150,
        thumb_speed: int = 180,
    ):
        # Rotation steps (reduced to 4 steps: omits 400 and 250 to minimize scan time, soft-limit at 995)
        self.rotation_steps = (
            rotation_steps if rotation_steps is not None else [995, 850, 700, 550]
        )
        self.retract_bend = retract_bend
        self.min_bend = min_bend
        self.flex_step = flex_step
        self.step_interval = step_interval
        self.retract_wait = retract_wait
        self.low_force_limit = low_force_limit
        self.thumb_speed = thumb_speed

        self.results: list[dict[str, Any]] = []
        self.classification: str = "none"
        self.summary: dict[str, Any] = {}
        self.is_active: bool = False
        self.is_finished: bool = False
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self.on_complete: Callable[[], None] | None = None

    def reset(self, robot: Any = None) -> None:
        """Stops any running scan, clears results, and restores hand defaults."""
        self.stop()
        self.results.clear()
        self.classification = "none"
        self.summary = {}
        self.is_active = False
        self.is_finished = False

        if robot is not None:
            robot.clear_errors()
            robot.set_forces([1500] * 6)
            robot.set_default_speed()

    def stop(self) -> None:
        """Signals the background probing loop to abort."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self.is_active = False

    def start(self, robot: Any, on_complete: Callable[[], None] | None = None) -> None:
        """Launches the probing routine in a dedicated worker thread."""
        self.stop()
        self.results.clear()
        self.classification = "probing"
        self.summary = {}
        self.is_active = True
        self.is_finished = False
        self._stop_event.clear()
        self.on_complete = on_complete

        self._thread = threading.Thread(
            target=self._run_probe_routine,
            args=(robot,),
            daemon=True,
            name="ThumbProberThread",
        )
        self._thread.start()

    @staticmethod
    def _get_tactile_points(robot: Any) -> np.ndarray:
        """Reuses fresh tactile matrix from main poll loop to avoid concurrent Modbus contention."""
        last_t = getattr(robot, "last_tactile_read_time", 0.0)
        if (time.time() - last_t) < 0.15 and hasattr(robot, "last_all_points"):
            return np.array(robot.last_all_points, copy=True)
        _, bp = robot.read_tactile_data()
        return bp

    def _retract_and_wait(self, robot: Any, timeout: float = 2.0) -> bool:
        """
        Retracts thumb bend to full extension (1000) while holding rotation unchanged (-1).
        Verifies via closed-loop feedback that actual bend reaches >= 900 before proceeding.
        """
        robot.set_angles([-1, -1, -1, -1, self.retract_bend, -1])
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._stop_event.is_set():
                return False
            angles = robot.get_angles()
            if angles and len(angles) > 4 and angles[4] >= 900:
                time.sleep(0.15)  # Stabilization settling time
                return True
            time.sleep(0.04)
        return False

    def _rotate_and_wait(self, robot: Any, target_rot: int, wait_time: float = 0.45) -> None:
        """
        Commands thumb rotation while holding bend at retracted position (-1).
        Pauses to ensure rotation is mechanically completed before flexing.
        """
        robot.set_angles([-1, -1, -1, -1, -1, target_rot])
        t0 = time.time()
        while time.time() - t0 < wait_time:
            if self._stop_event.is_set():
                return
            time.sleep(0.05)

    def detect_contact(
        self,
        current_points: np.ndarray,
        base_points: np.ndarray,
        current_thumb_force: float,
        baseline_thumb_force: float,
        actual_bend: int = 0,
    ) -> tuple[bool, str, float, float, float]:
        """
        Evaluates tactile matrices and actuator force delta.
        Returns: (has_contact, sensor_name, cop_u, cop_v, contact_force).
        """
        # Reject false triggers while thumb is at/near the retracted starting position
        if actual_bend >= 960:
            return False, "", 0.0, 0.0, 0.0

        # 1. Tactile matrix check across palmar thumb sensors with baseline subtraction
        for name, cfg in THUMB_SENSORS.items():
            col = cfg["col"]
            h, w = cfg["shape"]
            total_cells = h * w
            if col >= current_points.shape[1] or col >= base_points.shape[1]:
                continue

            curr_slice = current_points[:total_cells, col]
            base_slice = base_points[:total_cells, col]
            delta = np.maximum(0.0, curr_slice - base_slice)

            max_cell = float(np.max(delta))
            active_cells = delta[delta >= 15.0]
            sum_active = float(np.sum(active_cells))

            # Trigger on robust touch: single cell >= 30g, or multiple cells >= 45g total
            if max_cell >= 30.0 or (len(active_cells) >= 2 and sum_active >= 45.0):
                grid = delta.reshape(h, w)
                active_mask = grid >= 15.0
                weights = grid[active_mask]
                y_idx, x_idx = np.where(active_mask)
                total_w = float(np.sum(weights))
                cx = float(np.sum(x_idx * weights) / total_w) if total_w > 0 else 0.0
                cy = float(np.sum(y_idx * weights) / total_w) if total_w > 0 else 0.0
                return True, name, round(cx, 2), round(cy, 2), round(max(max_cell, sum_active), 1)

        # 2. Thumb actuator force delta check (catches physical resistance against object)
        delta_force = current_thumb_force - baseline_thumb_force
        if delta_force >= 35.0:
            return True, "Thumb Force Sensor", 0.0, 0.0, round(delta_force, 1)
        if current_thumb_force >= 80.0 and delta_force >= 15.0:
            return True, "Thumb Force Sensor", 0.0, 0.0, round(current_thumb_force, 1)

        return False, "", 0.0, 0.0, 0.0

    def classify_results(
        self, blocked_at_start: bool = False, blocked_bend: int | None = None
    ) -> tuple[str, dict[str, Any]]:
        """
        Distinguishes VEGABAR 38 (object present in thumb probing space) from
        VEGABAR 29 (free-air thumb flexion where ~35-36g is thumb weight/friction at Bend ~540).
        """
        if blocked_at_start:
            self.classification = "VEGABAR 38"
            self.summary = {
                "sensor_type": "VEGABAR 38",
                "mean_bend": float(blocked_bend if blocked_bend is not None else 850.0),
                "mean_force": float(self.low_force_limit),
                "object_hits": len(self.rotation_steps),
            }
            return self.classification, self.summary

        if not self.results:
            self.classification = "VEGABAR 29"
            self.summary = {
                "sensor_type": "VEGABAR 29",
                "mean_bend": float(self.min_bend),
                "mean_force": 0.0,
                "object_hits": 0,
            }
            return self.classification, self.summary

        bends = [float(r.get("bend_angle", 0.0)) for r in self.results]
        forces = [float(r.get("contact_force", 0.0)) for r in self.results]
        mean_bend = float(np.mean(bends))
        mean_force = float(np.mean(forces))

        object_hits = 0
        for r in self.results:
            is_tactile_hit = r.get("sensor_name", "Thumb Force Sensor") != "Thumb Force Sensor"
            is_early_bend = float(r.get("bend_angle", 0.0)) >= 700.0
            is_high_force = float(r.get("contact_force", 0.0)) >= 55.0
            is_preloaded = float(r.get("resting_force", 0.0)) >= 45.0
            if is_tactile_hit or is_early_bend or is_high_force or is_preloaded:
                object_hits += 1

        if object_hits >= 2 or mean_bend >= 700.0 or mean_force >= 55.0:
            self.classification = "VEGABAR 38"
        else:
            self.classification = "VEGABAR 29"

        self.summary = {
            "sensor_type": self.classification,
            "mean_bend": round(mean_bend, 1),
            "mean_force": round(mean_force, 1),
            "object_hits": object_hits,
        }
        return self.classification, self.summary

    def _run_probe_routine(self, robot: Any) -> None:
        """
        Pure nested-loop probing execution:
        Outer loop: Rotation (only rotated after verified full retraction)
        Inner loop: Flexion
        """
        blocked_at_start = False
        thumb_bend_angle: int | None = None
        try:
            print("[ThumbProber] Starting probing routine...")
            robot.clear_errors()

            # Primary fingers hold at 1500g, thumb actuators limited to gentle 150g
            robot.set_forces([1500, 1500, 1500, 1500, self.low_force_limit, self.low_force_limit])
            robot.set_speeds([400, 400, 400, 400, self.thumb_speed, 300])

            # 1. Closed-loop retraction to full extension
            retracted = self._retract_and_wait(robot, timeout=2.5)
            actual_angles = robot.get_angles()
            thumb_bend_angle = actual_angles[4] if actual_angles and len(actual_angles) > 4 else None

            # 2. Blockage detection: triggers ONLY if thumb physically failed to reach extension
            if not retracted or (thumb_bend_angle is not None and thumb_bend_angle < 880):
                blocked_at_start = True
                errors = robot.get_errors() if hasattr(robot, "get_errors") else []
                thumb_bend_err = errors[4] if len(errors) > 4 else 0
                thumb_rot_err = errors[5] if len(errors) > 5 else 0
                err_desc = []
                if thumb_bend_err:
                    err_desc.append(f"Bend Error: 0x{thumb_bend_err:04X}")
                if thumb_rot_err:
                    err_desc.append(f"Rot Error: 0x{thumb_rot_err:04X}")
                err_msg = ", ".join(err_desc) if err_desc else "Mechanical obstruction / stall"
                print(
                    f"[ThumbProber] WARNING: Thumb is blocked at start "
                    f"(Actual Bend={thumb_bend_angle} < 880, Status: {err_msg}). Aborting scan."
                )
                return

            # Clear any transient error flags from the cylindrical grasp once verified at full extension
            robot.clear_errors()

            # Outer loop: rotation scan
            for rot_idx, rot in enumerate(self.rotation_steps):
                if self._stop_event.is_set():
                    break

                # Step A: Ensure thumb is completely retracted before rotating
                self._retract_and_wait(robot, timeout=2.0)

                # Step B: Rotate thumb to target angle
                self._rotate_and_wait(robot, rot, wait_time=0.45)

                # Step C: Settle and tare baseline for THIS rotation angle at full extension
                time.sleep(0.2)
                # Sample 3 readings to obtain a rock-solid resting baseline (F_0)
                base_points_samples = []
                force_samples = []
                for _ in range(3):
                    bp = self._get_tactile_points(robot)
                    base_points_samples.append(bp)
                    f = robot.get_forces()
                    if len(f) > 4:
                        force_samples.append(float(f[4]))
                    time.sleep(0.03)

                base_points = np.mean(base_points_samples, axis=0) if base_points_samples else np.zeros((112, 17))
                baseline_thumb_force = float(np.mean(force_samples)) if force_samples else 0.0

                print(
                    f"[ThumbProber] Step {rot_idx + 1}/{len(self.rotation_steps)}: "
                    f"Rotation={rot}. Flexing inward (Resting F0: {baseline_thumb_force:.1f}g)..."
                )

                contact_found = False

                # Inner loop: flexion scan
                for bend in range(self.retract_bend, self.min_bend, -self.flex_step):
                    if self._stop_event.is_set():
                        break

                    # Step flexion inward (holding current rotation)
                    robot.set_angles([-1, -1, -1, -1, bend, rot])
                    time.sleep(self.step_interval)

                    # Probe actual angles, tactile matrix and thumb force
                    actual_angles = robot.get_angles()
                    actual_bend = actual_angles[4] if len(actual_angles) > 4 else bend
                    actual_rot = actual_angles[5] if len(actual_angles) > 5 else rot

                    curr_points = self._get_tactile_points(robot)
                    curr_forces = robot.get_forces()
                    thumb_f = float(curr_forces[4]) if len(curr_forces) > 4 else 0.0

                    contact, s_name, cx, cy, f_val = self.detect_contact(
                        curr_points, base_points, thumb_f, baseline_thumb_force, actual_bend=actual_bend
                    )

                    if contact:
                        record = {
                            "timestamp": time.time(),
                            "rotation_angle": actual_rot,
                            "bend_angle": actual_bend,
                            "sensor_name": s_name,
                            "cop_u": cx,
                            "cop_v": cy,
                            "contact_force": f_val,
                            "resting_force": round(baseline_thumb_force, 1),
                        }
                        self.results.append(record)
                        print(
                            f"[ThumbProber] Contact at Rot={actual_rot}, Bend={actual_bend} on {s_name} "
                            f"(CoP: [{cx}, {cy}], Force: {f_val}g)"
                        )
                        contact_found = True
                        break

                if not contact_found and not self._stop_event.is_set():
                    print(f"[ThumbProber] No contact reached at Rot={rot}. Retracting.")

                # Retract thumb bend completely after contact before next rotation
                robot.clear_errors()
                self._retract_and_wait(robot, timeout=2.0)

        finally:
            if not self._stop_event.is_set():
                sensor_label, summary = self.classify_results(
                    blocked_at_start=blocked_at_start,
                    blocked_bend=thumb_bend_angle,
                )
                print(
                    f"[ThumbProber] Classification: {sensor_label} "
                    f"(Mean Bend={summary.get('mean_bend')}, Mean Force={summary.get('mean_force')}g, "
                    f"Object Hits={summary.get('object_hits')}/{len(self.rotation_steps)})"
                )
            # Return thumb to full extension, restore standard speeds & forces
            robot.clear_errors()
            self._retract_and_wait(robot, timeout=2.0)
            robot.set_angles([-1, -1, -1, -1, 995, 995])
            robot.set_forces([1500] * 6)
            robot.set_default_speed()
            self.is_active = False
            self.is_finished = True
            print(
                f"[ThumbProber] Probing complete! Mapped {len(self.results)} contact points."
            )
            if self.on_complete:
                self.on_complete()
