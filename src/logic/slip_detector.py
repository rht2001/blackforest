import time
import numpy as np
from typing import Any
from collections import deque


MONITORED_SENSORS = {
    "Palm": {"col": 16, "shape": (8, 14)},
    "Little Nail": {"col": 1, "shape": (12, 8)},
    "Ring Nail": {"col": 4, "shape": (12, 8)},
    "Middle Nail": {"col": 7, "shape": (12, 8)},
    "Index Nail": {"col": 10, "shape": (12, 8)},
}


class TactileSlipDetector:
    """
    Monitors spatial Center of Pressure (CoP) displacement and force rate-of-change (dF/dt)
    across the Palm and primary finger Nail tactile patches to detect object slipping
    during confirmed cylindrical grasps.
    """

    def __init__(
        self,
        cop_drift_threshold: float = 0.45,
        df_dt_threshold: float = -150.0,
        cell_threshold: float = 30.0,
        min_active_cells: int = 1,
        min_patch_force: float = 30.0,
        stabilize_time: float = 0.4,
        window_duration: float = 0.25,
        ema_alpha: float = 0.6,
    ):
        self.cop_drift_threshold = cop_drift_threshold
        self.df_dt_threshold = df_dt_threshold
        self.cell_threshold = cell_threshold
        self.min_active_cells = min_active_cells
        self.min_patch_force = min_patch_force
        self.stabilize_time = stabilize_time
        self.window_duration = window_duration
        self.ema_alpha = ema_alpha

        # History per sensor: deque of (timestamp, cx, cy, total_force)
        self.history: dict[str, deque[tuple[float, float, float, float]]] = {
            name: deque(maxlen=20) for name in MONITORED_SENSORS
        }
        self.smooth_cop: dict[str, tuple[float, float]] = {}
        self.global_force_history: deque[tuple[float, float]] = deque(maxlen=20)

        self.is_slipping: bool = False
        self.slipping_contacts: list[str] = []
        self.last_slip_time: float = 0.0
        self.diagnostics: dict[str, Any] = {}

    def reset(self) -> None:
        """Clears all historical trajectory buffers and resets slip state."""
        for d in self.history.values():
            d.clear()
        self.smooth_cop.clear()
        self.global_force_history.clear()
        self.is_slipping = False
        self.slipping_contacts = []
        self.last_slip_time = 0.0
        self.diagnostics = {}

    def compute_cop(
        self, grid: np.ndarray
    ) -> tuple[float | None, float | None, float, int]:
        """
        Computes weighted Center of Pressure (cx, cy) and total force for active cells.
        Returns: (cx, cy, total_force, num_active_cells).
        If insufficient active cells or low force, cx and cy are None.
        """
        active = grid > self.cell_threshold
        num_active = int(np.count_nonzero(active))
        if num_active < self.min_active_cells:
            return None, None, float(np.sum(grid[active])) if num_active > 0 else 0.0, num_active

        weights = grid[active]
        total_force = float(np.sum(weights))
        if total_force < self.min_patch_force:
            return None, None, total_force, num_active

        y_indices, x_indices = np.where(active)
        cx = float(np.sum(x_indices * weights) / total_force)
        cy = float(np.sum(y_indices * weights) / total_force)
        return cx, cy, total_force, num_active

    def update(
        self, tactile_matrix: np.ndarray, data: dict[str, Any] | None = None
    ) -> tuple[bool, list[str], dict[str, Any]]:
        """
        Processes tactile matrix, evaluates EMA-smoothed CoP drift and force rate-of-change,
        and returns (is_slipping, slipping_contacts, diagnostics).
        """
        now = time.time()
        active_slip_contacts: list[str] = []
        diag_per_sensor: dict[str, Any] = {}

        for name, cfg in MONITORED_SENSORS.items():
            col = cfg["col"]
            h, w = cfg["shape"]
            if col >= tactile_matrix.shape[1]:
                continue

            raw_slice = tactile_matrix[0 : h * w, col]
            if len(raw_slice) < h * w:
                continue

            grid = raw_slice.reshape(h, w)
            cx, cy, total_f, n_cells = self.compute_cop(grid)

            diag_per_sensor[name] = {
                "active_cells": n_cells,
                "total_force": round(total_f, 1),
                "cop": (round(cx, 2), round(cy, 2)) if cx is not None else None,
                "slip_reason": None,
            }

            if cx is None or cy is None:
                self.smooth_cop.pop(name, None)
                continue

            # Apply Exponential Moving Average (EMA) to suppress high-frequency jitter
            if name in self.smooth_cop:
                prev_cx, prev_cy = self.smooth_cop[name]
                scx = self.ema_alpha * cx + (1.0 - self.ema_alpha) * prev_cx
                scy = self.ema_alpha * cy + (1.0 - self.ema_alpha) * prev_cy
            else:
                scx, scy = cx, cy

            self.smooth_cop[name] = (scx, scy)

            q = self.history[name]
            q.append((now, scx, scy, total_f))

            # Prune samples older than window_duration
            while len(q) > 1 and (now - q[0][0]) > self.window_duration:
                q.popleft()

            if len(q) >= 3:
                t0, cx0, cy0, f0 = q[0]
                dt = now - t0
                if dt >= 0.10:
                    # Spatial displacement on smoothed CoP
                    dx = scx - cx0
                    dy = scy - cy0
                    cop_displacement = float(np.hypot(dx, dy))

                    # Force rate-of-change and relative drop
                    df_dt = float((total_f - f0) / dt)
                    rel_force_drop = float((f0 - total_f) / max(f0, 1.0))

                    diag_per_sensor[name]["cop_displacement"] = round(cop_displacement, 2)
                    diag_per_sensor[name]["df_dt"] = round(df_dt, 1)

                    # Slip condition: spatial drift or notable relative force unloading
                    cop_slipped = cop_displacement >= self.cop_drift_threshold
                    force_slipped = (
                        (df_dt <= self.df_dt_threshold and rel_force_drop >= 0.08)
                        or (rel_force_drop >= 0.12)
                    )

                    if cop_slipped or force_slipped:
                        active_slip_contacts.append(name)
                        reason = (
                            f"CoP drift ({cop_displacement:.1f}c)"
                            if cop_slipped
                            else f"Force drop ({df_dt:.0f}g/s, {rel_force_drop*100:.0f}%)"
                        )
                        diag_per_sensor[name]["slip_reason"] = reason

        # Early global slip: detect dynamic grip unloading (>= 400g/s and >= 12% drop over 200ms window)
        # Note: Slow stress relaxation occurs at < 40g/s and will NOT trigger this check
        if data is not None:
            curr_f = float(data.get("total_finger_force", 0.0))
            if curr_f > 100.0:
                self.global_force_history.append((now, curr_f))
                while (
                    len(self.global_force_history) > 1
                    and (now - self.global_force_history[0][0]) > self.window_duration
                ):
                    self.global_force_history.popleft()

                if len(self.global_force_history) >= 3:
                    gt0, gf0 = self.global_force_history[0]
                    g_dt = now - gt0
                    if g_dt >= 0.10:
                        g_df_dt = float((curr_f - gf0) / g_dt)
                        g_rel_drop = float((gf0 - curr_f) / max(gf0, 1.0))
                        if (g_df_dt <= -400.0) and (g_rel_drop >= 0.12):
                            if "Grip Force" not in active_slip_contacts:
                                active_slip_contacts.append("Grip Force")
                            diag_per_sensor["Grip Force"] = {
                                "slip_reason": f"Rapid grip pull ({g_df_dt:.0f}g/s, {g_rel_drop*100:.0f}%)"
                            }

        # State transition: immediate trigger on genuine smoothed motion
        if len(active_slip_contacts) > 0:
            self.is_slipping = True
            self.slipping_contacts = active_slip_contacts
            self.last_slip_time = now
        else:
            # Auto-clear after stabilize_time
            if self.is_slipping and (now - self.last_slip_time) >= self.stabilize_time:
                self.is_slipping = False
                self.slipping_contacts = []

        self.diagnostics = {
            "is_slipping": self.is_slipping,
            "slipping_contacts": self.slipping_contacts,
            "sensors": diag_per_sensor,
        }

        return self.is_slipping, self.slipping_contacts, self.diagnostics
