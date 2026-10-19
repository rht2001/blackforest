"""Observe externally controlled grasps using telemetry; never actuate the hand."""
import time
from typing import Any, Callable

import numpy as np

from config.config_parameters import DataDict, STATES
from logic.grip_logic import confirm_grip, get_pad_pressures, is_empty_grasp


class GraspManager:
    """Detect stable closure, capture once, and wait for externally driven reopening."""

    def __init__(self, settle_duration: float = 1.0, settle_timeout: float = 3.0,
                 angle_tolerance: int = 75, min_active_pads: int = 3,
                 pad_force_threshold: float = 40.0,
                 contact_trigger_threshold: float = 80.0,
                 impulse_wait_time: float = 0.35, min_closing_time: float = 0.7,
                 reset_cooldown: float = 1.5,
                 on_grasp_closed: Callable[[], None] | None = None):
        self.settle_duration = settle_duration
        self.settle_timeout = settle_timeout
        self.angle_tolerance = angle_tolerance
        self.min_active_pads = min_active_pads
        self.pad_force_threshold = pad_force_threshold
        self.contact_trigger_threshold = contact_trigger_threshold
        self.impulse_wait_time = impulse_wait_time
        self.min_closing_time = min_closing_time
        self.reset_cooldown = reset_cooldown
        self.on_grasp_closed = on_grasp_closed
        self.state: STATES = "open"
        self.last_angles = None
        self.stable_since = None
        self.contact_since = None
        self.settle_start_time = 0.0
        self.valid_since = None
        self.reset_end_time = float("-inf")
        self.valid_positions = None
        self.valid_tactile = None

    def set_reference_profiles(self, positions=None, tactile=None):
        self.valid_positions = positions
        self.valid_tactile = tactile

    def step(self, robot: Any, data: DataDict) -> DataDict:
        """Classify received measurements. The robot argument is never called."""
        now = time.time()
        initial_state = self.state
        angles = np.asarray(data["angles"])
        if (angles.shape != (6,) or not np.all(np.isfinite(angles))
                or np.any(angles < 0) or np.any(angles > 1000)):
            self.last_angles = None
            self.stable_since = self.contact_since = self.valid_since = None
            data["state"], data["state_change"] = self.state, False
            return data

        tactile = np.asarray(data["tactile_matrix"])
        mean = np.asarray(data["tactile_mean_matrix"])
        contact = np.count_nonzero(tactile > self.contact_trigger_threshold) >= 10
        opened = np.all(angles[:4] > 880)
        stable = self.last_angles is not None and np.all(np.abs(angles - self.last_angles) < 3)

        if opened:
            if self.state != "open":
                self.reset_end_time = now
            self.state = "open"
            self.stable_since = self.contact_since = self.valid_since = None
        elif self.state == "open":
            # A receiver may attach after closure; detect that from measured pose/contact.
            if contact and now - self.reset_end_time >= self.reset_cooldown:
                self.state = "closing"
                self.contact_since = now
                self.stable_since = None
        elif self.state == "closing":
            if not contact:
                self.contact_since = self.stable_since = None
            else:
                if self.contact_since is None:
                    self.contact_since = now
                if stable:
                    if self.stable_since is None:
                        self.stable_since = now
                else:
                    self.stable_since = None
                if (self.stable_since is not None
                        and now - self.stable_since >= self.min_closing_time
                        and now - self.contact_since >= self.impulse_wait_time):
                    self.state = "settling"
                    self.settle_start_time = now
                    self.valid_since = None
                    if self.on_grasp_closed is not None:
                        try:
                            self.on_grasp_closed()
                        except Exception as exc:
                            print(f"[GraspManager] Taxel capture failed: {exc}")
        elif self.state == "settling":
            valid, _, _ = confirm_grip(
                finger_angles=angles[:4], tactile_mean_matrix=mean, tactile_matrix=tactile,
                valid_positions=self.valid_positions, valid_tactile=self.valid_tactile,
                angle_tolerance=self.angle_tolerance, min_active_pads=self.min_active_pads,
                pad_force_threshold=self.pad_force_threshold)
            if valid and stable:
                if self.valid_since is None:
                    self.valid_since = now
                if now - self.valid_since >= self.settle_duration:
                    self.state = "cylinder_confirmed"
            else:
                self.valid_since = None
            if self.state == "settling" and now - self.settle_start_time >= self.settle_timeout:
                self.state = "resetting"
        elif self.state == "cylinder_confirmed":
            if is_empty_grasp(angles[:4], get_pad_pressures(mean, tactile_matrix=tactile)):
                self.state = "resetting"
        # Resetting observes until an external controller opens the hand.

        self.last_angles = angles.copy()
        data["state"] = self.state
        data["state_change"] = self.state != initial_state
        return data
