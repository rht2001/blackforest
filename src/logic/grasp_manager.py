import time
import numpy as np
from typing import Callable, Any
from config.config_parameters import DataDict, STATES
from logic.grip_logic import confirm_grip, is_empty_grasp, get_pad_pressures


class GraspManager:
    """
    Orchestrates the robot hand grasp lifecycle:
    OPEN -> CLOSING -> SETTLING (1.0s continuous confirmation) -> CYLINDER_CONFIRMED.
    Transitions to RESETTING whenever an empty grasp or abort occurs.
    """

    def __init__(
        self,
        settle_duration: float = 1.0,
        settle_timeout: float = 3.0,
        angle_tolerance: int = 75,
        min_active_pads: int = 3,
        pad_force_threshold: float = 40.0,
        contact_trigger_threshold: float = 80.0,
        impulse_wait_time: float = 0.35,
        min_closing_time: float = 0.7,
        reset_cooldown: float = 1.5,
    ):
        self.settle_duration = settle_duration
        self.settle_timeout = settle_timeout
        self.angle_tolerance = angle_tolerance
        self.min_active_pads = min_active_pads
        self.pad_force_threshold = pad_force_threshold
        self.contact_trigger_threshold = contact_trigger_threshold
        self.impulse_wait_time = impulse_wait_time
        self.min_closing_time = min_closing_time
        self.reset_cooldown = reset_cooldown

        self.state: STATES = "open"
        self.last_angles: np.ndarray = np.array([1000] * 6)
        self.settle_start_time: float = 0.0
        self.closing_start_time: float = 0.0
        self.reset_start_time: float = 0.0
        self.reset_end_time: float = 0.0
        self.contact_start_time: float = 0.0
        self.contact_loss_start_time: float = 0.0
        self.confirmed_total_force: float = 0.0
        self.confirmed_forces: dict[str, float] = {}
        self.contact_triggered: bool = False
        self.contact_cleared: bool = True
        self.last_log_time: float = 0.0

        # Optional reference profiles for template validation
        self.valid_positions: np.ndarray | None = None
        self.valid_tactile: np.ndarray | None = None

        # Hook for Roadmap Task 2.5 (probing thumb)
        self.task_2_5_hook: Callable[[Any, DataDict], None] | None = None

        print("[GraspManager] Initialized. State: OPEN. Ready.")

    def set_reference_profiles(
        self,
        positions: np.ndarray | None = None,
        tactile: np.ndarray | None = None,
    ) -> None:
        """Sets optional reference profiles for strict template matching."""
        self.valid_positions = positions
        self.valid_tactile = tactile

    def reset(self, robot: Any = None) -> None:
        """
        Explicitly resets the grasp state machine to OPEN.
        Clears settling and closing timers, and applies the reset cooldown.
        If robot is provided, commands robot.open_hand().
        """
        if robot is not None:
            robot.open_hand()
        self.state = "open"
        self.settle_start_time = None
        self.closing_start_time = 0.0
        self.contact_loss_start_time = 0.0
        self.confirmed_total_force = 0.0
        self.confirmed_forces = {}
        self.reset_end_time = time.time()
        self.contact_cleared = False
        self.contact_triggered = False
        print("[GraspManager] Manual reset -> State: OPEN (re-armed)")

    def check_contact_trigger(self, tactile_matrix: np.ndarray) -> bool:
        """
        Detects sustained tactile contact using impulse rejection debounce.
        Requires contact to have been cleared before a new trigger can occur.
        """
        now = time.time()
        # Enforce post-reset cooldown
        if (now - self.reset_end_time) < self.reset_cooldown:
            return False

        contact_present = np.count_nonzero(tactile_matrix > self.contact_trigger_threshold) >= 10

        if not contact_present:
            self.contact_cleared = True
            self.contact_triggered = False
            return False

        # Only accept trigger if contact was previously cleared (edge-triggered)
        if not self.contact_cleared:
            return False

        if contact_present and not self.contact_triggered:
            self.contact_triggered = True
            self.contact_start_time = now
            return False

        if contact_present and self.contact_triggered:
            if (now - self.contact_start_time) >= self.impulse_wait_time:
                self.contact_triggered = False
                self.contact_cleared = False
                return True

        return False

    def on_cylinder_confirmed(self, robot: Any, data: DataDict, diagnostics: dict) -> None:
        """Triggered upon successful 1.0-second sustained cylinder grasp confirmation."""
        print(
            f"[GraspManager] *** CYLINDER CONFIRMED *** Grasp sustained for {self.settle_duration:.2f}s! "
            f"Spread: {diagnostics.get('spread')}, Active Fingers: {diagnostics.get('active_fingers')}, "
            f"Sensors: {diagnostics.get('active_sensors')}"
        )

        if self.task_2_5_hook is not None:
            print("[GraspManager] Triggering Roadmap Task 2.5 (Probing thumb feature detection)...")
            self.task_2_5_hook(robot, data)
        else:
            print("[GraspManager] Ready for Roadmap Task 2.5: No active thumb probing hook registered.")

    def step(self, robot: Any, data: DataDict) -> DataDict:
        """
        Per-frame state machine step.
        Evaluates telemetry and sensor matrices, mutates state, controls hand actuation.
        """
        now = time.time()
        current_angles = np.asarray(data["angles"])
        finger_angles = current_angles[:4]
        mean_matrix = np.asarray(data["tactile_mean_matrix"])
        tactile_matrix = np.asarray(data["tactile_matrix"])
        pad_pressures = get_pad_pressures(mean_matrix, tactile_matrix=tactile_matrix)

        initial_state = self.state
        state_changed = False

        diff = np.abs(current_angles - self.last_angles)

        # ---------------------------------------------------------
        # State: OPEN
        # ---------------------------------------------------------
        if self.state in ("open", "initialised", "starting"):
            self.state = "open"
            if self.check_contact_trigger(tactile_matrix):
                print("[GraspManager] Contact triggered -> State: CLOSING (latching fingers 0-3, thumb open)")
                self.state = "closing"
                self.closing_start_time = now
                state_changed = True
                # Latch fingers 0-3 only; thumb stays completely open (1000, 1000)
                robot.set_angles([0, 0, 0, 0, 1000, 1000])

        # ---------------------------------------------------------
        # State: CLOSING
        # ---------------------------------------------------------
        elif self.state == "closing":
            closing_elapsed = now - self.closing_start_time

            # Only evaluate steady state once physical motion has had time to start and slow down
            has_moved = np.any(finger_angles < 900)
            motion_halted = np.all(diff < 3) and (closing_elapsed >= self.min_closing_time) and has_moved

            if motion_halted:
                # Transition directly to settling to let compliance and tactile pressure develop
                self.state = "settling"
                self.settle_start_time = now
                self.last_log_time = now
                state_changed = True
                print(
                    f"[GraspManager] Hand motion stabilized (Angles: {finger_angles.tolist()}). "
                    f"Beginning stabilization & grip verification -> SETTLING"
                )

        # ---------------------------------------------------------
        # State: SETTLING (Continuous verification with settle_timeout)
        # ---------------------------------------------------------
        elif self.state == "settling":
            is_valid, reason, diagnostics = confirm_grip(
                finger_angles=finger_angles,
                tactile_mean_matrix=mean_matrix,
                tactile_matrix=tactile_matrix,
                valid_positions=self.valid_positions,
                valid_tactile=self.valid_tactile,
                angle_tolerance=self.angle_tolerance,
                min_active_pads=self.min_active_pads,
                pad_force_threshold=self.pad_force_threshold,
            )

            settling_elapsed = now - self.settle_start_time

            if is_valid:
                if (now - self.last_log_time) >= 0.25:
                    print(
                        f"[GraspManager] Settling confirmed... ({settling_elapsed:.2f}s/{self.settle_duration:.2f}s) | "
                        f"Angles: {finger_angles.tolist()} | Spread: {diagnostics.get('spread')} | "
                        f"Active Fingers: {diagnostics.get('active_fingers')} | "
                        f"Sensors: {diagnostics.get('active_sensors')}"
                    )
                    self.last_log_time = now

                if settling_elapsed >= self.settle_duration:
                    self.state = "cylinder_confirmed"
                    state_changed = True
                    self.confirmed_forces = {
                        f: pad_pressures.get(f, 0.0) for f in ("Little", "Ring", "Middle", "Index")
                    }
                    self.confirmed_total_force = sum(self.confirmed_forces.values())
                    self.contact_loss_start_time = 0.0
                    self.on_cylinder_confirmed(robot, data, diagnostics)
            else:
                if (now - self.last_log_time) >= 0.35:
                    print(
                        f"[GraspManager] Settling... ({reason}) | "
                        f"Active Fingers: {diagnostics.get('active_fingers')} | "
                        f"Sensors: {diagnostics.get('active_sensors')} | "
                        f"Pressures: {diagnostics.get('finger_pressures')}"
                    )
                    self.last_log_time = now


                # Timeout: if valid grip cannot be sustained within settle_timeout, abort to resetting
                if settling_elapsed >= self.settle_timeout:
                    print(
                        f"[GraspManager] Settling timeout after {self.settle_timeout:.1f}s ({reason}). "
                        f"Opening hand -> RESETTING"
                    )
                    self.state = "resetting"
                    self.reset_start_time = now
                    state_changed = True
                    robot.open_hand()

        # ---------------------------------------------------------
        # State: CYLINDER_CONFIRMED
        # ---------------------------------------------------------
        elif self.state == "cylinder_confirmed":
            primary_pressures = {f: pad_pressures.get(f, 0.0) for f in ("Little", "Ring", "Middle", "Index")}
            active_contact_fingers = sum(
                1 for p in primary_pressures.values() if p >= self.pad_force_threshold
            )
            total_finger_force = sum(primary_pressures.values())

            # Continuously track peak confirmed grasp force
            if total_finger_force > self.confirmed_total_force:
                self.confirmed_total_force = total_finger_force
                for f, p in primary_pressures.items():
                    self.confirmed_forces[f] = max(self.confirmed_forces.get(f, 0.0), p)

            # Check retention of individual fingers relative to peak hold force (at least 35% of peak)
            retained_fingers = sum(
                1 for f, p in primary_pressures.items()
                if p >= max(50.0, 0.35 * self.confirmed_forces.get(f, 0.0))
            )

            # Relative force drop threshold: drops below 45% of peak confirmed hold force, or below minimum floor
            rel_force_threshold = max(100.0, 0.45 * self.confirmed_total_force)
            force_dropped = bool(
                (total_finger_force < rel_force_threshold) or (retained_fingers < 2)
            )

            # Periodic telemetry for clear real-time visibility during cylinder hold
            if (now - self.last_log_time) >= 0.5:
                print(
                    f"[GraspManager] Cylinder hold active | "
                    f"Angles: {finger_angles.tolist()} | "
                    f"Active: {active_contact_fingers}/4 (Retained: {retained_fingers}/4) | "
                    f"Total: {total_finger_force:.1f}g (Peak: {self.confirmed_total_force:.1f}g) | "
                    f"Pressures: {dict((k, round(v, 1)) for k, v in primary_pressures.items())}"
                )
                self.last_log_time = now

            # 1. Hand opened detection (UI button, manual sliders, or open release)
            hand_opened = np.any(finger_angles > 350) or np.mean(finger_angles) > 250

            # 2. Empty fist / object slipped out detection
            # When the cylinder is removed, the commanded closed angles pull fingers into a fist (< 45)
            is_empty_fist = bool(np.all(finger_angles < 45) or np.mean(finger_angles) < 38)

            # 3. Palm crushed detection (fingers digging directly into palm with no cylinder)
            palm_val = pad_pressures.get("Palm", 0.0)
            is_palm_curled = bool(palm_val > 250.0 and np.mean(finger_angles) < 50)

            if hand_opened:
                print(f"[GraspManager] Hand opened (Angles: {finger_angles.tolist()}) -> Opening hand -> RESETTING")
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                state_changed = True
                robot.open_hand()
            elif is_empty_fist:
                print(f"[GraspManager] Contact lost: Object dropped / fingers closed into fist (Angles: {finger_angles.tolist()}) -> Opening hand -> RESETTING")
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                state_changed = True
                robot.open_hand()
            elif is_palm_curled:
                print(f"[GraspManager] Contact lost: Fingers curled into palm (Palm: {palm_val:.1f}g, Angles: {finger_angles.tolist()}) -> Opening hand -> RESETTING")
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                state_changed = True
                robot.open_hand()
            elif force_dropped:
                if self.contact_loss_start_time == 0.0:
                    self.contact_loss_start_time = now
                elif (now - self.contact_loss_start_time) >= 0.15:
                    print(
                        f"[GraspManager] Contact lost: Tactile force dropped "
                        f"(Total: {total_finger_force:.1f}g < {rel_force_threshold:.1f}g, "
                        f"Retained: {retained_fingers}/4 fingers, Peak: {self.confirmed_total_force:.1f}g) -> Opening hand -> RESETTING"
                    )
                    self.state = "resetting"
                    self.reset_start_time = now
                    self.contact_loss_start_time = 0.0
                    state_changed = True
                    robot.open_hand()
            else:
                self.contact_loss_start_time = 0.0


        # ---------------------------------------------------------
        # State: RESETTING
        # ---------------------------------------------------------
        elif self.state == "resetting":
            hand_opened = np.all(current_angles[:4] > 880)
            reset_timed_out = (now - self.reset_start_time) > 2.0

            if hand_opened or reset_timed_out:
                self.reset_end_time = now
                contact_still_on = np.count_nonzero(tactile_matrix > self.contact_trigger_threshold) >= 10
                self.contact_cleared = not contact_still_on
                self.contact_triggered = False
                self.state = "open"
                state_changed = True
                print(
                    f"[GraspManager] Hand reopened (Angles: {finger_angles.tolist()}). "
                    f"Cooldown {self.reset_cooldown:.1f}s active -> OPEN"
                )



        self.last_angles = current_angles
        data["state"] = self.state
        data["state_change"] = (initial_state != self.state) or state_changed
        return data
