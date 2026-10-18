import time
import numpy as np
from typing import Callable, Any
from config.config_parameters import DataDict, STATES
from logic.grip_logic import confirm_grip, is_empty_grasp, get_pad_pressures
from logic.slip_detector import TactileSlipDetector
from logic.thumb_prober import ThumbProber


class GraspManager:
    """Orchestrates the finite-state lifecycle of a robotic hand grasp.

    State Flow:
        OPEN -> CLOSING -> SETTLING -> CYLINDER_CONFIRMED -> THUMB_PROBING.
        Transitions to RESETTING upon empty grasps, object drop, manual releases,
        or sensor timeouts.

    Key Capabilities:
        - Debounced, edge-triggered contact detection from tactile matrices.
        - Compliance settling verification with geometric and force constraints.
        - Grasp force retention tracking and drop detection during hold.
        - Real-time tactile slip detection and callback dispatching.
        - Integrated thumb probing routines for surface feature detection.
    """

    def __init__(
        self,
        settle_duration: float = 1.0,
        settle_timeout: float = 3.0,
        angle_tolerance: int = 75,
        min_active_pads: int = 3,
        pad_force_threshold: float = 60.0,
        contact_trigger_threshold: float = 80.0,
        impulse_wait_time: float = 0.35,
        min_closing_time: float = 0.7,
        reset_cooldown: float = 1.5,
    ) -> None:
        """Initializes the grasp state machine parameters and tracking attributes.

        Args:
            settle_duration: Required duration (seconds) of stable tactile contact
                before confirming a cylinder grasp.
            settle_timeout: Maximum time (seconds) allowed in the SETTLING state before
                aborting to RESETTING.
            angle_tolerance: Angular variance allowed across fingers for alignment.
            min_active_pads: Minimum number of finger pads that must register contact.
            pad_force_threshold: Minimum tactile force (grams) required to count a pad as active.
            contact_trigger_threshold: Tactile matrix threshold used to detect contact initiation.
            impulse_wait_time: Debounce duration (seconds) to reject transient contact impulses.
            min_closing_time: Minimum time (seconds) to let fingers move before evaluating steady state.
            reset_cooldown: Duration (seconds) to block new triggers after releasing the hand.
        """
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

        self.slip_detector = TactileSlipDetector()
        self.was_slipping: bool = False
        self.slip_callback: Callable[[list[str], dict], None] | None = None

        self.thumb_prober = ThumbProber()
        self.probing_results: list[dict] = []
        self.probe_state: str = "none"
        self.probe_summary: dict[str, Any] = {}
        self.no_slip_start_time: float = 0.0
        self.no_slip_stability_duration: float = 0.4
        self.has_probed: bool = False

        self.valid_positions: np.ndarray | None = None
        self.valid_tactile: np.ndarray | None = None

        self.task_2_5_hook: Callable[[Any, DataDict], None] | None = None

        print("[GraspManager] Init | state=OPEN")

    def set_reference_profiles(
        self,
        positions: np.ndarray | None = None,
        tactile: np.ndarray | None = None,
    ) -> None:
        """Sets optional reference profiles for strict template matching.

        Args:
            positions: Expected joint angles for a reference grasp profile.
            tactile: Expected tactile matrix for a reference grasp profile.
        """
        self.valid_positions = positions
        self.valid_tactile = tactile

    def register_slip_callback(self, callback: Callable[[list[str], dict], None]) -> None:
        """Registers a callback hook invoked when slipping is detected.

        Args:
            callback: Callable accepting a list of slipping sensor labels and
                a diagnostics dictionary containing sensor metrics.
        """
        self.slip_callback = callback

    def reset(self, robot: Any = None) -> None:
        """Resets the state machine to OPEN and re-arms sensors.

        Clears settling and closing timers, drops confirmed force levels, resets
        the slip detector and thumb prober, and initiates the post-reset cooldown.

        Args:
            robot: Optional robot controller interface. If provided, commands
                the robot hand to fully open.
        """
        self.thumb_prober.reset(robot)
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
        self.slip_detector.reset()
        self.was_slipping = False
        self.probing_results.clear()
        self.probe_state = "none"
        self.probe_summary = {}
        self.no_slip_start_time = 0.0
        self.has_probed = False
        print("[GraspManager] Reset | state=OPEN")

    def check_contact_trigger(self, tactile_matrix: np.ndarray) -> bool:
        """Detects sustained tactile contact using impulse rejection debounce.

        Requires tactile contact to have cleared previously before allowing a new
        trigger edge. Suppresses triggers during post-reset cooldown and requires
        pressure to remain sustained for at least `impulse_wait_time`.

        Args:
            tactile_matrix: Current tactile sensor matrix from telemetry.

        Returns:
            True if sustained contact is confirmed; False otherwise.
        """
        now = time.time()
        if (now - self.reset_end_time) < self.reset_cooldown:
            return False

        contact_present = np.count_nonzero(tactile_matrix > self.contact_trigger_threshold) >= 10

        if not contact_present:
            self.contact_cleared = True
            self.contact_triggered = False
            return False

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
        """Handles post-confirmation actions once a cylinder grasp is verified.

        Invoked when grip confirmation criteria have been continuously satisfied
        for `settle_duration`. Emits structured telemetry and triggers registered
        probing hooks if available.

        Args:
            robot: Active robot controller interface.
            data: Current telemetry dictionary.
            diagnostics: Grip diagnostics dictionary containing active fingers,
                spread, and active sensor counts.
        """
        print(
            f"[GraspManager] Confirmed | hold={self.settle_duration:.1f}s "
            f"spread={diagnostics.get('spread')} "
            f"fingers={diagnostics.get('active_fingers')} "
            f"sensors={diagnostics.get('active_sensors')}"
        )

        if self.task_2_5_hook is not None:
            print("[GraspManager] Probing | triggering Task 2.5 hook")
            self.task_2_5_hook(robot, data)
        else:
            print("[GraspManager] Probing | no Task 2.5 hook registered")

    def step(self, robot: Any, data: DataDict) -> DataDict:
        """Executes a single state machine update tick.

        Evaluates current finger angles and tactile feedback matrices to drive
        transitions between OPEN, CLOSING, SETTLING, CYLINDER_CONFIRMED,
        THUMB_PROBING, and RESETTING. Commands robot actuation and populates
        the telemetry dictionary with updated state, slip metrics, and probing results.

        Args:
            robot: Robot hand controller interface.
            data: Telemetry dictionary containing current joint angles and tactile matrices.

        Returns:
            The input telemetry dictionary augmented with state machine outputs.
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

        # State: OPEN
        if self.state in ("open", "initialised", "starting"):
            self.state = "open"
            if (now - self.reset_end_time) >= self.reset_cooldown and is_empty_grasp(
                finger_angles, pad_pressures
            ):
                print(
                    f"[GraspManager] State: OPEN -> RESETTING | recovering closed empty hand | angles={finger_angles.tolist()}"
                )
                self.state = "resetting"
                self.reset_start_time = now
                state_changed = True
                robot.open_hand()
            elif self.check_contact_trigger(tactile_matrix):
                print("[GraspManager] State: OPEN -> CLOSING | contact triggered")
                self.state = "closing"
                self.closing_start_time = now
                state_changed = True
                # Close fingers 0-3 while keeping thumb fully open
                robot.set_angles([0, 0, 0, 0, 1000, 1000])

        # State: CLOSING
        elif self.state == "closing":
            closing_elapsed = now - self.closing_start_time

            has_moved = np.any(finger_angles < 900)
            motion_halted = np.all(diff < 3) and (closing_elapsed >= self.min_closing_time) and has_moved

            if motion_halted:
                self.state = "settling"
                self.settle_start_time = now
                self.last_log_time = now
                state_changed = True
                print(
                    f"[GraspManager] State: CLOSING -> SETTLING | motion stabilized | angles={finger_angles.tolist()}"
                )

        # State: SETTLING
        elif self.state == "settling":
            settling_elapsed = now - self.settle_start_time

            # Abort early if fingers closed into an empty fist
            if is_empty_grasp(finger_angles, pad_pressures):
                print(
                    f"[GraspManager] State: SETTLING -> RESETTING | empty grasp | angles={finger_angles.tolist()}"
                )
                self.state = "resetting"
                self.reset_start_time = now
                state_changed = True
                robot.open_hand()
            else:
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

                if is_valid:
                    if (now - self.last_log_time) >= 0.25:
                        print(
                            f"[GraspManager] Settling | progress={settling_elapsed:.2f}/{self.settle_duration:.2f}s | "
                            f"spread={diagnostics.get('spread')} | "
                            f"fingers={diagnostics.get('active_fingers')} | "
                            f"sensors={diagnostics.get('active_sensors')}"
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
                            f"[GraspManager] Settling | waiting={reason} | "
                            f"fingers={diagnostics.get('active_fingers')} | "
                            f"sensors={diagnostics.get('active_sensors')}"
                        )
                        self.last_log_time = now

                    if settling_elapsed >= self.settle_timeout:
                        print(
                            f"[GraspManager] State: SETTLING -> RESETTING | timeout={self.settle_timeout:.1f}s | reason={reason}"
                        )
                        self.state = "resetting"
                        self.reset_start_time = now
                        state_changed = True
                        robot.open_hand()

        # State: CYLINDER_CONFIRMED
        elif self.state == "cylinder_confirmed":
            primary_pressures = {f: pad_pressures.get(f, 0.0) for f in ("Little", "Ring", "Middle", "Index")}
            active_contact_fingers = sum(
                1 for p in primary_pressures.values() if p >= self.pad_force_threshold
            )
            total_finger_force = sum(primary_pressures.values())

            if total_finger_force > self.confirmed_total_force:
                self.confirmed_total_force = total_finger_force
                for f, p in primary_pressures.items():
                    self.confirmed_forces[f] = max(self.confirmed_forces.get(f, 0.0), p)

            # Require each finger to retain at least 35% of peak hold force (floor at 50g)
            retained_fingers = sum(
                1 for f, p in primary_pressures.items()
                if p >= max(50.0, 0.35 * self.confirmed_forces.get(f, 0.0))
            )

            # Drop grasp if total force drops below 45% of peak or fewer than 2 fingers contact
            rel_force_threshold = max(100.0, 0.45 * self.confirmed_total_force)
            force_dropped = bool(
                (total_finger_force < rel_force_threshold) or (retained_fingers < 2)
            )

            if (now - self.last_log_time) >= 0.5:
                print(
                    f"[GraspManager] Hold | "
                    f"force={total_finger_force:.0f}g (peak={self.confirmed_total_force:.0f}g) | "
                    f"fingers={active_contact_fingers}/4 (retained={retained_fingers}/4)"
                )
                self.last_log_time = now

            hand_opened = np.any(finger_angles > 350) or np.mean(finger_angles) > 250
            is_empty_fist = is_empty_grasp(finger_angles, pad_pressures)
            palm_val = pad_pressures.get("Palm", 0.0)
            is_palm_curled = bool(
                palm_val > 250.0
                and np.mean(finger_angles) < 30
                and (active_contact_fingers < 2 or total_finger_force < 200.0)
            )

            if hand_opened:
                print(f"[GraspManager] State: CYLINDER_CONFIRMED -> RESETTING | hand opened | angles={finger_angles.tolist()}")
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                self.slip_detector.reset()
                self.was_slipping = False
                state_changed = True
                robot.open_hand()
            elif is_empty_fist:
                print(f"[GraspManager] State: CYLINDER_CONFIRMED -> RESETTING | empty fist | angles={finger_angles.tolist()}")
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                self.slip_detector.reset()
                self.was_slipping = False
                state_changed = True
                robot.open_hand()
            elif is_palm_curled:
                print(f"[GraspManager] State: CYLINDER_CONFIRMED -> RESETTING | palm curled (palm={palm_val:.0f}g)")
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                self.slip_detector.reset()
                self.was_slipping = False
                state_changed = True
                robot.open_hand()
            elif force_dropped:
                if self.contact_loss_start_time == 0.0:
                    self.contact_loss_start_time = now
                elif (now - self.contact_loss_start_time) >= 0.15:
                    print(
                        f"[GraspManager] State: CYLINDER_CONFIRMED -> RESETTING | force drop "
                        f"({total_finger_force:.0f}g < {rel_force_threshold:.0f}g, retained={retained_fingers}/4)"
                    )
                    self.state = "resetting"
                    self.reset_start_time = now
                    self.contact_loss_start_time = 0.0
                    self.slip_detector.reset()
                    self.was_slipping = False
                    state_changed = True
                    robot.open_hand()
            else:
                self.contact_loss_start_time = 0.0

            # Slip detection during cylinder hold
            if self.state == "cylinder_confirmed":
                data["peak_force"] = self.confirmed_total_force
                data["total_finger_force"] = total_finger_force
                is_slipping, slipping_contacts, slip_diag = self.slip_detector.update(
                    tactile_matrix, data
                )
                if is_slipping and not self.was_slipping:
                    slip_details = {
                        k: v.get("slip_reason", "slip active")
                        for k, v in slip_diag.get("sensors", {}).items()
                        if v.get("slip_reason")
                    }
                    print(f"[GraspManager] Slip detected | contacts={slipping_contacts}")
                    if self.slip_callback is not None:
                        self.slip_callback(slipping_contacts, slip_diag)
                elif not is_slipping and self.was_slipping:
                    print("[GraspManager] Slip cleared | grasp stable")

                self.was_slipping = is_slipping

                if not is_slipping and not self.has_probed:
                    if self.no_slip_start_time == 0.0:
                        self.no_slip_start_time = now
                    elif (now - self.no_slip_start_time) >= self.no_slip_stability_duration:
                        print("[GraspManager] State: CYLINDER_CONFIRMED -> THUMB_PROBING | grip stable")
                        self.state = "thumb_probing"
                        self.has_probed = True
                        self.probe_state = "probing"
                        self.probe_summary = {}
                        self.contact_loss_start_time = 0.0
                        state_changed = True
                        self.thumb_prober.start(robot)
                else:
                    self.no_slip_start_time = 0.0

        # State: THUMB_PROBING
        elif self.state == "thumb_probing":
            primary_pressures = {f: pad_pressures.get(f, 0.0) for f in ("Little", "Ring", "Middle", "Index")}
            total_finger_force = sum(primary_pressures.values())

            hand_opened = np.any(finger_angles > 350) or np.mean(finger_angles) > 250
            is_empty_fist = is_empty_grasp(finger_angles, pad_pressures)

            if hand_opened or is_empty_fist:
                reason = "hand opened" if hand_opened else "empty fist"
                print(f"[GraspManager] State: THUMB_PROBING -> RESETTING | grasp lost ({reason})")
                self.thumb_prober.stop()
                self.state = "resetting"
                self.reset_start_time = now
                self.contact_loss_start_time = 0.0
                self.probe_state = "none"
                self.probe_summary = {}
                self.slip_detector.reset()
                self.was_slipping = False
                state_changed = True
                robot.open_hand()
            elif self.thumb_prober.is_finished:
                self.probing_results = list(self.thumb_prober.results)
                self.probe_state = self.thumb_prober.classification
                self.probe_summary = dict(self.thumb_prober.summary)
                self.state = "cylinder_confirmed"
                self.no_slip_start_time = now
                self.contact_loss_start_time = 0.0
                if total_finger_force >= 100.0:
                    self.confirmed_forces = dict(primary_pressures)
                    self.confirmed_total_force = total_finger_force
                self.slip_detector.reset()
                self.was_slipping = False
                state_changed = True
                print(
                    f"[GraspManager] State: THUMB_PROBING -> CYLINDER_CONFIRMED | "
                    f"sensor={self.probe_state} | points={len(self.probing_results)}"
                )

        # State: RESETTING
        elif self.state == "resetting":
            hand_opened = np.all(current_angles[:4] > 880)
            reset_timed_out = (now - self.reset_start_time) > 2.0

            if hand_opened or reset_timed_out:
                self.reset_end_time = now
                contact_still_on = np.count_nonzero(tactile_matrix > self.contact_trigger_threshold) >= 10
                self.contact_cleared = not contact_still_on
                self.contact_triggered = False
                self.state = "open"
                self.confirmed_total_force = 0.0
                self.confirmed_forces = {}
                self.contact_loss_start_time = 0.0
                self.no_slip_start_time = 0.0
                self.has_probed = False
                self.probing_results.clear()
                self.probe_state = "none"
                self.probe_summary = {}
                self.slip_detector.reset()
                self.was_slipping = False
                state_changed = True
                print(f"[GraspManager] State: RESETTING -> OPEN | hand reopened")

        self.last_angles = current_angles
        data["state"] = self.state
        data["state_change"] = (initial_state != self.state) or state_changed
        data["slip_detected"] = (
            self.slip_detector.is_slipping if self.state == "cylinder_confirmed" else False
        )
        data["slipping_contacts"] = (
            self.slip_detector.slipping_contacts if self.state == "cylinder_confirmed" else []
        )
        data["probing_results"] = (
            self.thumb_prober.results if self.state == "thumb_probing" else self.probing_results
        )
        data["probe_state"] = (
            "probing" if self.state == "thumb_probing" else self.probe_state
        )
        data["probe_summary"] = self.probe_summary
        return data

    def on_probing_finished(self) -> None:
        """Handles completion callback from the thumb probing routine."""
        self.probing_results = list(self.thumb_prober.results)
        self.probe_state = self.thumb_prober.classification
        self.probe_summary = dict(self.thumb_prober.summary)

    def restart_probing(self, robot: Any) -> None:
        """Manually triggers or restarts the autonomous thumb probing routine.

        Args:
            robot: Robot hand controller interface.
        """
        print("[GraspManager] State: -> THUMB_PROBING | manual trigger")
        self.state = "thumb_probing"
        self.has_probed = True
        self.probe_state = "probing"
        self.probe_summary = {}
        self.contact_loss_start_time = 0.0
        self.no_slip_start_time = 0.0
        self.thumb_prober.start(robot)
