from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Literal, Optional, Protocol

import numpy as np
from numpy.typing import NDArray

# Import STATES from config.config_parameters with a standalone fallback
from config.config_parameters import STATES

logger = logging.getLogger(__name__)

# ============================================================================
# CONSTANTS & THRESHOLDS
# ============================================================================
# TODO: Verify and tune physical sensor taper geometry constants
SENSOR_LENGTH_M: float = 0.100        # 100 mm total sensor length (-50 mm to +50 mm)
SENSOR_HALF_LENGTH_M: float = 0.050   # +50 mm at sensor head (+x_sensor), -50 mm at tail (-x_sensor)
SENSOR_MAX_DIAMETER_M: float = 0.030  # 30 mm diameter at x_sensor = +50 mm (+0.050 m)
SENSOR_MIN_DIAMETER_M: float = 0.020  # 20 mm diameter at x_sensor = -50 mm (-0.050 m)
SENSOR_ORIGIN_Z_REF_M: float = 0.0125 # Sensor centerline z-height at origin (12.5 mm mid-radius)
CLAMP_LOCAL_X_TO_SENSOR: bool = False # Set True to clamp extrapolated x_local to [-0.05, +0.05]

# TODO: Verify and tune 2D Circle & 360-degree Roll sweep constants
CIRCLE_RADIUS_M: float = 0.010            # 10 mm circle radius
CIRCLE_CENTER_X_OFFSET_M: float = 0.070   # 70 mm to the right (+x_sensor) of sensor origin
CIRCLE_CENTER_Y_OFFSET_M: float = -0.015  # Arbitrary lateral offset ("some distance to the right")
CIRCLE_CENTER_Z_OFFSET_M: float = 0.000   # Arbitrary local z offset of circle center
CIRCLE_YAW_OFFSET_DEG: float = 30.0       # +30 degrees yaw relative to sensor origin
CIRCLE_NUM_POINTS: int = 100              # 100 equally distributed points on the circle
ROLL_NUM_ORIENTATIONS: int = 360          # 360 discrete roll orientations (0..359 deg about x_sensor)

# TODO: Verify and tune Thumb Arc Plane fitting & taxel deletion constants
THUMB_SURFACE_TAXEL_KEYS: tuple[str, ...] = (
    "thumb_pad",
    "thumb_mid",
    "thumb_tip",
)
THUMB_ARC_LONGITUDINAL_SAMPLES: int = 35  # Uniform samples from start to end of thumb arc
THUMB_ARC_LATERAL_SAMPLES: int = 16       # Uniform samples across thumb width plane
THUMB_TAXEL_DELETE_RADIUS_M: float = 0.0025  # Radius (2.5 mm) around real taxels to delete from arc plane
THUMB_ARC_MAX_RADIUS_M: float = 2.0       # Max radius cap before treating arc as near-flat polynomial

# Nail Yaw Constants
NAIL_SENSOR_NAMES: tuple[str, str, str, str] = (
    "Little Nail",
    "Ring Nail",
    "Middle Nail",
    "Index Nail",
)

# Maps robot.matrices sensor names -> (BaseTaxel.key, BaseTaxel.link) from regions()
NAIL_TAXEL_REGION_MAP: dict[str, tuple[str, str]] = {
    "Little Nail": ("little_nail", "left_little_force_sensor_2"),
    "Ring Nail": ("ring_nail", "left_ring_force_sensor_2"),
    "Middle Nail": ("middle_nail", "left_middle_force_sensor_2"),
    "Index Nail": ("index_nail", "left_index_force_sensor_2"),
}

SINGLE_NAIL_SHAPE: tuple[int, int] = (12, 8)
COMBINED_NAIL_SHAPE: tuple[int, int] = (12, 32)
NAIL_FORCE_THRESHOLD: float = 15.0        # Taxels below this are zeroed out
MIN_YAW_ACTIVE_COLS: int = 3              # Minimum active columns needed to fit a line
MIN_YAW_COLUMN_SPAN: int = 4              # Minimum horizontal spread (cols) to avoid single-finger noise
YAW_FLAT_SLOPE_EPSILON: float = 1e-2      # Abs slope below this is considered flat (0 yaw)

# Palm Taper Constants
PALM_SENSOR_NAME: str = "Palm"
PALM_SHAPE: tuple[int, int] = (8, 14)
PALM_FORCE_NOISE_FLOOR: float = 5.0       # Minimum taxel reading to consider valid contact
PALM_MIN_ACTIVE_TAXELS: int = 4           # Minimum active taxels required to judge taper
PALM_SLOPE_EPSILON: float = 1e-2          # Abs slope below this is considered flat / untapered

# Thumb Probing Constants
THUMB_INWARD_IDX: int = 4                 # 5th element: inward rotation angle
THUMB_POSITION_IDX: int = 5               # 6th element: lateral sweep/station position
REQUIRED_ANGLES_LEN: int = 6

THUMB_STABILITY_THRESHOLD: float = 2.0    # Max delta on BOTH angles[4] and angles[5] between ticks
THUMB_SAME_STATION_THRESHOLD: float = 2.0 # While |angles[5] - prev_probe_angle_5| <= 2.0, same probe
THUMB_SHALLOW_ANGLE_THRESHOLD: float = 45.0  # Steady angles[4] > this -> shallow (Sensor 1); else deep (Sensor 2)
THUMB_PROB_TEMPERATURE: float = 5.0       # Softness parameter for logistic confidence scoring
NUM_PROBE_STEPS: int = 2                  # Total number of thumb probes to collect (2 probes for roll)
ROLL_REQUIRED_PROBES: int = 2             # Number of probes required to trigger roll estimation
SAFETY_SKIP_TICKS: int = 1                # Number of initial ticks to skip completely in thumb_probing


# TODO : Maybe import data classes from here
from digital_twin.extract_tectile import reframe_to_palm_taxel, TactileHandKinematics, BaseTaxel
from interface.robot_control import RobotHandController


# ============================================================================
# OUTPUT / STATE DATA STRUCTURES
# ============================================================================
@dataclass(frozen=True)
class TaperEstimation:
    valid: bool
    direction: Optional[Literal["+x", "-x", "flat"]]
    slope_along_x: float
    left_half_mean: float
    right_half_mean: float
    active_taxel_count: int


@dataclass(frozen=True)
class YawEstimation:
    valid: bool
    tilt_direction: Optional[Literal["up", "down", "flat"]]
    cartesian_slope: Optional[float]   # dy/dx where +x is left-to-right (0..31), +y is bottom-to-top
    matrix_row_slope: Optional[float]  # d(row)/d(col) where row 0 is top, row 11 is bottom
    yaw_rad: Optional[float]
    yaw_deg: Optional[float]
    active_columns: int
    column_span: int
    thresholded_matrix: NDArray[np.float64]


@dataclass(frozen=True)
class NailMaxContactPoint:
    sensor_name: str
    taxel_key: str
    taxel_link: str
    row: int
    col: int
    max_force: float
    position_ref: NDArray[np.float64]  # (3,) [x, y, z] in palm reference frame (meters)
    normal_ref: NDArray[np.float64]    # (3,) [nx, ny, nz] in palm reference frame


@dataclass(frozen=True)
class SensorOriginEstimation:
    valid: bool
    origin_xy_ref: Optional[NDArray[np.float64]]          # (2,) [X_origin, Y_origin] in palm frame (meters)
    origin_xyz_ref: Optional[NDArray[np.float64]]         # (3,) [X_origin, Y_origin, Z_origin] in palm frame (meters)
    sensor_heading_rad: Optional[float]                   # Signed heading of +x_sensor in palm xy-plane
    sensor_heading_deg: Optional[float]
    sensor_x_axis_unit_xy: Optional[NDArray[np.float64]]  # (2,) [cos(heading), sin(heading)]
    sensor_y_axis_unit_xy: Optional[NDArray[np.float64]]  # (2,) [-sin(heading), cos(heading)]
    sensor_x_axis_3d: Optional[NDArray[np.float64]]       # (3,) unit vector of +x_sensor in palm frame
    sensor_y_axis_3d: Optional[NDArray[np.float64]]       # (3,) unit vector of +y_sensor in palm frame
    sensor_z_axis_3d: Optional[NDArray[np.float64]]       # (3,) unit vector of +z_sensor in palm frame
    index_contact_ref: Optional[NDArray[np.float64]]      # (3,) [x, y, z] of max-force Index Nail taxel
    index_local_x_m: Optional[float]                      # Extrapolated x_sensor offset (meters) at Index Nail
    nail_max_points: dict[str, NailMaxContactPoint] = field(default_factory=dict)


@dataclass(frozen=True)
class ThumbProbeStep:
    probe_number: int          # 1 to NUM_PROBE_STEPS
    tick_recorded: int         # Tick index within thumb_probing when settled
    x_position_angle: float    # angles[5]: thumb position along sweep axis
    inward_angle: float        # angles[4]: thumb inward penetration angle
    is_shallow: bool           # True if inward_angle > THUMB_SHALLOW_ANGLE_THRESHOLD
    sensor_1_Step_prob: float  # Logistic confidence for Sensor 1 on this single step
    thumb_comparison_points_ref: Optional[NDArray[np.float64]] = None  # (M, 3) fitted arc plane minus taxels
    fitted_arc_radius_m: Optional[float] = None


@dataclass(frozen=True)
class RollEstimation:
    valid: bool
    best_roll_deg: Optional[float]                        # Winning orientation (0..359 deg) about x_sensor
    best_roll_rad: Optional[float]
    min_mean_distance_m: Optional[float]                  # Lowest mean distance across Probe 1 and Probe 2
    probe_1_min_distance_m: Optional[float]               # Min distance for Probe 1 at winning orientation
    probe_2_min_distance_m: Optional[float]               # Min distance for Probe 2 at winning orientation
    probe_1_closest_circle_pt_ref: Optional[NDArray[np.float64]] = None  # (3,) global point on circle for Probe 1
    probe_2_closest_circle_pt_ref: Optional[NDArray[np.float64]] = None  # (3,) global point on circle for Probe 2
    orientation_mean_distances_m: Optional[NDArray[np.float64]] = None   # (360,) mean distance per roll angle
    best_circle_points_ref: Optional[NDArray[np.float64]] = None         # (100, 3) global circle points at best roll


@dataclass
class SensorDecision:
    is_complete: bool = False
    predicted_sensor: Optional[Literal["sensor_1", "sensor_2"]] = None
    sensor_1_probability: float = 0.5
    sensor_2_probability: float = 0.5
    shallow_probe_count: int = 0
    deep_probe_count: int = 0
    mean_inward_angle: Optional[float] = None


@dataclass
class EstimatorOutput:
    current_state: Optional[STATES] = None
    taper: Optional[TaperEstimation] = None
    yaw: Optional[YawEstimation] = None
    origin: Optional[SensorOriginEstimation] = None
    roll: Optional[RollEstimation] = None
    probes: list[ThumbProbeStep] = field(default_factory=list)
    decision: SensorDecision = field(default_factory=SensorDecision)


# ============================================================================
# MAIN LOGIC CONTROLLER
# ============================================================================
class TactilePoseAndSensorEstimator:
    """
    State-driven controller that estimates 6-DOF cylinder/sensor alignment cues
    (with z and pitch locked -> 4 active DOFs: x, y, yaw, and roll) and classifies
    Sensor 1 vs. Sensor 2 via thumb probing.
    """

    # States permitted to execute on consecutive ticks without a state transition
    CONTINUOUS_POLLING_STATES: frozenset[str] = frozenset({"thumb_probing"})

    def __init__(self, robot: RobotHandController) -> None:
        self.robot = robot
        self.previous_state: Optional[STATES] = None
        self.output = EstimatorOutput()

        # Internal trackers for thumb_probing
        self._probing_tick: int = 0
        self._prev_thumb_angles: Optional[NDArray[np.float64]] = None  # shape (2,): [angles[4], angles[5]]
        self._latched_station_angle: Optional[float] = None            # angles[5] of last recorded probe
        self._station_is_latched: bool = False

    # ------------------------------------------------------------------------
    # PUBLIC ENTRYPOINT
    # ------------------------------------------------------------------------
    def main(
        self,
        kinematics: Optional[TactileHandKinematics] | None = None,
    ) -> EstimatorOutput:
        """
        Main loop callback invoked on every iteration from the external runner.

        Enforces the single-execution rule: a state's logic is never executed
        on two consecutive calls EXCEPT for states in CONTINUOUS_POLLING_STATES
        ('thumb_probing').
        """
        resolved_state = self.robot.state
        self.output.current_state = resolved_state

        is_state_transition = resolved_state != self.previous_state

        # Enforce single-execution rule for non-polling states
        if not is_state_transition and resolved_state not in self.CONTINUOUS_POLLING_STATES:
            return self.output

        if is_state_transition:
            self._on_state_transition(self.previous_state, resolved_state)

        # Dispatch state handlers
        if resolved_state == "starting":
            self._execute_starting()
        elif resolved_state == "initialised":
            self._execute_initialised()
        elif resolved_state == "open":
            self._execute_open()
        elif resolved_state == "closing":
            self._execute_closing()
        elif resolved_state == "settling":
            self._execute_settling()
        elif resolved_state == "steady_state":
            self._execute_steady_state()
        elif resolved_state == "cylinder_confirmed":
            self._execute_cylinder_confirmed(kinematics)
        elif resolved_state == "thumb_probing":
            self._execute_thumb_probing(kinematics)
        elif resolved_state == "probing_complete":
            self._execute_probing_complete()
        elif resolved_state == "resetting":
            self._execute_resetting()
        else:
            raise ValueError(f"Unhandled state received: {resolved_state!r}")

        self.previous_state = resolved_state
        return self.output

    # Alias for callers using .step() convention
    step = main

    # ------------------------------------------------------------------------
    # STATE LIFECYCLE & HANDLERS
    # ------------------------------------------------------------------------
    def _on_state_transition(
        self,
        old_state: Optional[STATES],
        new_state: STATES,
    ) -> None:
        logger.debug("State transition: %s -> %s", old_state, new_state)
        if new_state == "thumb_probing":
            self._reset_probing_trackers(clear_recorded_probes=True)

    def _execute_starting(self) -> None:
        self._full_reset()

    def _execute_initialised(self) -> None:
        pass

    def _execute_open(self) -> None:
        pass

    def _execute_closing(self) -> None:
        pass

    def _execute_settling(self) -> None:
        pass

    def _execute_steady_state(self) -> None:
        pass

    def _execute_cylinder_confirmed(
        self,
        kinematics: Optional[TactileHandKinematics],
    ) -> None:
        """
        Executes once upon entering 'cylinder_confirmed':
        1. Evaluates the 8x14 Palm tactile matrix to determine force taper (+x / -x).
        2. Evaluates the concatenated 12x32 Nail matrix to determine yaw tilt.
        3. Extracts the max-force BaseTaxel on each of the 4 finger nails and uses
           the Index Nail z-height + taper geometry to resolve the sensor's local
           coordinate origin and 3D axes in the palm reference frame.
        """
        self.output.taper = self._calculate_palm_taper()
        self.output.yaw = self._calculate_nail_yaw()
        self.output.origin = self._calculate_sensor_origin(
            kinematics=kinematics,
            taper=self.output.taper,
            yaw=self.output.yaw,
        )

    def _execute_thumb_probing(
        self,
        kinematics: Optional[TactileHandKinematics],
    ) -> None:
        """
        Executes on every tick while in 'thumb_probing' until NUM_PROBE_STEPS (2)
        probes are collected:
        - Tick 0: Skips completely for safety.
        - Tick 1: Captures initial baseline thumb angles [angles[4], angles[5]].
        - Tick 2+: Checks if both thumb angles settled within THUMB_STABILITY_THRESHOLD (2.0).
          When settled at a new station:
            1. Fits the 2D plane + circular arc to the current thumb taxels and
               deletes the actual taxel coordinates to obtain the thumb comparison points.
            2. Updates Sensor 1 vs Sensor 2 probabilities.
            3. Once 2 probes are recorded, executes the 360-degree Roll optimization.
        """
        if len(self.output.probes) >= NUM_PROBE_STEPS:
            return

        current_tick = self._probing_tick
        self._probing_tick += 1

        # 1. Safety skip on the first iteration (Tick 0)
        if current_tick < SAFETY_SKIP_TICKS:
            return

        current_thumb = self._read_thumb_angles()  # shape (2,): [inward_angle_4, pos_angle_5]
        inward_angle = float(current_thumb[0])
        pos_angle = float(current_thumb[1])

        # 2. First active tick after safety skip: initialize baseline and exit
        if self._prev_thumb_angles is None:
            self._prev_thumb_angles = current_thumb
            return

        # 3. Check if we have moved away from the previously latched probe station
        if self._station_is_latched and self._latched_station_angle is not None:
            if abs(pos_angle - self._latched_station_angle) > THUMB_SAME_STATION_THRESHOLD:
                # Thumb has moved out of the previous station window; unlatch for next probe
                self._station_is_latched = False

        # 4. Compare current thumb angles against previous tick for stability (both <= 2.0)
        deltas = np.abs(current_thumb - self._prev_thumb_angles)
        is_steady = bool(np.all(deltas <= THUMB_STABILITY_THRESHOLD))

        # Always roll _prev_thumb_angles forward for the next tick
        self._prev_thumb_angles = current_thumb

        if not is_steady:
            return

        # 5. Suppress duplicate recording if still at the same probe station (angles[5] similar)
        if self._station_is_latched:
            return

        if (
            self._latched_station_angle is not None
            and abs(pos_angle - self._latched_station_angle) <= THUMB_SAME_STATION_THRESHOLD
        ):
            return

        # 6. Build the fitted thumb circular-arc plane and delete taxel coordinates
        thumb_comp_pts, fitted_radius = self._build_thumb_arc_comparison_points(kinematics)

        # 7. Record validated steady probe step
        probe_num = len(self.output.probes) + 1
        is_shallow = inward_angle > THUMB_SHALLOW_ANGLE_THRESHOLD

        z = (inward_angle - THUMB_SHALLOW_ANGLE_THRESHOLD) / THUMB_PROB_TEMPERATURE
        step_s1_prob = float(1.0 / (1.0 + np.exp(-np.clip(z, -50.0, 50.0))))

        probe_step = ThumbProbeStep(
            probe_number=probe_num,
            tick_recorded=current_tick,
            x_position_angle=pos_angle,
            inward_angle=inward_angle,
            is_shallow=is_shallow,
            sensor_1_Step_prob=step_s1_prob,
            thumb_comparison_points_ref=thumb_comp_pts,
            fitted_arc_radius_m=fitted_radius,
        )
        self.output.probes.append(probe_step)

        # Latch current station so we don't record again until angles[5] moves
        self._latched_station_angle = pos_angle
        self._station_is_latched = True

        # 8. Update sensor classification
        self._update_sensor_decision()

        # 9. Once 2 probes are done, compute the 360-degree Roll estimation
        if len(self.output.probes) >= ROLL_REQUIRED_PROBES:
            self.output.roll = self._calculate_sensor_roll()

    def _execute_probing_complete(self) -> None:
        """Finalizes sensor classification and roll estimation if transitioning out of probing."""
        self._update_sensor_decision(force_complete=True)
        if self.output.roll is None and len(self.output.probes) >= ROLL_REQUIRED_PROBES:
            self.output.roll = self._calculate_sensor_roll()

    def _execute_resetting(self) -> None:
        self._full_reset()

    # ------------------------------------------------------------------------
    # CORE COMPUTATION: PALM TAPER, NAIL YAW & SENSOR ORIGIN (X, Y, Z)
    # ------------------------------------------------------------------------
    def _calculate_palm_taper(self) -> TaperEstimation:
        """
        Determines the taper direction along the hand's x-axis from the 8x14 Palm matrix.
        Columns 0 -> 13 correspond to +x (Little finger -> Index/Thumb).
        If force decreases left-to-right (negative slope), taper is '-x'.
        If force increases left-to-right (positive slope), taper is '+x'.
        """
        palm = self._get_validated_matrix(PALM_SENSOR_NAME, PALM_SHAPE)

        clean_palm = np.where(np.isfinite(palm), palm, 0.0).astype(np.float64)
        active_palm = np.where(clean_palm >= PALM_FORCE_NOISE_FLOOR, clean_palm, 0.0)

        active_count = int(np.count_nonzero(active_palm))
        half_col = PALM_SHAPE[1] // 2
        left_mean = float(np.mean(active_palm[:, :half_col]))
        right_mean = float(np.mean(active_palm[:, half_col:]))

        if active_count < PALM_MIN_ACTIVE_TAXELS:
            return TaperEstimation(
                valid=False,
                direction=None,
                slope_along_x=0.0,
                left_half_mean=left_mean,
                right_half_mean=right_mean,
                active_taxel_count=active_count,
            )

        col_forces = np.mean(active_palm, axis=0)  # shape (14,)
        col_indices = np.arange(PALM_SHAPE[1], dtype=np.float64)

        slope = float(np.polyfit(col_indices, col_forces, 1)[0])

        if abs(slope) <= PALM_SLOPE_EPSILON:
            direction: Literal["+x", "-x", "flat"] = "flat"
        elif slope < 0.0:
            direction = "-x"
        else:
            direction = "+x"

        return TaperEstimation(
            valid=True,
            direction=direction,
            slope_along_x=slope,
            left_half_mean=left_mean,
            right_half_mean=right_mean,
            active_taxel_count=active_count,
        )

    def _calculate_nail_yaw(self) -> YawEstimation:
        """
        Concatenates the 4 finger nail matrices (Little, Ring, Middle, Index)
        into a single 12x32 matrix, zeros values below NAIL_FORCE_THRESHOLD,
        and fits a force-weighted line from left to right to estimate yaw tilt.
        """
        nail_blocks = [
            self._get_validated_matrix(name, SINGLE_NAIL_SHAPE)
            for name in NAIL_SENSOR_NAMES
        ]
        combined = np.hstack(nail_blocks).astype(np.float64)  # shape (12, 32)
        if combined.shape != COMBINED_NAIL_SHAPE:
            raise ValueError(
                f"Combined nail matrix has shape {combined.shape}, expected {COMBINED_NAIL_SHAPE}."
            )

        clean_combined = np.where(np.isfinite(combined), combined, 0.0)
        thresholded = np.where(clean_combined >= NAIL_FORCE_THRESHOLD, clean_combined, 0.0)

        col_weights = np.sum(thresholded, axis=0)  # shape (32,)
        active_col_mask = col_weights > 0.0
        active_cols = np.nonzero(active_col_mask)[0]
        num_active_cols = int(active_cols.size)

        if num_active_cols < MIN_YAW_ACTIVE_COLS:
            return YawEstimation(
                valid=False,
                tilt_direction=None,
                cartesian_slope=None,
                matrix_row_slope=None,
                yaw_rad=None,
                yaw_deg=None,
                active_columns=num_active_cols,
                column_span=0,
                thresholded_matrix=thresholded,
            )

        col_span = int(active_cols[-1] - active_cols[0])
        if col_span < MIN_YAW_COLUMN_SPAN:
            return YawEstimation(
                valid=False,
                tilt_direction=None,
                cartesian_slope=None,
                matrix_row_slope=None,
                yaw_rad=None,
                yaw_deg=None,
                active_columns=num_active_cols,
                column_span=col_span,
                thresholded_matrix=thresholded,
            )

        row_indices = np.arange(COMBINED_NAIL_SHAPE[0], dtype=np.float64)[:, np.newaxis]
        weighted_row_sums = np.sum(thresholded[:, active_cols] * row_indices, axis=0)
        row_centroids = weighted_row_sums / col_weights[active_cols]

        x_cols = active_cols.astype(np.float64)
        w = col_weights[active_cols]
        matrix_row_slope = float(np.polyfit(x_cols, row_centroids, 1, w=w)[0])

        cartesian_slope = -matrix_row_slope
        yaw_rad = float(np.arctan(cartesian_slope))
        yaw_deg = float(np.degrees(yaw_rad))

        if abs(cartesian_slope) <= YAW_FLAT_SLOPE_EPSILON:
            tilt_dir: Literal["up", "down", "flat"] = "flat"
        elif cartesian_slope > 0.0:
            tilt_dir = "up"
        else:
            tilt_dir = "down"

        return YawEstimation(
            valid=True,
            tilt_direction=tilt_dir,
            cartesian_slope=cartesian_slope,
            matrix_row_slope=matrix_row_slope,
            yaw_rad=yaw_rad,
            yaw_deg=yaw_deg,
            active_columns=num_active_cols,
            column_span=col_span,
            thresholded_matrix=thresholded,
        )

    def _calculate_sensor_origin(
        self,
        kinematics: Optional[TactileHandKinematics],
        taper: Optional[TaperEstimation],
        yaw: Optional[YawEstimation],
    ) -> SensorOriginEstimation:
        """
        Resolves the sensor's local coordinate system 3D origin (X_origin, Y_origin, Z_origin)
        and orthonormal 3D axes in the palm reference frame using the Index Nail max-force taxel
        and sensor taper geometry.
        """
        if kinematics is None or not kinematics.base_taxels:
            logger.warning("Kinematics with base_taxels is required to calculate sensor origin.")
            return SensorOriginEstimation(
                valid=False,
                origin_xy_ref=None,
                origin_xyz_ref=None,
                sensor_heading_rad=None,
                sensor_heading_deg=None,
                sensor_x_axis_unit_xy=None,
                sensor_y_axis_unit_xy=None,
                sensor_x_axis_3d=None,
                sensor_y_axis_3d=None,
                sensor_z_axis_3d=None,
                index_contact_ref=None,
                index_local_x_m=None,
                nail_max_points={},
            )

        taxel_by_key: dict[tuple[str, int, int], BaseTaxel] = {}
        taxel_by_link: dict[tuple[str, int, int], BaseTaxel] = {}
        for bt in kinematics.base_taxels:
            taxel_by_key[(bt.key, int(bt.row), int(bt.col))] = bt
            taxel_by_link[(bt.link, int(bt.row), int(bt.col))] = bt

        nail_max_points: dict[str, NailMaxContactPoint] = {}
        for sensor_name in NAIL_SENSOR_NAMES:
            contact_pt = self._extract_max_nail_taxel(
                sensor_name=sensor_name,
                taxel_by_key=taxel_by_key,
                taxel_by_link=taxel_by_link,
            )
            if contact_pt is not None:
                nail_max_points[sensor_name] = contact_pt

        index_contact = nail_max_points.get("Index Nail")
        if index_contact is None:
            logger.warning("Index Nail has no valid active taxel or matching BaseTaxel.")
            return SensorOriginEstimation(
                valid=False,
                origin_xy_ref=None,
                origin_xyz_ref=None,
                sensor_heading_rad=None,
                sensor_heading_deg=None,
                sensor_x_axis_unit_xy=None,
                sensor_y_axis_unit_xy=None,
                sensor_x_axis_3d=None,
                sensor_y_axis_3d=None,
                sensor_z_axis_3d=None,
                index_contact_ref=None,
                index_local_x_m=None,
                nail_max_points=nail_max_points,
            )

        z_index_ref = float(index_contact.position_ref[2])
        index_local_x_m = self._extrapolate_local_x_from_z(z_index_ref)

        base_yaw_rad = yaw.yaw_rad if (yaw is not None and yaw.valid and yaw.yaw_rad is not None) else 0.0

        if taper is not None and taper.valid and taper.direction == "-x":
            heading_rad = float(
                np.arctan2(np.sin(base_yaw_rad + np.pi), np.cos(base_yaw_rad + np.pi))
            )
        else:
            heading_rad = float(base_yaw_rad)

        heading_deg = float(np.degrees(heading_rad))

        u_x_2d = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=np.float64)
        u_y_2d = np.array([-np.sin(heading_rad), np.cos(heading_rad)], dtype=np.float64)

        u_x_3d = np.array([np.cos(heading_rad), np.sin(heading_rad), 0.0], dtype=np.float64)
        u_y_3d = np.array([-np.sin(heading_rad), np.cos(heading_rad), 0.0], dtype=np.float64)
        u_z_3d = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        index_xy_ref = index_contact.position_ref[:2]
        origin_xy_ref = index_xy_ref - (index_local_x_m * u_x_2d)
        origin_xyz_ref = np.array(
            [origin_xy_ref[0], origin_xy_ref[1], SENSOR_ORIGIN_Z_REF_M],
            dtype=np.float64,
        )

        return SensorOriginEstimation(
            valid=True,
            origin_xy_ref=origin_xy_ref,
            origin_xyz_ref=origin_xyz_ref,
            sensor_heading_rad=heading_rad,
            sensor_heading_deg=heading_deg,
            sensor_x_axis_unit_xy=u_x_2d,
            sensor_y_axis_unit_xy=u_y_2d,
            sensor_x_axis_3d=u_x_3d,
            sensor_y_axis_3d=u_y_3d,
            sensor_z_axis_3d=u_z_3d,
            index_contact_ref=index_contact.position_ref.copy(),
            index_local_x_m=index_local_x_m,
            nail_max_points=nail_max_points,
        )

    @staticmethod
    def _extrapolate_local_x_from_z(z_height_m: float) -> float:
        mid_diameter_m = 0.5 * (SENSOR_MAX_DIAMETER_M + SENSOR_MIN_DIAMETER_M)
        diameter_span_m = SENSOR_MAX_DIAMETER_M - SENSOR_MIN_DIAMETER_M
        if np.isclose(diameter_span_m, 0.0):
            raise ValueError("SENSOR_MAX_DIAMETER_M and SENSOR_MIN_DIAMETER_M cannot be equal.")

        taper_rate = diameter_span_m / SENSOR_LENGTH_M
        x_local_m = float((z_height_m - mid_diameter_m) / taper_rate)

        if CLAMP_LOCAL_X_TO_SENSOR:
            x_local_m = float(np.clip(x_local_m, -SENSOR_HALF_LENGTH_M, SENSOR_HALF_LENGTH_M))

        return x_local_m

    def _extract_max_nail_taxel(
        self,
        sensor_name: str,
        taxel_by_key: dict[tuple[str, int, int], BaseTaxel],
        taxel_by_link: dict[tuple[str, int, int], BaseTaxel],
    ) -> Optional[NailMaxContactPoint]:
        mat = self._get_validated_matrix(sensor_name, SINGLE_NAIL_SHAPE)
        clean_mat = np.where(np.isfinite(mat), mat, -np.inf).astype(np.float64)

        flat_idx = int(np.argmax(clean_mat))
        max_row, max_col = np.unravel_index(flat_idx, SINGLE_NAIL_SHAPE)
        max_force = float(clean_mat[max_row, max_col])

        if not np.isfinite(max_force) or max_force < NAIL_FORCE_THRESHOLD:
            return None

        region_key, region_link = NAIL_TAXEL_REGION_MAP[sensor_name]
        r, c = int(max_row), int(max_col)

        matched_taxel = (
            taxel_by_key.get((region_key, r, c))
            or taxel_by_link.get((region_link, r, c))
            or taxel_by_key.get((sensor_name, r, c))
        )
        if matched_taxel is None:
            logger.warning(
                "Could not find BaseTaxel for %s at (row=%d, col=%d) [key=%r, link=%r].",
                sensor_name, r, c, region_key, region_link,
            )
            return None

        return NailMaxContactPoint(
            sensor_name=sensor_name,
            taxel_key=matched_taxel.key,
            taxel_link=matched_taxel.link,
            row=r,
            col=c,
            max_force=max_force,
            position_ref=np.asarray(matched_taxel.position, dtype=np.float64),
            normal_ref=np.asarray(matched_taxel.normal, dtype=np.float64),
        )

    # ------------------------------------------------------------------------
    # CORE COMPUTATION: THUMB ARC FITTING & 360-DEGREE ROLL ESTIMATION
    # ------------------------------------------------------------------------
    def _build_thumb_arc_comparison_points(
        self,
        kinematics: Optional[TactileHandKinematics],
    ) -> tuple[Optional[NDArray[np.float64]], Optional[float]]:
        """
        Builds the thumb comparison surface for the current probe posture:
        1. Extracts the 3D positions of the thumb taxels from `kinematics.base_taxels`.
        2. Finds the 2D plane and longitudinal limits (beginning to end of thumb taxels)
           via SVD, and fits a circular arc directly to the thumb taxel profile.
        3. Uniformly samples 3D points across that fitted circular arc plane.
        4. Deletes all points on the arc plane that lie within `THUMB_TAXEL_DELETE_RADIUS_M`
           of the actual thumb taxel coordinates, leaving only the un-instrumented
           thumb surface points.
        """
        if kinematics is None or not kinematics.base_taxels:
            logger.warning("Cannot build thumb arc plane: kinematics or base_taxels is missing.")
            return None, None

        # Collect inner thumb surface taxels (pad, mid, tip); fallback to any thumb taxel if empty
        thumb_taxels = [
            bt for bt in kinematics.base_taxels
            if bt.key in THUMB_SURFACE_TAXEL_KEYS
        ]
        if len(thumb_taxels) < 3:
            thumb_taxels = [
                bt for bt in kinematics.base_taxels
                if "thumb" in bt.key.lower() or "thumb" in bt.link.lower()
            ]

        if len(thumb_taxels) < 3:
            logger.warning("Not enough thumb taxels (%d) to fit an arc plane.", len(thumb_taxels))
            return None, None

        taxel_positions = np.array([bt.position for bt in thumb_taxels], dtype=np.float64)  # (N, 3)
        taxel_normals = np.array([bt.normal for bt in thumb_taxels], dtype=np.float64)      # (N, 3)

        # 1. Principal axes of the thumb taxel cloud via SVD
        centroid = np.mean(taxel_positions, axis=0)
        centered = taxel_positions - centroid
        _, _, vt = np.linalg.svd(centered, full_matrices=False)

        e_long = vt[0]   # Longitudinal axis: beginning -> end of thumb taxels
        e_lat = vt[1]    # Lateral width axis across the 2D plane of the thumb
        e_norm = vt[2]   # Out-of-plane curvature/normal axis

        # Orient e_norm consistently with the average taxel normal
        mean_normal = np.mean(taxel_normals, axis=0)
        if np.dot(e_norm, mean_normal) < 0.0:
            e_norm = -e_norm

        # Ensure right-handed local basis (e_long, e_lat, e_norm)
        if np.dot(np.cross(e_long, e_lat), e_norm) < 0.0:
            e_lat = -e_lat

        # Project thumb taxels into local (u_long, v_lat, w_norm) coordinates
        u_coords = centered @ e_long
        v_coords = centered @ e_lat
        w_coords = centered @ e_norm

        # 2. Fit a circular arc to (u_coords, w_coords) from the beginning to the end of the taxels
        u_samples, w_samples, fitted_radius = self._fit_2d_circular_arc(
            u_coords=u_coords,
            w_coords=w_coords,
            num_samples=THUMB_ARC_LONGITUDINAL_SAMPLES,
        )

        # 3. Extrude the fitted circular arc across the lateral width limits [v_min, v_max]
        v_min, v_max = float(np.min(v_coords)), float(np.max(v_coords))
        v_samples = np.linspace(v_min, v_max, THUMB_ARC_LATERAL_SAMPLES, dtype=np.float64)

        # Meshgrid over longitudinal arc samples and lateral width samples
        u_grid, v_grid = np.meshgrid(u_samples, v_samples, indexing="ij")
        w_grid = np.broadcast_to(w_samples[:, np.newaxis], u_grid.shape)

        # Map local grid back to global 3D palm reference coordinates: shape (L * W, 3)
        arc_plane_points = (
            centroid
            + np.outer(u_grid.ravel(), e_long)
            + np.outer(v_grid.ravel(), e_lat)
            + np.outer(w_grid.ravel(), e_norm)
        )

        # 4. Delete the actual taxel coordinates from the generated arc plane
        # Compute distance from every arc-plane point (G, 3) to every thumb taxel (N, 3)
        diffs = arc_plane_points[:, np.newaxis, :] - taxel_positions[np.newaxis, :, :]
        min_dist_to_taxel = np.min(np.linalg.norm(diffs, axis=-1), axis=1)

        keep_mask = min_dist_to_taxel > THUMB_TAXEL_DELETE_RADIUS_M
        comparison_points = arc_plane_points[keep_mask]

        # Safety fallback if the exclusion radius was set too wide and deleted all points
        if comparison_points.size == 0:
            logger.warning(
                "All generated thumb arc points were within THUMB_TAXEL_DELETE_RADIUS_M=%.4f m; "
                "retaining the farthest 25%% of arc points as fallback.",
                THUMB_TAXEL_DELETE_RADIUS_M,
            )
            cutoff = float(np.percentile(min_dist_to_taxel, 75.0))
            comparison_points = arc_plane_points[min_dist_to_taxel >= cutoff]

        return comparison_points, fitted_radius

    @staticmethod
    def _fit_2d_circular_arc(
        u_coords: NDArray[np.float64],
        w_coords: NDArray[np.float64],
        num_samples: int,
    ) -> tuple[NDArray[np.float64], NDArray[np.float64], float]:
        """
        Fits a 2D circular arc `(u - u_c)^2 + (w - w_c)^2 = R^2` directly to the
        thumb taxel coordinates `(u_coords, w_coords)` between the start (`u_min`)
        and end (`u_max`) of the thumb taxels.

        Uses the algebraic Kåsa circle fit:
            A*u + B*w + C = u^2 + w^2,  where (u_c, w_c) = (A/2, B/2), R = sqrt(u_c^2 + w_c^2 + C).
        If the thumb taxels are nearly collinear (radius > THUMB_ARC_MAX_RADIUS_M),
        smoothly falls back to a 2nd-order parabolic arc fit.
        """
        u_min, u_max = float(np.min(u_coords)), float(np.max(u_coords))

        design_mat = np.column_stack([u_coords, w_coords, np.ones_like(u_coords)])
        rhs = u_coords**2 + w_coords**2

        try:
            coeffs, _, rank, _ = np.linalg.lstsq(design_mat, rhs, rcond=None)
            u_c = 0.5 * float(coeffs[0])
            w_c = 0.5 * float(coeffs[1])
            r_sq = u_c**2 + w_c**2 + float(coeffs[2])
            radius = float(np.sqrt(max(r_sq, 0.0)))
        except np.linalg.LinAlgError:
            rank = 0
            u_c, w_c, radius = 0.0, 0.0, np.inf

        span = max(u_max - u_min, 1e-6)
        if rank == 3 and (0.25 * span) <= radius <= THUMB_ARC_MAX_RADIUS_M:
            # Find the start and end points along the fitted circle corresponding to u_min and u_max
            idx_start = int(np.argmin(u_coords))
            idx_end = int(np.argmax(u_coords))

            theta_start = float(np.arctan2(w_coords[idx_start] - w_c, u_coords[idx_start] - u_c))
            theta_end = float(np.arctan2(w_coords[idx_end] - w_c, u_coords[idx_end] - u_c))

            # Unwrap angular difference so we always traverse the short arc through the thumb taxels
            delta_theta = float(np.arctan2(np.sin(theta_end - theta_start), np.cos(theta_end - theta_start)))
            thetas = np.linspace(theta_start, theta_start + delta_theta, num_samples, dtype=np.float64)

            u_arc = u_c + radius * np.cos(thetas)
            w_arc = w_c + radius * np.sin(thetas)
            return u_arc, w_arc, radius

        # Fallback for near-collinear taxel profiles: 2nd-order polynomial arc fit
        poly_coeffs = np.polyfit(u_coords, w_coords, 2)
        u_arc = np.linspace(u_min, u_max, num_samples, dtype=np.float64)
        w_arc = np.polyval(poly_coeffs, u_arc)
        approx_curvature = abs(2.0 * float(poly_coeffs[0]))
        eff_radius = (1.0 / approx_curvature) if approx_curvature > 1e-6 else float("inf")
        return u_arc, w_arc, eff_radius

    @staticmethod
    def _generate_local_tilted_circle() -> NDArray[np.float64]:
        """
        Generates 100 equally distributed points on a 2D circle of radius `CIRCLE_RADIUS_M` (10 mm),
        whose plane is tilted by `CIRCLE_YAW_OFFSET_DEG` (+30 deg positive yaw) relative to the
        sensor origin, and whose center sits at:
            [CIRCLE_CENTER_X_OFFSET_M (+70 mm), CIRCLE_CENTER_Y_OFFSET_M, CIRCLE_CENTER_Z_OFFSET_M]
        in the sensor's local coordinate system (prior to the 360-deg x-axis roll sweep).

        Returns:
            NDArray of shape (CIRCLE_NUM_POINTS, 3) in sensor local coordinates.
        """
        angles = np.linspace(0.0, 2.0 * np.pi, CIRCLE_NUM_POINTS, endpoint=False, dtype=np.float64)

        # 2D circle parameterization (x_plane = R*cos(a), y_plane = 0, z_plane = R*sin(a))
        # tilted by +30 degrees yaw around the sensor's z-axis:
        yaw_rad = np.radians(CIRCLE_YAW_OFFSET_DEG)
        cos_yaw = float(np.cos(yaw_rad))
        sin_yaw = float(np.sin(yaw_rad))

        local_x = CIRCLE_CENTER_X_OFFSET_M + (CIRCLE_RADIUS_M * np.cos(angles) * cos_yaw)
        local_y = CIRCLE_CENTER_Y_OFFSET_M + (CIRCLE_RADIUS_M * np.cos(angles) * sin_yaw)
        local_z = CIRCLE_CENTER_Z_OFFSET_M + (CIRCLE_RADIUS_M * np.sin(angles))

        return np.column_stack([local_x, local_y, local_z])

    def _compute_global_circle_orientations(
        self,
        origin: SensorOriginEstimation,
    ) -> NDArray[np.float64]:
        """
        Rotates the 100 local circle points across all 360 roll orientations (0..359 deg)
        about the sensor's local x-axis, then transforms each point into the global
        palm reference coordinate system.

        Returns:
            NDArray of shape (360, 100, 3) containing the global 3D coordinates of each
            circle point for every roll orientation.
        """
        if (
            origin.origin_xyz_ref is None
            or origin.sensor_x_axis_3d is None
            or origin.sensor_y_axis_3d is None
            or origin.sensor_z_axis_3d is None
        ):
            raise ValueError("SensorOriginEstimation must have valid 3D origin and basis vectors.")

        base_circle_local = self._generate_local_tilted_circle()  # shape (100, 3)
        lx = base_circle_local[:, 0]  # (100,)
        ly = base_circle_local[:, 1]  # (100,)
        lz = base_circle_local[:, 2]  # (100,)

        roll_deg = np.arange(ROLL_NUM_ORIENTATIONS, dtype=np.float64)  # (360,)
        roll_rad = np.radians(roll_deg)
        cos_r = np.cos(roll_rad)[:, np.newaxis]  # (360, 1)
        sin_r = np.sin(roll_rad)[:, np.newaxis]  # (360, 1)

        # Rotate about the sensor's local x-axis: R_x(roll)
        rx_local = np.broadcast_to(lx[np.newaxis, :], (ROLL_NUM_ORIENTATIONS, CIRCLE_NUM_POINTS))
        ry_local = (cos_r * ly[np.newaxis, :]) - (sin_r * lz[np.newaxis, :])
        rz_local = (sin_r * ly[np.newaxis, :]) + (cos_r * lz[np.newaxis, :])

        # Stack into (360, 100, 3) in sensor local coordinates
        rotated_local = np.stack([rx_local, ry_local, rz_local], axis=-1)

        # Transform to global palm reference frame:
        # P_global = origin_xyz_ref + x_local * u_x + y_local * u_y + z_local * u_z
        basis_matrix = np.column_stack(
            [origin.sensor_x_axis_3d, origin.sensor_y_axis_3d, origin.sensor_z_axis_3d]
        )  # shape (3, 3), columns are local axes in global coordinates

        global_orientations = origin.origin_xyz_ref + (rotated_local @ basis_matrix.T)
        return global_orientations  # shape (360, 100, 3)

    @staticmethod
    def _min_distances_per_orientation(
        global_circle_orientations: NDArray[np.float64],
        thumb_points_ref: NDArray[np.float64],
    ) -> tuple[NDArray[np.float64], NDArray[np.int64]]:
        """
        For each of the 360 orientations, computes the Euclidean distance from every
        point of the 100-point circle to every point on the thumb comparison surface,
        and returns the minimum distance (and winning circle point index) per orientation.

        Args:
            global_circle_orientations: shape (360, 100, 3)
            thumb_points_ref: shape (M, 3)

        Returns:
            min_dists: shape (360,)
            best_circle_point_indices: shape (360,)
        """
        # Pairwise squared Euclidean distance via ||c - t||^2 = ||c||^2 + ||t||^2 - 2 c.t
        c_sq = np.sum(global_circle_orientations**2, axis=-1, keepdims=True)  # (360, 100, 1)
        t_sq = np.sum(thumb_points_ref**2, axis=-1)[np.newaxis, np.newaxis, :]  # (1, 1, M)
        cross_term = np.matmul(global_circle_orientations, thumb_points_ref.T)  # (360, 100, M)

        dist_sq = np.maximum(c_sq + t_sq - (2.0 * cross_term), 0.0)
        dists = np.sqrt(dist_sq)  # (360, 100, M)

        # Minimum distance to the thumb for each (orientation, circle_point): shape (360, 100)
        min_dist_per_circle_pt = np.min(dists, axis=-1)

        # Pick the circle point with the lowest distance for each orientation: shape (360,)
        best_circle_pt_idx = np.argmin(min_dist_per_circle_pt, axis=-1)
        orientation_min_dists = min_dist_per_circle_pt[
            np.arange(global_circle_orientations.shape[0]), best_circle_pt_idx
        ]
        return orientation_min_dists, best_circle_pt_idx

    def _calculate_sensor_roll(self) -> RollEstimation:
        """
        Evaluates all 360 roll orientations of the 100-point sensor circle against
        the fitted thumb comparison surfaces from Probe 1 and Probe 2, and selects
        the orientation with the lowest mean minimum distance across both probes.
        """
        origin = self.output.origin
        if origin is None or not origin.valid or origin.origin_xyz_ref is None:
            logger.warning("Cannot calculate sensor roll: sensor origin is not valid.")
            return RollEstimation(
                valid=False,
                best_roll_deg=None,
                best_roll_rad=None,
                min_mean_distance_m=None,
                probe_1_min_distance_m=None,
                probe_2_min_distance_m=None,
            )

        if len(self.output.probes) < ROLL_REQUIRED_PROBES:
            logger.warning(
                "Cannot calculate sensor roll: need %d probes, have %d.",
                ROLL_REQUIRED_PROBES,
                len(self.output.probes),
            )
            return RollEstimation(
                valid=False,
                best_roll_deg=None,
                best_roll_rad=None,
                min_mean_distance_m=None,
                probe_1_min_distance_m=None,
                probe_2_min_distance_m=None,
            )

        probe_1 = self.output.probes[0]
        probe_2 = self.output.probes[1]

        if (
            probe_1.thumb_comparison_points_ref is None
            or probe_1.thumb_comparison_points_ref.size == 0
            or probe_2.thumb_comparison_points_ref is None
            or probe_2.thumb_comparison_points_ref.size == 0
        ):
            logger.warning("Cannot calculate sensor roll: Probe 1 or Probe 2 has no thumb comparison points.")
            return RollEstimation(
                valid=False,
                best_roll_deg=None,
                best_roll_rad=None,
                min_mean_distance_m=None,
                probe_1_min_distance_m=None,
                probe_2_min_distance_m=None,
            )

        # 1. Compute global coordinates for all 100 circle points across all 360 orientations
        global_orientations = self._compute_global_circle_orientations(origin)  # (360, 100, 3)

        # 2. Compute lowest distances for Probe 1 and Probe 2 at each of the 360 orientations
        p1_min_dists, p1_best_pt_idx = self._min_distances_per_orientation(
            global_orientations, probe_1.thumb_comparison_points_ref
        )
        p2_min_dists, p2_best_pt_idx = self._min_distances_per_orientation(
            global_orientations, probe_2.thumb_comparison_points_ref
        )

        # 3. Mean of Probe 1 and Probe 2 minimum distances per orientation
        mean_dists = 0.5 * (p1_min_dists + p2_min_dists)  # shape (360,)

        # 4. Select the orientation with the lowest mean distance
        best_idx = int(np.argmin(mean_dists))
        best_roll_deg = float(best_idx)
        best_roll_rad = float(np.radians(best_roll_deg))

        return RollEstimation(
            valid=True,
            best_roll_deg=best_roll_deg,
            best_roll_rad=best_roll_rad,
            min_mean_distance_m=float(mean_dists[best_idx]),
            probe_1_min_distance_m=float(p1_min_dists[best_idx]),
            probe_2_min_distance_m=float(p2_min_dists[best_idx]),
            probe_1_closest_circle_pt_ref=global_orientations[best_idx, int(p1_best_pt_idx[best_idx])].copy(),
            probe_2_closest_circle_pt_ref=global_orientations[best_idx, int(p2_best_pt_idx[best_idx])].copy(),
            orientation_mean_distances_m=mean_dists,
            best_circle_points_ref=global_orientations[best_idx].copy(),
        )

    # ------------------------------------------------------------------------
    # CORE COMPUTATION: SENSOR 1 VS SENSOR 2 CLASSIFICATION
    # ------------------------------------------------------------------------
    def _update_sensor_decision(self, force_complete: bool = False) -> None:
        probes = self.output.probes
        if not probes:
            self.output.decision = SensorDecision(is_complete=False)
            return

        shallow_count = sum(1 for p in probes if p.is_shallow)
        deep_count = len(probes) - shallow_count
        mean_inward = float(np.mean([p.inward_angle for p in probes]))

        # Combine per-probe logistic probabilities (equally weighted across collected steps)
        s1_prob = float(np.mean([p.sensor_1_Step_prob for p in probes]))
        s2_prob = 1.0 - s1_prob

        is_complete = force_complete or (len(probes) >= NUM_PROBE_STEPS)
        predicted: Literal["sensor_1", "sensor_2"] = (
            "sensor_1" if s1_prob >= 0.5 else "sensor_2"
        )

        self.output.decision = SensorDecision(
            is_complete=is_complete,
            predicted_sensor=predicted,
            sensor_1_probability=s1_prob,
            sensor_2_probability=s2_prob,
            shallow_probe_count=shallow_count,
            deep_probe_count=deep_count,
            mean_inward_angle=mean_inward,
        )

    # ------------------------------------------------------------------------
    # INTERNAL VALIDATION & RESET HELPERS
    # ------------------------------------------------------------------------
    def _get_validated_matrix(
        self,
        sensor_name: str,
        expected_shape: tuple[int, int],
    ) -> NDArray[np.float32]:
        if not hasattr(self.robot, "matrices") or sensor_name not in self.robot.matrices:
            raise KeyError(f"Required tactile matrix {sensor_name!r} is missing from robot.matrices.")

        mat = np.asarray(self.robot.matrices[sensor_name])
        if mat.shape != expected_shape:
            raise ValueError(
                f"Tactile matrix {sensor_name!r} has shape {mat.shape}, expected {expected_shape}."
            )
        return mat

    def _read_thumb_angles(self) -> NDArray[np.float64]:
        if not hasattr(self.robot, "results") or "angles" not in self.robot.results:
            raise KeyError("robot.results['angles'] is missing.")

        angles = np.asarray(self.robot.results["angles"], dtype=np.float64).ravel()
        if angles.size < REQUIRED_ANGLES_LEN:
            raise ValueError(
                f"robot.results['angles'] must have at least {REQUIRED_ANGLES_LEN} elements, "
                f"got {angles.size}."
            )

        thumb_pair = np.array(
            [angles[THUMB_INWARD_IDX], angles[THUMB_POSITION_IDX]],
            dtype=np.float64,
        )
        if not np.all(np.isfinite(thumb_pair)):
            raise ValueError(f"Non-finite thumb angles encountered: {thumb_pair}")

        return thumb_pair

    def _reset_probing_trackers(self, clear_recorded_probes: bool = False) -> None:
        self._probing_tick = 0
        self._prev_thumb_angles = None
        self._latched_station_angle = None
        self._station_is_latched = False
        if clear_recorded_probes:
            self.output.probes.clear()
            self.output.decision = SensorDecision()
            self.output.roll = None

    def _full_reset(self) -> None:
        self.output = EstimatorOutput(current_state=self.output.current_state)
        self._reset_probing_trackers(clear_recorded_probes=True)