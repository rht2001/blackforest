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
CLAMP_LOCAL_X_TO_SENSOR: bool = False # Set True to clamp extrapolated x_local to [-0.05, +0.05]

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
NUM_PROBE_STEPS: int = 5                  # Total number of thumb probes to collect
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
    origin_xy_ref: Optional[NDArray[np.float64]]      # (2,) [X_origin, Y_origin] in palm frame (meters)
    sensor_heading_rad: Optional[float]               # Signed heading of +x_sensor in palm xy-plane
    sensor_heading_deg: Optional[float]
    sensor_x_axis_unit_xy: Optional[NDArray[np.float64]]  # (2,) [cos(heading), sin(heading)]
    sensor_y_axis_unit_xy: Optional[NDArray[np.float64]]  # (2,) [-sin(heading), cos(heading)]
    index_contact_ref: Optional[NDArray[np.float64]]  # (3,) [x, y, z] of max-force Index Nail taxel
    index_local_x_m: Optional[float]                  # Extrapolated x_sensor offset (meters) at Index Nail
    nail_max_points: dict[str, NailMaxContactPoint] = field(default_factory=dict)


@dataclass(frozen=True)
class ThumbProbeStep:
    probe_number: int          # 1 to 5
    tick_recorded: int         # Tick index within thumb_probing when settled
    x_position_angle: float    # angles[5]: thumb position along sweep axis
    inward_angle: float        # angles[4]: thumb inward penetration angle
    is_shallow: bool           # True if inward_angle > THUMB_SHALLOW_ANGLE_THRESHOLD
    sensor_1_Step_prob: float  # Logistic confidence for Sensor 1 on this single step


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
    probes: list[ThumbProbeStep] = field(default_factory=list)
    decision: SensorDecision = field(default_factory=SensorDecision)


# ============================================================================
# MAIN LOGIC CONTROLLER
# ============================================================================
class TactilePoseAndSensorEstimator:
    """
    State-driven controller that estimates 4-DOF cylinder/sensor alignment cues
    (Palm taper, Finger Nail yaw, Sensor local origin & axes) and classifies
    Sensor 1 vs. Sensor 2 via a 5-step thumb probing routine.
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
           coordinate origin and axes in the palm reference frame.
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
        Executes on every tick while in 'thumb_probing' until 5 probes are collected:
        - Tick 0: Skips completely for safety.
        - Tick 1: Captures initial baseline thumb angles [angles[4], angles[5]].
        - Tick 2+: Checks if both thumb angles settled within THUMB_STABILITY_THRESHOLD (2.0).
          While angles[5] stays within THUMB_SAME_STATION_THRESHOLD (2.0) of the last
          recorded probe, holds off recording. Once at a new steady station, records
          the probe and updates Sensor 1 vs Sensor 2 probabilities on completion.
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

        # 6. Record validated steady probe step
        probe_num = len(self.output.probes) + 1
        is_shallow = inward_angle > THUMB_SHALLOW_ANGLE_THRESHOLD

        # Smooth logistic probability for this single probe:
        # > 0.5 when inward_angle > THUMB_SHALLOW_ANGLE_THRESHOLD (Sensor 1)
        # < 0.5 when inward_angle < THUMB_SHALLOW_ANGLE_THRESHOLD (Sensor 2)
        z = (inward_angle - THUMB_SHALLOW_ANGLE_THRESHOLD) / THUMB_PROB_TEMPERATURE
        step_s1_prob = float(1.0 / (1.0 + np.exp(-np.clip(z, -50.0, 50.0))))

        probe_step = ThumbProbeStep(
            probe_number=probe_num,
            tick_recorded=current_tick,
            x_position_angle=pos_angle,
            inward_angle=inward_angle,
            is_shallow=is_shallow,
            sensor_1_Step_prob=step_s1_prob,
        )
        self.output.probes.append(probe_step)

        # Latch current station so we don't record again until angles[5] moves
        self._latched_station_angle = pos_angle
        self._station_is_latched = True

        # 7. Update classification (marks complete when 5th probe arrives)
        self._update_sensor_decision()

    def _execute_probing_complete(self) -> None:
        """Finalizes sensor classification if transitioning out of probing."""
        self._update_sensor_decision(force_complete=True)

    def _execute_resetting(self) -> None:
        self._full_reset()

    # ------------------------------------------------------------------------
    # CORE COMPUTATION: PALM TAPER, NAIL YAW & SENSOR ORIGIN (X, Y)
    # ------------------------------------------------------------------------
    def _calculate_palm_taper(self) -> TaperEstimation:
        """
        Determines the taper direction along the hand's x-axis from the 8x14 Palm matrix.
        Columns 0 -> 13 correspond to +x (Little finger -> Index/Thumb).
        If force decreases left-to-right (negative slope), taper is '-x'.
        If force increases left-to-right (positive slope), taper is '+x'.
        """
        palm = self._get_validated_matrix(PALM_SENSOR_NAME, PALM_SHAPE)

        # Clean non-finite values and apply noise floor
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

        # Column force profile across the 14 columns (left to right = +x)
        col_forces = np.mean(active_palm, axis=0)  # shape (14,)
        col_indices = np.arange(PALM_SHAPE[1], dtype=np.float64)

        # Linear least-squares fit: col_force = slope * col_idx + intercept
        slope = float(np.polyfit(col_indices, col_forces, 1)[0])

        if abs(slope) <= PALM_SLOPE_EPSILON:
            direction: Literal["+x", "-x", "flat"] = "flat"
        elif slope < 0.0:
            # Force decreases from left to right -> -x taper
            direction = "-x"
        else:
            # Force increases from left to right -> +x taper
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

        Coordinate convention:
        - Columns 0..31 run left-to-right (+x: Little to Index).
        - Matrix rows 0..11 run top-to-bottom. Therefore, a line sloping UP
          from left to right has a positive cartesian_slope (dy/dx > 0) and a
          negative matrix_row_slope (d_row/d_col < 0).
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

        # Clean non-finite entries and apply strict thresholding
        clean_combined = np.where(np.isfinite(combined), combined, 0.0)
        thresholded = np.where(clean_combined >= NAIL_FORCE_THRESHOLD, clean_combined, 0.0)

        # Compute column-wise total force and force-weighted row centroids
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
        # Force-weighted row centroid per active column
        weighted_row_sums = np.sum(thresholded[:, active_cols] * row_indices, axis=0)
        row_centroids = weighted_row_sums / col_weights[active_cols]

        # Weighted linear regression: row_centroid = matrix_row_slope * col + intercept
        x_cols = active_cols.astype(np.float64)
        w = col_weights[active_cols]
        matrix_row_slope = float(np.polyfit(x_cols, row_centroids, 1, w=w)[0])

        # Convert matrix row slope (downward positive) to Cartesian y-up slope (+y is palm -> tip)
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
        Resolves the sensor's local coordinate system origin (X_origin, Y_origin)
        and axes in the palm reference frame using the Index Nail max-force taxel
        and sensor taper geometry.
        """
        if kinematics is None or not kinematics.base_taxels:
            logger.warning("Kinematics with base_taxels is required to calculate sensor origin.")
            return SensorOriginEstimation(
                valid=False,
                origin_xy_ref=None,
                sensor_heading_rad=None,
                sensor_heading_deg=None,
                sensor_x_axis_unit_xy=None,
                sensor_y_axis_unit_xy=None,
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
                sensor_heading_rad=None,
                sensor_heading_deg=None,
                sensor_x_axis_unit_xy=None,
                sensor_y_axis_unit_xy=None,
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

        u_x = np.array([np.cos(heading_rad), np.sin(heading_rad)], dtype=np.float64)
        u_y = np.array([-np.sin(heading_rad), np.cos(heading_rad)], dtype=np.float64)

        index_xy_ref = index_contact.position_ref[:2]
        origin_xy_ref = index_xy_ref - (index_local_x_m * u_x)

        return SensorOriginEstimation(
            valid=True,
            origin_xy_ref=origin_xy_ref,
            sensor_heading_rad=heading_rad,
            sensor_heading_deg=heading_deg,
            sensor_x_axis_unit_xy=u_x,
            sensor_y_axis_unit_xy=u_y,
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

    def _full_reset(self) -> None:
        self.output = EstimatorOutput(current_state=self.output.current_state)
        self._reset_probing_trackers(clear_recorded_probes=True)