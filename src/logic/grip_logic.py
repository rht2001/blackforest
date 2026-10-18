import numpy as np
from typing import Any

# Sensor groups across all 17 channels
SENSOR_NAMES = [
    "Little Tip", "Little Nail", "Little Pad",
    "Ring Tip", "Ring Nail", "Ring Pad",
    "Middle Tip", "Middle Nail", "Middle Pad",
    "Index Tip", "Index Nail", "Index Pad",
    "Thumb Tip", "Thumb Nail", "Thumb Mid", "Thumb Pad",
    "Palm"
]

FINGER_SENSORS: dict[str, list[int]] = {
    "Little": [0, 1, 2],    # Tip, Nail, Pad
    "Ring": [3, 4, 5],      # Tip, Nail, Pad
    "Middle": [6, 7, 8],    # Tip, Nail, Pad
    "Index": [9, 10, 11],   # Tip, Nail, Pad
    "Thumb": [12, 13, 14, 15], # Tip, Nail, Mid, Pad
    "Palm": [16],
}

PRIMARY_FINGERS = ["Little", "Ring", "Middle", "Index"]


def extract_all_sensor_pressures(
    mean_matrix: np.ndarray,
    tactile_matrix: np.ndarray | None = None,
    cell_threshold: float = 50.0,
) -> dict[str, float]:
    """
    Extracts effective contact pressure for all 17 sensor groups.
    Requires at least 2 active cells (> cell_threshold) to confirm true physical contact,
    matching the UI visualizer and completely rejecting single-taxel noise/drift.
    """
    pressures: dict[str, float] = {}

    for idx, name in enumerate(SENSOR_NAMES):
        if tactile_matrix is not None and idx < tactile_matrix.shape[1]:
            col = tactile_matrix[:, idx]
            active_cells = col[col > cell_threshold]
            if len(active_cells) >= 2:
                pressures[name] = float(np.mean(active_cells))
            else:
                pressures[name] = 0.0
        else:
            pressures[name] = 0.0

    return pressures


CYLINDER_FINGER_SENSORS: dict[str, str] = {
    "Little": "Little Nail",
    "Ring": "Ring Nail",
    "Middle": "Middle Nail",
    "Index": "Index Nail",
}


def get_finger_pressures(
    all_pressures: dict[str, float],
) -> dict[str, float]:
    """
    Computes effective cylindrical contact pressure per finger.
    Focuses on the Nail sensors which physically establish contact
    with the cylinder during closed curls, plus Palm.
    """
    finger_pressures: dict[str, float] = {}
    for finger, sensor_name in CYLINDER_FINGER_SENSORS.items():
        finger_pressures[finger] = float(all_pressures.get(sensor_name, 0.0))
    finger_pressures["Palm"] = float(all_pressures.get("Palm", 0.0))
    return finger_pressures


def get_pad_pressures(
    mean_matrix: np.ndarray,
    tactile_matrix: np.ndarray | None = None,
) -> dict[str, float]:
    """Backward-compatibility facade returning finger Nail and Palm contact pressures."""
    all_p = extract_all_sensor_pressures(mean_matrix, tactile_matrix)
    return get_finger_pressures(all_p)


def is_empty_grasp(
    finger_angles: np.ndarray,
    finger_pressures: dict[str, float],
    empty_angle_threshold: int = 60,
    min_contact_pressure: float = 60.0,
) -> bool:
    """
    Returns True only if the hand closed into an empty fist without contact:
    fingers reached near bottom AND tactile sensors confirm NO object is present.
    If tactile sensors detect active contact (>=2 fingers or total force >= 250g),
    it is never an empty grasp regardless of finger angles.
    """
    angles = np.asarray(finger_angles)[:4]
    palm_pressure = finger_pressures.get("Palm", 0.0)

    active_fingers = sum(
        1 for f in PRIMARY_FINGERS if finger_pressures.get(f, 0.0) >= min_contact_pressure
    )
    total_force = sum(finger_pressures.get(f, 0.0) for f in PRIMARY_FINGERS) + palm_pressure

    # If tactile sensors detect significant contact, an object is present
    if active_fingers >= 2 or total_force >= 250.0:
        return False

    # Empty fist: all fingers curled low (< 60) AND no tactile contact
    if np.all(angles < empty_angle_threshold) and active_fingers == 0 and total_force < 100.0:
        return True

    # Empty fist: mean angle at bottom (< 35) with at most 1 weak contact and low total force
    if np.mean(angles) < 35 and active_fingers <= 1 and total_force < 100.0:
        return True

    return False


def confirm_grip(
    finger_angles: np.ndarray,
    tactile_mean_matrix: np.ndarray,
    tactile_matrix: np.ndarray | None = None,
    valid_positions: np.ndarray | None = None,
    valid_tactile: np.ndarray | None = None,
    angle_tolerance: int = 75,
    min_active_pads: int = 3,
    pad_force_threshold: float = 60.0,
    force_spread_ratio: float = 6.0,
    min_grasp_angle: int = 50,
    pos_tolerance: float = 0.15,
    pos_atol: float = 30.0,
    tactile_tolerance: float = 0.25,
    tactile_atol: float = 20.0,
) -> tuple[bool, str, dict[str, Any]]:
    """
    Evaluates whether a stable cylindrical grasp has occurred across primary fingers 0..3.
    Rejects grasps where any finger curled completely to the palm (misaligned/vertical object)
    or where contact is severely unbalanced.
    """
    angles = np.asarray(finger_angles)[:4]
    all_sensor_pressures = extract_all_sensor_pressures(
        tactile_mean_matrix, tactile_matrix=tactile_matrix
    )
    finger_pressures = get_finger_pressures(all_sensor_pressures)

    spread = int(np.max(angles) - np.min(angles))
    active_fingers = {
        f: finger_pressures[f]
        for f in PRIMARY_FINGERS
        if finger_pressures.get(f, 0.0) >= pad_force_threshold
    }

    active_sensors = {
        name: round(val, 1)
        for name, val in all_sensor_pressures.items()
        if val >= pad_force_threshold
    }

    diagnostics: dict[str, Any] = {
        "angles": angles.tolist(),
        "spread": spread,
        "finger_pressures": {k: round(v, 2) for k, v in finger_pressures.items()},
        "active_fingers": list(active_fingers.keys()),
        "active_finger_count": len(active_fingers),
        "active_sensors": active_sensors,
    }

    # 1. Check for missed fingers: A finger missed the object only if it closed near bottom (< 30) AND has no contact
    missed_fingers = [
        f for i, f in enumerate(PRIMARY_FINGERS)
        if angles[i] < 30 and finger_pressures.get(f, 0.0) < pad_force_threshold
    ]
    if missed_fingers:
        return (
            False,
            f"Misaligned grasp: Finger(s) {missed_fingers} curled into palm with no contact ({angles.tolist()}).",
            diagnostics,
        )


    # 2. Open hand check
    if all(a > 900 for a in angles):
        return False, "Hand still open: No contact detected.", diagnostics

    # 3. Angle spread homogeneity check across all 4 primary fingers
    if spread > angle_tolerance:
        return False, f"Non-homogeneous grasp: Angle spread {spread} exceeds tolerance {angle_tolerance}.", diagnostics

    # 4. Finger contact check (across Tip, Nail, and Pad patches)
    if len(active_fingers) < min_active_pads:
        return (
            False,
            f"Insufficient tactile contact: {len(active_fingers)}/{min_active_pads} fingers engaged ({list(active_fingers.keys())}).",
            diagnostics,
        )

    # 5. Tactile force consistency among active contacting fingers
    if len(active_fingers) >= 2:
        forces = list(active_fingers.values())
        max_f = max(forces)
        min_f = max(min(forces), 1.0)
        ratio = max_f / min_f
        diagnostics["force_ratio"] = round(ratio, 2)
        if ratio > force_spread_ratio:
            return (
                False,
                f"Tactile force disparity too high among fingers (ratio {ratio:.1f} > {force_spread_ratio}).",
                diagnostics,
            )


    # 6. Optional reference positions comparison
    if valid_positions is not None:
        ref_angles = np.asarray(valid_positions)[:4]
        diff_angles = np.abs(ref_angles - angles)
        allowed_diff = pos_tolerance * np.abs(ref_angles) + pos_atol
        if not np.all(diff_angles <= allowed_diff):
            return (
                False,
                f"Angles do not match reference profile (diff: {diff_angles.tolist()}, max allowed: {allowed_diff.tolist()}).",
                diagnostics,
            )

    # 7. Optional reference tactile comparison
    if valid_tactile is not None:
        flat_mean = np.asarray(tactile_mean_matrix).flatten()
        flat_ref = np.asarray(valid_tactile).flatten()
        min_len = min(len(flat_mean), len(flat_ref))
        diff_tactile = np.abs(flat_ref[:min_len] - flat_mean[:min_len])
        allowed_tactile = tactile_tolerance * np.abs(flat_ref[:min_len]) + tactile_atol
        if not np.all(diff_tactile <= allowed_tactile):
            return (
                False,
                "Tactile matrix does not match reference template profile.",
                diagnostics,
            )

    return True, "Stable cylindrical grasp confirmed.", diagnostics
