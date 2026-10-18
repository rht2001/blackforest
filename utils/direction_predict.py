from __future__ import annotations

import ast
import json
from pathlib import Path
from typing import Any, Optional

import numpy as np
import pandas as pd
from numpy.typing import NDArray

# Import your estimator class (adjust module path if stored in a separate file)
from logic.orientation_logic import TactilePoseAndSensorEstimator, STATES

# ============================================================================
# SENSOR GROUPS DEFINITION
# ============================================================================
SENSOR_GROUPS: list[tuple[str, int, int, int, str, tuple[int, int]]] = [
    ("Little Tip", 3000, 3, 3, "Little", (8, 10)),
    ("Little Nail", 3018, 12, 8, "Little", (-2, 40)),
    ("Little Pad", 3210, 10, 8, "Little", (4, 190)),

    ("Ring Tip", 3370, 3, 3, "Ring", (-9, -20)),
    ("Ring Nail", 3388, 12, 8, "Ring", (-23, 30)),
    ("Ring Pad", 3580, 10, 8, "Ring", (-18, 200)),

    ("Middle Tip", 3740, 3, 3, "Middle", (-21, -25)),
    ("Middle Nail", 3758, 12, 8, "Middle", (-39, 22)),
    ("Middle Pad", 3950, 10, 8, "Middle", (-35, 210)),

    ("Index Tip", 4110, 3, 3, "Index", (-38, -19)),
    ("Index Nail", 4128, 12, 8, "Index", (-49, 17)),
    ("Index Pad", 4320, 10, 8, "Index", (-54, 200)),

    ("Thumb Tip", 4480, 3, 3, "Thumb", (185, 140)),
    ("Thumb Nail", 4498, 12, 8, "Thumb", (145, 170)),
    ("Thumb Mid", 4690, 3, 3, "Thumb", (90, 240)),
    ("Thumb Pad", 4708, 12, 8, "Thumb", (-30, 240)),

    ("Palm", 4900, 8, 14, "Palm", (-15, 0)),
]

ALL_POINTS_SHAPE: tuple[int, int] = (112, 17)
ALL_POINTS_FLAT_SIZE: int = ALL_POINTS_SHAPE[0] * ALL_POINTS_SHAPE[1]  # 1904
TOTAL_TAXEL_COUNT: int = sum(h * w for _, _, h, w, *_ in SENSOR_GROUPS)  # 1062


# ============================================================================
# MOCK / REPLAY ROBOT CONTROLLER FOR OFFLINE DATASET
# ============================================================================
class ReplayRobotHandController:
    """
    Lightweight stand-in for RobotHandController during CSV replay.
    Holds the current row's `.state`, `.matrices`, and `.results['angles']`.
    """

    def __init__(self) -> None:
        self.sensor_groups = SENSOR_GROUPS
        self.matrices: dict[str, NDArray[np.float32]] = {
            name: np.zeros((rows, cols), dtype=np.float32)
            for name, _, rows, cols, *_ in self.sensor_groups
        }
        self.results: dict[str, Any] = {
            "angles": np.zeros(6, dtype=np.float64),
        }
        self.state: Optional[STATES] = None


# ============================================================================
# PARSING & MATRIX RECONSTRUCTION HELPERS
# ============================================================================
def parse_serialized_array(raw_val: Any, row_idx: int, col_name: str) -> NDArray[np.float64]:
    """
    Parses a serialized array from a CSV cell (handles Python lists, JSON,
    or space/comma-separated numpy string outputs).
    """
    if isinstance(raw_val, np.ndarray):
        return raw_val.astype(np.float64)
    if isinstance(raw_val, (list, tuple)):
        return np.asarray(raw_val, dtype=np.float64)

    if pd.isna(raw_val):
        raise ValueError(f"Row {row_idx}: Column {col_name!r} is NaN/empty.")

    text = str(raw_val).strip()
    if "..." in text:
        raise ValueError(
            f"Row {row_idx}: Column {col_name!r} contains '...' truncation. "
            "Re-export the CSV with np.set_printoptions(threshold=sys.maxsize)."
        )

    # 1. Fast path for flat bracketed arrays (space or comma separated)
    cleaned = text.replace("[", " ").replace("]", " ").replace("\n", " ").replace("\r", " ")
    sep = "," if "," in cleaned else " "
    arr = np.fromstring(cleaned, dtype=np.float64, sep=sep)
    if arr.size > 0:
        return arr

    # 2. Fallback to literal_eval if formatted in a non-standard structure
    try:
        parsed = ast.literal_eval(text)
        return np.asarray(parsed, dtype=np.float64)
    except Exception as exc:
        raise ValueError(
            f"Row {row_idx}: Could not parse array in column {col_name!r}: {text[:80]!r}"
        ) from exc


def reconstruct_matrices(
    tactile_matrix_raw: Any,
    row_idx: int = 0,
) -> dict[str, NDArray[np.float32]]:
    """
    Rebuilds the `matr` dictionary {sensor_name: ndarray(h, w)} from the
    flattened `tactile_matrix` column in the CSV.

    Supports both:
    - Standard `all_points` layout: shape (112, 17) -> 1,904 elements
      where `all_points[0:h*w, index]` holds sensor `index`.
    - Sequential layout: 1,062 elements concatenated in `SENSOR_GROUPS` order.
    """
    flat = parse_serialized_array(tactile_matrix_raw, row_idx, "tactile_matrix").ravel()
    matr: dict[str, NDArray[np.float32]] = {}

    if flat.size == ALL_POINTS_FLAT_SIZE:
        all_points = flat.reshape(ALL_POINTS_SHAPE)
        for index, (name, _, h, w, *_) in enumerate(SENSOR_GROUPS):
            length = h * w
            sensor_flat = all_points[0:length, index]
            matr[name] = sensor_flat.reshape((h, w)).astype(np.float32)

    elif flat.size == TOTAL_TAXEL_COUNT:
        offset = 0
        for name, _, h, w, *_ in SENSOR_GROUPS:
            length = h * w
            matr[name] = flat[offset : offset + length].reshape((h, w)).astype(np.float32)
            offset += length

    else:
        raise ValueError(
            f"Row {row_idx}: Unexpected `tactile_matrix` element count {flat.size}. "
            f"Expected {ALL_POINTS_FLAT_SIZE} (112x17 all_points) or {TOTAL_TAXEL_COUNT} (concatenated)."
        )

    return matr


def serialize_matrices_for_csv(matr: dict[str, NDArray[np.float32]]) -> str:
    """
    Serializes the reconstructed `{sensor_name: (h, w) array}` dictionary
    into a compact JSON string so it can be stored cleanly in the new CSV column.
    """
    return json.dumps({name: arr.tolist() for name, arr in matr.items()})


# ============================================================================
# MAIN DATASET FORWARD-STEP RUNNER
# ============================================================================
def process_dataset_csv(
    input_csv_path: str | Path,
    output_csv_path: Optional[str | Path] = None,
    serialize_matrices_in_df: bool = False,
) -> pd.DataFrame:
    """
    Loads the semicolon-separated dataset, reconstructs `robot.matrices` for each
    row, runs `TactilePoseAndSensorEstimator.main()` sequentially across all rows,
    and appends three new columns at the end:
        1. `matrices`
        2. `direction`  (from TaperEstimation.direction)
        3. `yaw_angle`  (from YawEstimation.yaw_deg)

    Args:
        input_csv_path: Path to the input `;`-delimited CSV file.
        output_csv_path: Optional path to write the updated `;`-delimited CSV.
        serialize_matrices_in_df: If True, stores `matrices` as JSON strings in
            the returned DataFrame; if False, keeps live dict[str, np.ndarray]
            objects in the DataFrame and only serializes to JSON when writing CSV.
    """
    df = pd.read_csv(input_csv_path, sep=";")
    print(df.head())
    required_cols = {"angles", "tactile_matrix", "state"}
    missing = required_cols - set(df.columns)
    if missing:
        raise KeyError(f"CSV is missing required columns: {sorted(missing)}")

    robot = ReplayRobotHandController()
    estimator = TactilePoseAndSensorEstimator(robot)

    matrices_col: list[Any] = []
    direction_col: list[Optional[str]] = []
    yaw_angle_col: list[Optional[float]] = []

    for row_idx, row in df.iterrows():
        # 1. Reconstruct robot.matrices from tactile_matrix
        matr = reconstruct_matrices(row["tactile_matrix"], row_idx=int(row_idx))
        robot.matrices = matr

        # 2. Populate robot.results["angles"] and robot.state
        robot.results["angles"] = parse_serialized_array(
            row["angles"], row_idx=int(row_idx), col_name="angles"
        )
        robot.state = str(row["state"]).strip()  # type: ignore[assignment]

        # 3. Execute forward step on the estimator
        output = estimator.main(kinematics=None)

        # 4. Extract taper direction and yaw_deg from estimator output
        taper_direction = (
            output.taper.direction
            if (output.taper is not None and output.taper.valid)
            else None
        )
        yaw_deg = (
            output.yaw.yaw_deg
            if (output.yaw is not None and output.yaw.valid)
            else None
        )

        matrices_col.append(
            serialize_matrices_for_csv(matr) if serialize_matrices_in_df else matr
        )
        direction_col.append(taper_direction)
        yaw_angle_col.append(yaw_deg)

    # Append the 3 new columns at the end of the table
    df["matrices"] = matrices_col
    df["direction"] = direction_col
    df["yaw_angle"] = yaw_angle_col

    if output_csv_path is not None:
        df_to_save = df.copy()
        if not serialize_matrices_in_df:
            df_to_save["matrices"] = [serialize_matrices_for_csv(m) for m in matrices_col]
        df_to_save.to_csv(output_csv_path, sep=";", index=False)

    return df


if __name__ == "__main__":
    # Example usage:
    INPUT_CSV = "main_dataset.csv"
    OUTPUT_CSV = "robot_recording_with_predictions.csv"

    if Path(INPUT_CSV).exists():
        processed_df = process_dataset_csv(INPUT_CSV, OUTPUT_CSV)
        print(processed_df[["index", "state", "direction", "yaw_angle"]].head(20))