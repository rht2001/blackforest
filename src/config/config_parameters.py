from enum import IntEnum
from typing import Literal

REGISTERS = {
    # System / Configuration
    "HAND_ID": 1000,
    "REDU_RATIO": 1002,
    "CLEAR_ERROR": 1004,
    "SAVE": 1005,
    "RESET_PARA": 1006,
    "GESTURE_FORCE_CLB": 1009,

    "DEFAULT_SPEED_SET": 1032,
    "DEFAULT_FORCE_SET": 1044,

    # Control / Commands
    "POS_SET": 1474,
    "ANGLE_SET": 1486,
    "FORCE_SET": 1498,
    "SPEED_SET": 1522,

    # Telemetry / Status
    "POS_ACT": 1534,
    "ANGLE_ACT": 1546,
    "FORCE_ACT": 1582,
    "CURRENT": 1594,
    "ERROR": 1606,
    "STATUS": 1612,
    "TEMP": 1618,

    # Network / IP
    "IP_PART1": 1700,
    "IP_PART2": 1701,
    "IP_PART3": 1702,
    "IP_PART4": 1703,

    # Tactile / Touch Sensors
    "FINGERONE_TOUCH": 3000,
    "FINGERTWO_TOUCH": 3370,
    "FINGERTHE_TOUCH": 3740,
    "FINGERFOR_TOUCH": 4110,
    "FINGERFIV_TOUCH": 4480,
    "FINGERPALM_TOUCH": 4900,
}
# String literal type if you pass names as strings
RegisterName = Literal[
    "HAND_ID",
    "REDU_RATIO",
    "CLEAR_ERROR",
    "SAVE",
    "RESET_PARA",
    "GESTURE_FORCE_CLB",
    "DEFAULT_SPEED_SET",
    "DEFAULT_FORCE_SET",
    "POS_SET",
    "ANGLE_SET",
    "FORCE_SET",
    "SPEED_SET",
    "POS_ACT",
    "ANGLE_ACT",
    "FORCE_ACT",
    "CURRENT",
    "ERROR",
    "STATUS",
    "TEMP",
    "IP_PART1",
    "IP_PART2",
    "IP_PART3",
    "IP_PART4",
    "FINGERONE_TOUCH",
    "FINGERTWO_TOUCH",
    "FINGERTHE_TOUCH",
    "FINGERFOR_TOUCH",
    "FINGERFIV_TOUCH",
    "FINGERPALM_TOUCH",
]
from typing import TypedDict, Literal
import numpy as np
import numpy.typing as npt

STATES = Literal[
    "starting",
    "initialised",
    "open",
    "closing",
    "thumb_closing",
    "settling",
    "steady_state",
    "cylinder_confirmed",
    "resetting",
]
class DataDict(TypedDict):
    angles: np.ndarray
    forces: np.ndarray
    position: np.ndarray
    tactile_matrix: np.ndarray
    tactile_mean_matrix: np.ndarray
    state: STATES
    state_change: bool


default_results: DataDict = {
    "angles": np.zeros(6, dtype=int),
    "forces": np.zeros(6, dtype=float),
    "position": np.zeros(6, dtype=int),
    "tactile_matrix": np.zeros((112, 17), dtype=float),
    "tactile_mean_matrix": np.zeros((1, 17), dtype=float),
    "state": "starting",
    "state_change": False
}
