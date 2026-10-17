"""Fixed telemetry schema for one acquisition run; None means unavailable."""
from typing import TypedDict


class robot_outputs(TypedDict):
    simulated: bool
    timestamp_ns: int
    angles: list[int] | None
    forces: list[int] | None
    positions: list[int] | None
    tactile: list[list[int | None]]
    tactile_valid: list[bool]

