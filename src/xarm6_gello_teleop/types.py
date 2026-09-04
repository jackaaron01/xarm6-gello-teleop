from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3")


def six(values: Iterable[float]) -> tuple[float, float, float, float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 6:
        raise ValueError(f"Expected 6 joints, got {len(result)}")
    return result  # type: ignore[return-value]


@dataclass(frozen=True)
class TeleopTarget:
    joints_rad: tuple[float, float, float, float, float, float]
    gripper_open: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "joints_rad", six(self.joints_rad))
        if not 0.0 <= self.gripper_open <= 1.0:
            raise ValueError("gripper_open must be within [0, 1]")
