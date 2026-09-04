from __future__ import annotations

from dataclasses import dataclass

from .types import TeleopTarget, six


@dataclass(frozen=True)
class JointSafetyLimiter:
    lower_rad: tuple[float, float, float, float, float, float]
    upper_rad: tuple[float, float, float, float, float, float]
    max_delta_rad: float
    max_velocity_rad_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "lower_rad", six(self.lower_rad))
        object.__setattr__(self, "upper_rad", six(self.upper_rad))
        if self.max_delta_rad <= 0 or self.max_velocity_rad_s <= 0:
            raise ValueError("Safety limits must be positive")
        if any(lower >= upper for lower, upper in zip(self.lower_rad, self.upper_rad, strict=True)):
            raise ValueError("Each joint lower limit must be below its upper limit")

    def apply(self, requested: TeleopTarget, previous_joints_rad: tuple[float, ...], dt_s: float) -> TeleopTarget:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        previous = six(previous_joints_rad)
        allowed_delta = min(self.max_delta_rad, self.max_velocity_rad_s * dt_s)
        limited = []
        for requested_value, previous_value, lower, upper in zip(requested.joints_rad, previous, self.lower_rad, self.upper_rad, strict=True):
            bounded = min(upper, max(lower, requested_value))
            limited.append(previous_value + min(allowed_delta, max(-allowed_delta, bounded - previous_value)))
        return TeleopTarget(six(limited), requested.gripper_open)
