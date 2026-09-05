from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionProfile:
    name: str
    max_delta_rad: float
    max_velocity_rad_s: float


SAFE_PROFILE = MotionProfile("safe", max_delta_rad=0.004, max_velocity_rad_s=0.20)
RESPONSIVE_PROFILE = MotionProfile("responsive", max_delta_rad=0.005, max_velocity_rad_s=0.25)
MOTION_PROFILES = {
    SAFE_PROFILE.name: SAFE_PROFILE,
    RESPONSIVE_PROFILE.name: RESPONSIVE_PROFILE,
}


def motion_profile(name: str) -> MotionProfile:
    if name not in MOTION_PROFILES:
        raise ValueError(f"Unknown motion profile: {name}")
    return MOTION_PROFILES[name]
