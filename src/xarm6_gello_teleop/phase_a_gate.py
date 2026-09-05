from __future__ import annotations

from dataclasses import dataclass
from math import copysign

from .config import RelativeCalibration
from .types import JOINT_NAMES, TeleopTarget, six


@dataclass(frozen=True)
class GateUpdate:
    requested: TeleopTarget
    active_index: int | None
    activated_index: int | None
    released_index: int | None
    raw_deltas: tuple[int, int, int, int, int, int]


@dataclass
class DominantAxisGate:
    """Sequential six-axis gate for a mechanically coupled passive leader.

    One leader joint is selected when its displacement from the last release
    reference exceeds its calibration deadband.  That joint remains selected
    until it has been still for ``release_idle_s``.  The next joint can then be
    selected without undoing previously latched xArm targets.
    """

    calibration: RelativeCalibration
    reference_raw: dict[str, int]
    latched_joints_rad: tuple[float, float, float, float, float, float]
    release_idle_s: float
    _active_index: int | None = None
    _active_anchor_joints_rad: tuple[float, float, float, float, float, float] | None = None
    _previous_active_raw: int | None = None
    _idle_started_s: float | None = None

    def __post_init__(self) -> None:
        if self.release_idle_s <= 0:
            raise ValueError("release_idle_s must be positive")
        self.reference_raw = {name: int(self.reference_raw[name]) for name in JOINT_NAMES}
        self.latched_joints_rad = six(self.latched_joints_rad)

    def update(self, raw: dict[str, int], now_s: float) -> GateUpdate:
        current = {name: int(raw[name]) for name in JOINT_NAMES}
        raw_deltas = tuple(current[name] - self.reference_raw[name] for name in JOINT_NAMES)
        activated_index = None
        released_index = None

        if self._active_index is None:
            magnitudes = tuple(abs(delta) for delta in raw_deltas)
            candidate = max(range(6), key=lambda index: magnitudes[index])
            if magnitudes[candidate] <= self.calibration.joint_deadband_raw[candidate]:
                return GateUpdate(
                    TeleopTarget(self.latched_joints_rad, 0.0), None, None, None, raw_deltas
                )
            self._active_index = candidate
            self._active_anchor_joints_rad = self.latched_joints_rad
            self._previous_active_raw = current[JOINT_NAMES[candidate]]
            self._idle_started_s = None
            activated_index = candidate

        assert self._active_index is not None
        assert self._active_anchor_joints_rad is not None
        active_index = self._active_index
        active_name = JOINT_NAMES[active_index]
        raw_delta = float(current[active_name] - self.reference_raw[active_name])
        deadband = self.calibration.joint_deadband_raw[active_index]
        effective_delta = 0.0 if abs(raw_delta) <= deadband else copysign(abs(raw_delta) - deadband, raw_delta)
        joint_delta = (
            effective_delta
            / self.calibration.counts_per_turn
            * self.calibration.sign[active_index]
            * self.calibration.gain_rad_per_turn[active_index]
        )
        requested_joints = list(self._active_anchor_joints_rad)
        requested_joints[active_index] += joint_delta

        if current[active_name] != self._previous_active_raw:
            self._idle_started_s = None
            self._previous_active_raw = current[active_name]
        elif self._idle_started_s is None:
            self._idle_started_s = now_s
        elif now_s - self._idle_started_s >= self.release_idle_s:
            released_index = active_index
            self.reference_raw = current
            self._active_index = None
            self._active_anchor_joints_rad = None
            self._previous_active_raw = None
            self._idle_started_s = None

        return GateUpdate(
            TeleopTarget(six(requested_joints), 0.0),
            active_index,
            activated_index,
            released_index,
            raw_deltas,
        )

    def latch_limited_joints(self, joints_rad: tuple[float, ...]) -> None:
        """Keep the safety-limited output as the base for the next selected axis."""
        self.latched_joints_rad = six(joints_rad)
