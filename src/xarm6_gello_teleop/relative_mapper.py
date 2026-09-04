from __future__ import annotations

from dataclasses import dataclass

from .config import RelativeCalibration
from .types import JOINT_NAMES, TeleopTarget, six


@dataclass
class RelativeMapper:
    """Maps a passive leader's raw encoder deltas into xArm joint targets."""

    calibration: RelativeCalibration
    _zero_raw: dict[str, int] | None = None
    _zero_joints_rad: tuple[float, float, float, float, float, float] | None = None

    def align(self, raw_positions: dict[str, int], xarm_joints_rad: tuple[float, ...]) -> None:
        required = (*JOINT_NAMES, self.calibration.gripper_motor_name)
        missing = set(required).difference(raw_positions)
        if missing:
            raise ValueError(f"Leader data is missing motors: {sorted(missing)}")
        self._zero_raw = {name: int(raw_positions[name]) for name in required}
        self._zero_joints_rad = six(xarm_joints_rad)

    @property
    def aligned(self) -> bool:
        return self._zero_raw is not None and self._zero_joints_rad is not None

    def target(self, raw_positions: dict[str, int]) -> TeleopTarget:
        if not self.aligned:
            raise RuntimeError("Session zero is not aligned")
        assert self._zero_raw is not None
        assert self._zero_joints_rad is not None
        joints = []
        for index, name in enumerate(JOINT_NAMES):
            raw_delta = float(raw_positions[name] - self._zero_raw[name])
            if abs(raw_delta) <= self.calibration.joint_deadband_raw[index]:
                raw_delta = 0.0
            delta_rad = raw_delta / self.calibration.counts_per_turn * self.calibration.sign[index] * self.calibration.gain_rad_per_turn[index]
            joints.append(self._zero_joints_rad[index] + delta_rad)
        raw_gripper = float(raw_positions[self.calibration.gripper_motor_name])
        raw_fraction = (raw_gripper - self.calibration.gripper_raw_closed) / (self.calibration.gripper_raw_open - self.calibration.gripper_raw_closed)
        return TeleopTarget(six(joints), min(1.0, max(0.0, raw_fraction)))
