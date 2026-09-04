from __future__ import annotations

from dataclasses import dataclass

from xarm.wrapper import XArmAPI

from ..config import GripperConfig
from ..types import six


@dataclass
class XArm6:
    """Minimal xArm6 adapter that never commands a move during connect()."""

    ip: str
    _arm: XArmAPI | None = None
    _last_gripper_pulse: int | None = None

    def __post_init__(self) -> None:
        if ":" in self.ip:
            raise ValueError(
                "xArm 参数只能填写主机 IP，例如 `192.168.1.100`；不要附加 `:502`。"
            )

    @property
    def connected(self) -> bool:
        return self._arm is not None and bool(self._arm.connected)

    def connect(self) -> None:
        if self._arm is not None:
            raise RuntimeError("xArm is already connected")
        arm = XArmAPI(self.ip, is_radian=True, do_not_open=True)
        arm.connect()
        if not arm.connected:
            raise RuntimeError(f"Cannot connect to xArm at {self.ip}")
        self._arm = arm

    def _require_connection(self) -> XArmAPI:
        if self._arm is None or not self._arm.connected:
            raise RuntimeError("xArm is not connected")
        return self._arm

    @staticmethod
    def _check(code: int, operation: str) -> None:
        if code != 0:
            raise RuntimeError(f"xArm {operation} failed with code {code}")

    def joint_positions(self) -> tuple[float, float, float, float, float, float]:
        arm = self._require_connection()
        code, angles = arm.get_servo_angle(is_radian=True)
        self._check(code, "get_servo_angle")
        return six(angles[:6])

    def preflight(self) -> None:
        arm = self._require_connection()
        code, errors = arm.get_err_warn_code()
        self._check(code, "get_err_warn_code")
        if any(int(value) != 0 for value in errors):
            raise RuntimeError(f"xArm reports error/warning codes: {errors}; clear and investigate in xArm Studio")

    def arm_for_joint_servo(self, gripper: GripperConfig) -> None:
        """Enable only after explicit user confirmation; this method still sends no joint target."""
        arm = self._require_connection()
        self._check(arm.motion_enable(enable=True), "motion_enable")
        self._check(arm.set_mode(1), "set servo motion mode")
        self._check(arm.set_state(0), "set ready state")
        self._check(arm.set_gripper_mode(0), "set gripper position mode")
        self._check(arm.set_gripper_enable(True), "enable standard gripper")
        self._check(arm.set_gripper_speed(gripper.speed), "set gripper speed")

    def arm_for_joint_servo_without_gripper(self) -> None:
        """Enable xArm joint servo mode while leaving the gripper untouched."""
        arm = self._require_connection()
        self._check(arm.motion_enable(enable=True), "motion_enable")
        self._check(arm.set_mode(1), "set servo motion mode")
        self._check(arm.set_state(0), "set ready state")

    def send_joint_target(self, joints_rad: tuple[float, ...]) -> None:
        arm = self._require_connection()
        self._check(arm.set_servo_angle_j(list(six(joints_rad)), is_radian=True), "set_servo_angle_j")

    def send_gripper_open_fraction(self, open_fraction: float, config: GripperConfig) -> None:
        if not 0.0 <= open_fraction <= 1.0:
            raise ValueError("open_fraction must be within [0, 1]")
        target = round(config.closed_pulse + open_fraction * (config.open_pulse - config.closed_pulse))
        if self._last_gripper_pulse is not None and abs(target - self._last_gripper_pulse) < config.minimum_delta_pulse:
            return
        arm = self._require_connection()
        self._check(arm.set_gripper_position(target, speed=config.speed, wait=False), "set_gripper_position")
        self._last_gripper_pulse = target

    def stop(self) -> None:
        if self._arm is not None and self._arm.connected:
            self._check(self._arm.set_state(4), "stop")

    def disconnect(self) -> None:
        if self._arm is not None:
            self._arm.disconnect()
        self._arm = None
        self._last_gripper_pulse = None
