from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from .config import XArmConfig
from .drivers.dynamixel_leader import DynamixelLeader
from .drivers.xarm6 import XArm6
from .relative_mapper import RelativeMapper
from .safety import JointSafetyLimiter
from .types import TeleopTarget


class RuntimeState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    PREFLIGHT = "preflight"
    ARMED = "armed"
    ACTIVE = "active"
    FAULT = "fault"
    STOPPED = "stopped"


@dataclass
class TeleopRuntime:
    leader: DynamixelLeader
    xarm: XArm6
    mapper: RelativeMapper
    config: XArmConfig
    state: RuntimeState = RuntimeState.DISCONNECTED
    _previous_target: TeleopTarget | None = None
    _previous_tick_s: float | None = None

    def __post_init__(self) -> None:
        self._limiter = JointSafetyLimiter(
            self.config.joint_lower_rad, self.config.joint_upper_rad, self.config.max_delta_rad, self.config.max_velocity_rad_s
        )

    def connect(self) -> None:
        if self.state is not RuntimeState.DISCONNECTED:
            raise RuntimeError(f"Cannot connect from {self.state}")
        self.leader.connect()
        self.xarm.connect()
        self.state = RuntimeState.CONNECTED

    def align_session_zero(self) -> None:
        if self.state is not RuntimeState.CONNECTED:
            raise RuntimeError(f"Align requires connected state, got {self.state}")
        self.mapper.align(self.leader.read_raw(), self.xarm.joint_positions())
        zero_target = self.mapper.target(self.leader.read_raw())
        self._previous_target = zero_target
        self.state = RuntimeState.PREFLIGHT

    def preflight(self) -> None:
        if self.state is not RuntimeState.PREFLIGHT:
            raise RuntimeError(f"Preflight requires session alignment, got {self.state}")
        self.xarm.preflight()

    def arm(self) -> None:
        if self.state is not RuntimeState.PREFLIGHT:
            raise RuntimeError(f"Arm requires successful preflight, got {self.state}")
        self.xarm.arm_for_joint_servo(self.config.gripper)
        self.state = RuntimeState.ARMED

    def start(self) -> None:
        if self.state is not RuntimeState.ARMED:
            raise RuntimeError(f"Start requires armed state, got {self.state}")
        self._previous_tick_s = time.monotonic()
        self.state = RuntimeState.ACTIVE

    def tick(self) -> TeleopTarget:
        if self.state is not RuntimeState.ACTIVE or self._previous_target is None or self._previous_tick_s is None:
            raise RuntimeError(f"Tick requires active state, got {self.state}")
        started_s = time.monotonic()
        raw = self.leader.read_raw()
        read_duration_s = time.monotonic() - started_s
        if read_duration_s > self.config.leader_timeout_s:
            self.fault(f"Leader read exceeded timeout: {read_duration_s:.3f}s")
        requested = self.mapper.target(raw)
        now_s = time.monotonic()
        target = self._limiter.apply(requested, self._previous_target.joints_rad, now_s - self._previous_tick_s)
        self.xarm.send_joint_target(target.joints_rad)
        self.xarm.send_gripper_open_fraction(target.gripper_open, self.config.gripper)
        self._previous_target = target
        self._previous_tick_s = now_s
        return target

    def fault(self, reason: str) -> None:
        self.state = RuntimeState.FAULT
        self.xarm.stop()
        raise RuntimeError(reason)

    def stop(self) -> None:
        if self.state is RuntimeState.ACTIVE:
            self.xarm.stop()
        self.state = RuntimeState.STOPPED

    def disconnect(self) -> None:
        self.stop()
        self.xarm.disconnect()
        self.leader.disconnect()
        self.state = RuntimeState.DISCONNECTED
