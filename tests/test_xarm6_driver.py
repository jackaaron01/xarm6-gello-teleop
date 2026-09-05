import pytest

from xarm6_gello_teleop.drivers.xarm6 import XArm6


def test_xarm_rejects_ip_with_port() -> None:
    with pytest.raises(ValueError, match="不要附加"):
        XArm6("192.168.1.100:502")


class FakeServoArm:
    connected = True

    def __init__(self) -> None:
        self.mode = 0
        self.calls: list[str] = []

    def motion_enable(self, enable: bool) -> int:
        assert enable is True
        self.calls.append("motion_enable")
        return 0

    def set_mode(self, mode: int) -> int:
        assert mode == 1
        self.mode = mode
        self.calls.append("set_mode")
        return 0

    def set_state(self, state: int) -> int:
        assert state == 0
        self.calls.append("set_state")
        return 0


def test_joint_servo_waits_for_reported_mode_before_streaming() -> None:
    fake = FakeServoArm()
    xarm = XArm6("192.168.1.100", _arm=fake)  # type: ignore[arg-type]

    xarm.arm_for_joint_servo_without_gripper()

    assert fake.calls == ["motion_enable", "set_mode", "set_state"]
    assert fake.mode == 1
