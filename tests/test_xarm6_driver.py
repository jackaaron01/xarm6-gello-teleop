import pytest

from xarm6_gello_teleop.drivers.xarm6 import XArm6


def test_xarm_rejects_ip_with_port() -> None:
    with pytest.raises(ValueError, match="不要附加"):
        XArm6("192.168.1.100:502")
