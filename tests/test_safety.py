from xarm6_gello_teleop.safety import JointSafetyLimiter
from xarm6_gello_teleop.types import TeleopTarget


def test_limiter_applies_range_and_velocity() -> None:
    limiter = JointSafetyLimiter((-1,) * 6, (1,) * 6, max_delta_rad=0.2, max_velocity_rad_s=0.5)
    result = limiter.apply(TeleopTarget((3,) * 6, 0.4), (0,) * 6, dt_s=0.1)
    assert result.joints_rad == (0.05,) * 6
    assert result.gripper_open == 0.4
