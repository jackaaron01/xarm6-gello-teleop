import sys
from importlib.util import module_from_spec, spec_from_file_location
from math import isclose, radians
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "teleop_multi_axis_limited.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = spec_from_file_location("teleop_multi_axis_limited", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_parse_axes_requires_two_unique_known_axes() -> None:
    assert MODULE.parse_axes("shoulder_pan,wrist_3") == ("shoulder_pan", "wrist_3")


def test_parse_return_pose_uses_six_degree_values() -> None:
    assert MODULE.parse_six_joint_degrees("0,-35,-40,0,80,0") == (
        0.0,
        radians(-35),
        radians(-40),
        0.0,
        radians(80),
        0.0,
    )


def test_return_target_step_is_bounded_and_converges_to_p0() -> None:
    limiter = MODULE.JointSafetyLimiter((-3.0,) * 6, (3.0,) * 6, 0.003, 0.15)
    start = MODULE.TeleopTarget((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 0.0)
    p0 = MODULE.parse_six_joint_degrees("0,-35,-40,0,80,0")
    first = MODULE.return_target_step(limiter, start, p0, 0.02)
    assert first.joints_rad == (0.0, -0.003, -0.003, 0.0, 0.003, 0.0)

    target = start
    for _ in range(600):
        target = MODULE.return_target_step(limiter, target, p0, 0.02)
    assert all(isclose(actual, expected) for actual, expected in zip(target.joints_rad, p0, strict=True))


def test_allowed_axes_target_holds_unselected_axes() -> None:
    calibration = MODULE.RelativeCalibration(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3"),
        4096,
        (1,) * 6,
        (6.283185307,) * 6,
        (20,) * 6,
        "gripper",
        2438,
        1857,
    )
    mapper = MODULE.RelativeMapper(calibration)
    zero_raw = {name: 0 for name in MODULE.JOINT_NAMES} | {"gripper": 1857}
    zero_joints = (0.0, -0.6, -0.7, 0.0, 1.4, 0.0)
    mapper.align(zero_raw, zero_joints)

    target = MODULE.allowed_axes_target(
        mapper,
        zero_joints,
        {**zero_raw, "shoulder_pan": 200, "shoulder_lift": -300, "wrist_3": 400},
        ("shoulder_pan", "shoulder_lift"),
    )

    assert target.joints_rad[0] > 0.0
    assert target.joints_rad[1] < -0.6
    assert target.joints_rad[2:] == (-0.7, 0.0, 1.4, 0.0)
