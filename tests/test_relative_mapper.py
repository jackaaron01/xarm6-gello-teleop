from xarm6_gello_teleop.config import RelativeCalibration
from xarm6_gello_teleop.relative_mapper import RelativeMapper


def test_relative_mapping_and_gripper_clamp() -> None:
    calibration = RelativeCalibration(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3"),
        4096,
        (1, -1, 1, 1, 1, 1),
        (6.283185307,) * 6,
        (20,) * 6,
        "gripper",
        100,
        1100,
    )
    mapper = RelativeMapper(calibration)
    mapper.align(
        {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_1": 0, "wrist_2": 0, "wrist_3": 0, "gripper": 100},
        (0, 0, 0, 0, 0, 0),
    )
    target = mapper.target(
        {"shoulder_pan": 1024, "shoulder_lift": 1024, "elbow_flex": 0, "wrist_1": 0, "wrist_2": 0, "wrist_3": 0, "gripper": 1200}
    )
    assert abs(target.joints_rad[0] - 1.57079632675) < 1e-8
    assert abs(target.joints_rad[1] + 1.57079632675) < 1e-8
    assert target.gripper_open == 1.0


def test_relative_mapping_ignores_small_passive_joint_drift() -> None:
    calibration = RelativeCalibration(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3"),
        4096,
        (1,) * 6,
        (6.283185307,) * 6,
        (20,) * 6,
        "gripper",
        100,
        1100,
    )
    mapper = RelativeMapper(calibration)
    zero = {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_1": 0, "wrist_2": 0, "wrist_3": 0, "gripper": 100}
    mapper.align(zero, (0, 0, 0, 0, 0, 0))
    target = mapper.target({**zero, "shoulder_pan": 20, "wrist_2": -17})
    assert target.joints_rad == (0, 0, 0, 0, 0, 0)
