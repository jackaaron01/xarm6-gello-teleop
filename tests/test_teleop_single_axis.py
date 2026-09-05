import sys
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "teleop_single_axis.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = spec_from_file_location("teleop_single_axis", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_single_axis_target_holds_other_five_axes() -> None:
    calibration = MODULE.RelativeCalibration(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3"),
        4096,
        (-1, 1, 1, -1, 1, -1),
        (7.1, 4.5, 12.1, 4.6, 11.5, 5.6),
        (20,) * 6,
        "gripper",
        2438,
        1857,
    )
    zero_raw = {"shoulder_pan": 1000, "shoulder_lift": 2000, "elbow_flex": 300, "wrist_1": 3000, "wrist_2": 2900, "wrist_3": 0, "gripper": 1857}
    target = MODULE.requested_axis_target(
        0, zero_raw, (0, -0.6, -0.7, 0, 1.4, 0), {**zero_raw, "shoulder_pan": 800, "elbow_flex": -200}, calibration
    )
    assert target.joints_rad[0] > 0
    assert target.joints_rad[1:] == (-0.6, -0.7, 0.0, 1.4, 0.0)
    assert target.gripper_open == 0.0


def test_default_servo_rate_and_velocity_are_smooth_but_usable() -> None:
    assert MODULE.DEFAULT_RATE_HZ == 100.0
    assert MODULE.DEFAULT_MAX_DELTA_RAD == 0.004
    assert MODULE.DEFAULT_MAX_VELOCITY_RAD_S == 0.20
    assert MODULE.MAX_ALLOWED_VELOCITY_RAD_S == 0.25


def test_long_send_event_keeps_only_control_data() -> None:
    zero_raw = {name: 0 for name in MODULE.JOINT_NAMES}
    raw = {**zero_raw, "wrist_3": 123}
    requested = MODULE.TeleopTarget((0.0, 0.0, 0.0, 0.0, 0.0, 0.6), 0.0)
    limited = MODULE.TeleopTarget((0.0, 0.0, 0.0, 0.0, 0.0, 0.2), 0.0)

    event = MODULE.long_send_event(7, 1.2345678, "wrist_3", raw, zero_raw, requested, limited, 0.045)

    assert event == {
        "tick": 7,
        "elapsed_s": 1.234568,
        "axis": "wrist_3",
        "leader_raw": 123,
        "leader_raw_delta": 123,
        "requested_joint_rad": 0.6,
        "limited_joint_rad": 0.2,
        "send_ms": 45.0,
    }
