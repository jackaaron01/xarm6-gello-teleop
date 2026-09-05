from xarm6_gello_teleop.config import RelativeCalibration
from xarm6_gello_teleop.phase_a_gate import DominantAxisGate


def calibration() -> RelativeCalibration:
    return RelativeCalibration(
        ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3"),
        4096,
        (1,) * 6,
        (6.283185307,) * 6,
        (20,) * 6,
        "gripper",
        2438,
        1857,
    )


def raw(**changes: int) -> dict[str, int]:
    result = {
        "shoulder_pan": 0,
        "shoulder_lift": 0,
        "elbow_flex": 0,
        "wrist_1": 0,
        "wrist_2": 0,
        "wrist_3": 0,
        "gripper": 1857,
    }
    result.update(changes)
    return result


def test_gate_selects_largest_axis_and_holds_the_others() -> None:
    gate = DominantAxisGate(calibration(), raw(), (0.0,) * 6, 0.35)

    update = gate.update(raw(shoulder_lift=90, wrist_1=25), 0.0)

    assert update.active_index == 1
    assert update.activated_index == 1
    assert update.requested.joints_rad[0] == 0.0
    assert update.requested.joints_rad[2:] == (0.0, 0.0, 0.0, 0.0)
    assert update.requested.joints_rad[1] > 0.0


def test_gate_latches_one_axis_before_selecting_the_next() -> None:
    gate = DominantAxisGate(calibration(), raw(), (0.0,) * 6, 0.35)
    first = gate.update(raw(shoulder_pan=100), 0.0)
    held = gate.update(raw(shoulder_pan=100), 0.1)
    released = gate.update(raw(shoulder_pan=100), 0.5)
    gate.latch_limited_joints(released.requested.joints_rad)

    second = gate.update(raw(shoulder_pan=100, wrist_2=-100), 0.6)

    assert first.active_index == 0
    assert held.released_index is None
    assert released.released_index == 0
    assert second.active_index == 4
    assert second.requested.joints_rad[0] == released.requested.joints_rad[0]
    assert second.requested.joints_rad[4] < 0.0
