import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "gello_joint_direction_check.py"
SPEC = importlib.util.spec_from_file_location("gello_joint_direction_check", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_position_delta_report_passes_when_only_target_moves() -> None:
    before = {motor_id: 100 for motor_id in range(1, 8)}
    after = {motor_id: 100 for motor_id in range(1, 8)}
    after[3] = 250

    report = MODULE.position_delta_report(before, after, target_id=3, minimum_delta_raw=80)

    assert report["target_is_dominant"] is True
    assert report["target_delta_raw"] == 150
    assert report["active_ids"] == [3]
    assert report["target_rank"] == 1


def test_position_delta_report_flags_other_movement() -> None:
    before = {motor_id: 100 for motor_id in range(1, 8)}
    after = {motor_id: 100 for motor_id in range(1, 8)}
    after[2] = 0
    after[3] = 250

    report = MODULE.position_delta_report(before, after, target_id=3, minimum_delta_raw=80)

    assert report["target_is_dominant"] is False
    assert report["active_ids"] == [3, 2]
    assert report["dominant_id"] == 3
