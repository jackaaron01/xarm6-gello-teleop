from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "calibrate_gripper.py"
SPEC = spec_from_file_location("calibrate_gripper", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_derive_calibration_when_zero_is_closed() -> None:
    result = MODULE.derive_calibration(1200, 300, 0, "CLOSED", 850, "OPEN")
    assert result["leader"] == {
        "motor_name": "gripper",
        "motor_id": 7,
        "raw_open": 1200,
        "raw_closed": 300,
    }
    assert result["xarm_standard_gripper"] == {"open_pulse": 850, "closed_pulse": 0}


def test_derive_calibration_when_zero_is_open() -> None:
    result = MODULE.derive_calibration(300, 1200, 0, "OPEN", 850, "CLOSED")
    assert result["xarm_standard_gripper"] == {"open_pulse": 0, "closed_pulse": 850}
