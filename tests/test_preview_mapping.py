from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys


SCRIPT = Path(__file__).parents[1] / "scripts" / "preview_mapping.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = spec_from_file_location("preview_mapping", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_dominant_joint_selects_only_the_largest_motion() -> None:
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
    zero = {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 0, "wrist_1": 0, "wrist_2": 0, "wrist_3": 0, "gripper": 1857}
    moved = {**zero, "shoulder_pan": -192, "elbow_flex": -31}
    index, deltas = MODULE.dominant_joint(zero, moved, calibration)
    assert index == 0
    assert deltas == (-192, 0, -31, 0, 0, 0)
