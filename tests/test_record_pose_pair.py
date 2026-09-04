import sys
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "record_pose_pair.py"
sys.path.insert(0, str(SCRIPT.parent))
SPEC = spec_from_file_location("record_pose_pair", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_validate_target_pose_rejects_unmoved_target_axis() -> None:
    reference = {
        "schema": "xarm6-gello-pose-pair/v1",
        "leader_raw": {"shoulder_pan": 0, "shoulder_lift": 0, "elbow_flex": 144, "wrist_1": 0, "wrist_2": 0, "wrist_3": 0, "gripper": 1857},
        "xarm_joints_rad": [0.0] * 6,
    }
    with pytest.raises(RuntimeError, match="xArm 变化仅"):
        MODULE.validate_target_pose(
            reference,
            {**reference["leader_raw"], "elbow_flex": 668},
            (0, 0, 0, 0, 0, 0),
            "elbow_flex",
            250,
            0.1745,
            0.0873,
        )
