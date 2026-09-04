from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "fit_relative_calibration.py"
SPEC = spec_from_file_location("fit_relative_calibration", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def pose(raw: int, robot: float) -> dict[str, object]:
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3")
    return {
        "schema": "xarm6-gello-pose-pair/v1",
        "counts_per_turn": 4096,
        "leader_raw": {**{name: 0 for name in names}, "gripper": 1857},
        "xarm_joints_rad": [0.0] * 6,
    }


def test_fit_joint_uses_pose_delta_sign_and_scale() -> None:
    p0 = pose(0, 0.0)
    pair = pose(0, 0.0)
    pair["leader_raw"]["shoulder_pan"] = 512
    pair["xarm_joints_rad"][0] = -0.8
    sign, gain, detail = MODULE.fit_joint(p0, pair, 0, "shoulder_pan")
    assert sign == -1.0
    assert gain == 6.4
    assert detail["other_xarm_axis_max_delta_rad"] == 0.0
