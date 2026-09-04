#!/usr/bin/env python3
"""Fit a candidate relative calibration from P0 plus six single-joint pose pairs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import median
from typing import Any

import yaml


JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_1", "wrist_2", "wrist_3")


def load_pose_pair(path: Path) -> dict[str, Any]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema") != "xarm6-gello-pose-pair/v1":
        raise ValueError(f"不是有效姿态对文件：{path}")
    if int(data.get("counts_per_turn", 0)) != 4096:
        raise ValueError(f"姿态对 counts_per_turn 不为 4096：{path}")
    if tuple(data.get("leader_raw", {}).keys()) != (*JOINT_NAMES, "gripper"):
        raise ValueError(f"姿态对 leader_raw 字段不完整或顺序错误：{path}")
    if len(data.get("xarm_joints_rad", [])) != 6:
        raise ValueError(f"姿态对 xarm_joints_rad 必须包含 6 轴：{path}")
    return data


def parse_joint_pairs(values: list[str]) -> dict[str, list[Path]]:
    pairs: dict[str, list[Path]] = {name: [] for name in JOINT_NAMES}
    for value in values:
        if "=" not in value:
            raise ValueError(f"--joint-pair 格式应为 关节名=文件路径，实际为：{value}")
        name, raw_path = value.split("=", 1)
        if name not in JOINT_NAMES:
            raise ValueError(f"未知关节名：{name}")
        pairs[name].append(Path(raw_path))
    missing = [name for name, paths in pairs.items() if not paths]
    if missing:
        raise ValueError(f"每个关节至少提供一次姿态对，缺少：{missing}")
    return pairs


def fit_joint(p0: dict[str, Any], pair: dict[str, Any], index: int, name: str) -> tuple[float, float, dict[str, float]]:
    raw_delta = float(pair["leader_raw"][name]) - float(p0["leader_raw"][name])
    robot_delta = float(pair["xarm_joints_rad"][index]) - float(p0["xarm_joints_rad"][index])
    if abs(raw_delta) < 80:
        raise ValueError(f"{name} 的 leader 变化仅 {raw_delta:+.0f} counts，小于 80，姿态差异不足")
    if abs(robot_delta) < 0.0873:
        raise ValueError(f"{name} 的 xArm 变化仅 {robot_delta:+.4f} rad，小于 5°，姿态差异不足")
    sign = 1.0 if raw_delta * robot_delta > 0 else -1.0
    gain = abs(robot_delta / raw_delta) * 4096.0
    cross_axis_max = max(
        abs(float(pair["xarm_joints_rad"][other]) - float(p0["xarm_joints_rad"][other]))
        for other in range(6)
        if other != index
    )
    return sign, gain, {
        "leader_delta_raw": raw_delta,
        "xarm_delta_rad": robot_delta,
        "other_xarm_axis_max_delta_rad": cross_axis_max,
    }


def fit_calibration(
    p0: dict[str, Any], joint_pairs: dict[str, list[dict[str, Any]]], gripper: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    signs = []
    gains = []
    diagnostics: dict[str, Any] = {}
    for index, name in enumerate(JOINT_NAMES):
        samples = [fit_joint(p0, pair, index, name) for pair in joint_pairs[name]]
        sample_signs = {sample[0] for sample in samples}
        if len(sample_signs) != 1:
            raise ValueError(f"{name} 的多组姿态对得出了相互矛盾的方向：{sample_signs}")
        signs.append(sample_signs.pop())
        gains.append(float(median(sample[1] for sample in samples)))
        diagnostics[name] = {"samples": [sample[2] for sample in samples]}
    candidate = {
        "joint_names": list(JOINT_NAMES),
        "counts_per_turn": 4096,
        "sign": signs,
        "gain_rad_per_turn": gains,
        # Passive leader links can introduce roughly 16--17 counts of incidental
        # movement in untouched wrist axes.  Twenty counts suppresses this noise.
        "joint_deadband_raw": [20.0] * 6,
        "gripper": {
            "motor_name": str(gripper["leader"]["motor_name"]),
            "raw_closed": int(gripper["leader"]["raw_closed"]),
            "raw_open": int(gripper["leader"]["raw_open"]),
        },
    }
    return candidate, diagnostics


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="由 P0 与六个单关节姿态对生成候选相对标定")
    parser.add_argument("--p0", type=Path, required=True)
    parser.add_argument(
        "--joint-pair",
        action="append",
        required=True,
        help="重复六次：shoulder_pan=results/pose_pairs/shoulder_pan.json",
    )
    parser.add_argument("--gripper-calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    p0 = load_pose_pair(args.p0)
    paths = parse_joint_pairs(args.joint_pair)
    pairs = {name: [load_pose_pair(path) for path in joint_paths] for name, joint_paths in paths.items()}
    gripper = json.loads(args.gripper_calibration.read_text(encoding="utf-8"))
    candidate, diagnostics = fit_calibration(p0, pairs, gripper)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print("候选标定已保存：", args.output)
    print(yaml.safe_dump(candidate, allow_unicode=True, sort_keys=False))
    for name, detail in diagnostics.items():
        for index, sample in enumerate(detail["samples"], start=1):
            print(
                f"{name}[{index}]: leader={sample['leader_delta_raw']:+.0f} counts, "
                f"xArm={sample['xarm_delta_rad']:+.4f} rad, "
                f"其它 xArm 轴最大变化={sample['other_xarm_axis_max_delta_rad']:.4f} rad"
            )
    print("这是候选文件，下一步必须先做只读映射预览；不要直接运行 teleop。")


if __name__ == "__main__":
    main()
