#!/usr/bin/env python3
"""Preview a calibrated leader motion without sending any xArm command."""

from __future__ import annotations

import argparse
from math import degrees
from pathlib import Path

from record_pose_pair import open_leader, read_leader, read_xarm

from xarm6_gello_teleop.config import RelativeCalibration, XArmConfig
from xarm6_gello_teleop.relative_mapper import RelativeMapper
from xarm6_gello_teleop.types import JOINT_NAMES


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="只读预览 leader 动作映射，绝不发送 xArm 运动命令")
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--xarm-ip", required=True, help="只填 IP，不要加 :502")
    parser.add_argument("--xarm", type=Path, required=True, help="xArm 硬件 YAML")
    parser.add_argument("--calibration", type=Path, required=True, help="候选或已验证的相对标定 YAML")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--samples", type=int, default=5)
    return parser.parse_args()


def dominant_joint(
    zero_raw: dict[str, int], moved_raw: dict[str, int], calibration: RelativeCalibration
) -> tuple[int | None, tuple[int, int, int, int, int, int]]:
    """Return the only joint permitted to move during the Phase-A preview."""
    deltas = tuple(int(moved_raw[name] - zero_raw[name]) for name in JOINT_NAMES)
    magnitudes = tuple(abs(delta) for delta in deltas)
    index = max(range(6), key=lambda candidate: magnitudes[candidate])
    if magnitudes[index] <= calibration.joint_deadband_raw[index]:
        return None, deltas
    return index, deltas


def main() -> None:
    args = parse_args()
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.samples <= 0:
        raise ValueError("samples 必须为正数")
    calibration = RelativeCalibration.from_file(args.calibration)
    xarm_config = XArmConfig.from_file(args.xarm)
    mapper = RelativeMapper(calibration)

    print("只读预览：不写 Dynamixel，不使能 xArm/夹爪，不发送任何 xArm 运动目标。")
    input("请让 xArm 与被动 leader 处于同一安全起始姿态；确认后按 Enter 记录本次 session zero。")
    port, packet, reader = open_leader(args.leader_port, args.baudrate)
    zero_raw = read_leader(reader, packet, args.samples)
    zero_xarm = read_xarm(args.xarm_ip)
    mapper.align(zero_raw, zero_xarm)
    print("已记录 session zero。现在只手动移动 leader 的一个关节约 5--15°；不要移动 xArm。")
    input("保持 leader 在新姿态后按 Enter，显示预测目标（仍不会命令机器人）：")
    moved_raw = read_leader(reader, packet, args.samples)
    port.closePort()
    predicted = mapper.target(moved_raw)
    actual_xarm = read_xarm(args.xarm_ip)
    active_index, raw_deltas = dominant_joint(zero_raw, moved_raw, calibration)
    gated_joints = list(zero_xarm)
    if active_index is not None:
        gated_joints[active_index] = predicted.joints_rad[active_index]

    if active_index is None:
        print("\n没有任何关节超过死区；本次门控不会产生 xArm 目标变化。")
    else:
        print(
            f"\nPhase-A 主导关节门控：{JOINT_NAMES[active_index]}（raw 变化 "
            f"{raw_deltas[active_index]:+d} counts）。其它关节的被动随动被保持在 session zero。"
        )
    print("\n关节           raw变化   xArm起点°   门控目标°   门控变化°   当前实际°   实际变化°   在范围内")
    for index, name in enumerate(JOINT_NAMES):
        start = degrees(zero_xarm[index])
        target = degrees(gated_joints[index])
        actual = degrees(actual_xarm[index])
        lower = xarm_config.joint_lower_rad[index]
        upper = xarm_config.joint_upper_rad[index]
        in_range = lower <= gated_joints[index] <= upper
        print(
            f"{name:14} {raw_deltas[index]:+8d} {start:+9.2f} {target:+10.2f} {target - start:+10.2f} "
            f"{actual:+10.2f} {actual - start:+10.2f} {str(in_range):>10}"
        )
    print(f"\n预测夹爪张开比例：{predicted.gripper_open:.3f}（0=闭合，1=张开）")
    print("预览完成：上述门控目标仅打印，没有被发送给 xArm。")


if __name__ == "__main__":
    main()
