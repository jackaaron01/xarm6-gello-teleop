#!/usr/bin/env python3
"""Explicitly selected simultaneous-axis test for the coupled Phase-A leader."""

from __future__ import annotations

import argparse
import signal
import time
from pathlib import Path

from xarm6_gello_teleop.config import LeaderConfig, RelativeCalibration, XArmConfig
from xarm6_gello_teleop.drivers.dynamixel_leader import DynamixelLeader
from xarm6_gello_teleop.drivers.xarm6 import XArm6
from xarm6_gello_teleop.motion_profiles import MOTION_PROFILES, SAFE_PROFILE, motion_profile
from xarm6_gello_teleop.relative_mapper import RelativeMapper
from xarm6_gello_teleop.safety import JointSafetyLimiter
from xarm6_gello_teleop.types import JOINT_NAMES, TeleopTarget


DEFAULT_RATE_HZ = 50.0
DEFAULT_DURATION_S = 10.0


def parse_axes(value: str) -> tuple[str, ...]:
    axes = tuple(item.strip() for item in value.split(",") if item.strip())
    if len(axes) < 2:
        raise ValueError("--axes 至少需要两个关节，例如 shoulder_pan,shoulder_lift")
    if len(set(axes)) != len(axes):
        raise ValueError("--axes 不允许重复关节")
    unknown = set(axes).difference(JOINT_NAMES)
    if unknown:
        raise ValueError(f"--axes 包含未知关节：{sorted(unknown)}")
    return axes


def allowed_axes_target(
    mapper: RelativeMapper,
    zero_joints_rad: tuple[float, float, float, float, float, float],
    raw: dict[str, int],
    allowed_axes: tuple[str, ...],
) -> TeleopTarget:
    """Map only explicitly selected axes; every other xArm axis remains at session zero."""
    predicted = mapper.target(raw)
    joints = list(zero_joints_rad)
    for index, name in enumerate(JOINT_NAMES):
        if name in allowed_axes:
            joints[index] = predicted.joints_rad[index]
    return TeleopTarget(tuple(joints), 0.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="仅允许显式指定轴同时跟随的 Phase-A 多轴测试")
    parser.add_argument("--axes", required=True, help="逗号分隔的同时跟随轴，例如 shoulder_pan,shoulder_lift")
    parser.add_argument("--leader", type=Path, required=True)
    parser.add_argument("--xarm", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--xarm-ip", required=True, help="只填 IP，不要加 :502")
    parser.add_argument("--duration-s", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument(
        "--profile",
        choices=tuple(MOTION_PROFILES),
        default=SAFE_PROFILE.name,
        help="safe=0.004 rad/周期、0.20 rad/s；responsive=0.005 rad/周期、0.25 rad/s",
    )
    parser.add_argument("--max-velocity-rad-s", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    allowed_axes = parse_axes(args.axes)
    profile = motion_profile(args.profile)
    max_velocity_rad_s = profile.max_velocity_rad_s if args.max_velocity_rad_s is None else args.max_velocity_rad_s
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.duration_s <= 0:
        raise ValueError("duration-s 必须为正数")
    if not 20 <= args.rate_hz <= 100:
        raise ValueError("rate-hz 必须在 20--100 Hz 内")
    if not 0 < max_velocity_rad_s <= profile.max_velocity_rad_s:
        raise ValueError("max-velocity-rad-s 必须在所选 profile 的范围内")

    leader = DynamixelLeader(LeaderConfig.from_file(args.leader))
    xarm_config = XArmConfig.from_file(args.xarm)
    calibration = RelativeCalibration.from_file(args.calibration)
    mapper = RelativeMapper(calibration)
    xarm = XArm6(args.xarm_ip)
    limiter = JointSafetyLimiter(
        xarm_config.joint_lower_rad,
        xarm_config.joint_upper_rad,
        profile.max_delta_rad,
        max_velocity_rad_s,
    )

    selected_text = ", ".join(allowed_axes)
    locked_axes = tuple(name for name in JOINT_NAMES if name not in allowed_axes)
    print("危险边界：此命令会使能 xArm，但仅让 --axes 中明确列出的轴同时跟随。")
    print(f"本次同时跟随轴：{selected_text}。锁定轴：{', '.join(locked_axes) or '无'}。")
    print("夹爪不会被使能或发送命令；Ctrl-C 可立即停止。")
    print(
        f"{profile.name} 档；{args.rate_hz:.0f} Hz；每周期最多 {profile.max_delta_rad:.3f} rad；"
        f"最高目标变化率 {max_velocity_rad_s:.2f} rad/s；最长 {args.duration_s:.0f} 秒。"
    )
    print("请清空工作区，确认 xArm reduced mode 与实体急停均已就绪。")
    input("确认现场安全后按 Enter，连接并执行只读预检：")
    leader.connect()
    xarm.connect()
    xarm.preflight()
    input("让 xArm 与 leader 处于安全起始姿态；按 Enter 记录零点并开始：")
    zero_raw = leader.read_raw()
    zero_xarm = xarm.joint_positions()
    mapper.align(zero_raw, zero_xarm)
    previous = TeleopTarget(zero_xarm, 0.0)
    print("零点已记录；正在确认 xArm servo mode。")
    xarm.arm_for_joint_servo_without_gripper()
    xarm.send_joint_target(previous.joints_rad)
    print("多轴测试已开始；只同时移动终端显示的指定 leader 轴，Ctrl-C 可立即停止。")

    stop_requested = False

    def stop_on_signal(signum: int, frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True
        print(f"\n收到停止请求 {signum}：不再发送新目标，正在执行受控停止。")

    signal.signal(signal.SIGINT, stop_on_signal)
    signal.signal(signal.SIGTERM, stop_on_signal)
    period_s = 1.0 / args.rate_hz
    started_s = time.monotonic()
    next_deadline_s = started_s
    previous_tick_s = started_s
    tick_count = 0
    missed_deadlines = 0
    leader_read_total_s = 0.0
    xarm_send_total_s = 0.0
    leader_read_max_s = 0.0
    xarm_send_max_s = 0.0
    while not stop_requested and time.monotonic() - started_s < args.duration_s:
        loop_s = time.monotonic()
        read_started_s = time.monotonic()
        raw = leader.read_raw()
        read_elapsed_s = time.monotonic() - read_started_s
        leader_read_total_s += read_elapsed_s
        leader_read_max_s = max(leader_read_max_s, read_elapsed_s)
        if stop_requested:
            break
        requested = allowed_axes_target(mapper, zero_xarm, raw, allowed_axes)
        target = limiter.apply(requested, previous.joints_rad, loop_s - previous_tick_s)
        send_started_s = time.monotonic()
        xarm.send_joint_target(target.joints_rad)
        send_elapsed_s = time.monotonic() - send_started_s
        xarm_send_total_s += send_elapsed_s
        xarm_send_max_s = max(xarm_send_max_s, send_elapsed_s)
        previous = target
        previous_tick_s = loop_s
        tick_count += 1
        if stop_requested:
            break
        next_deadline_s += period_s
        remaining_s = next_deadline_s - time.monotonic()
        if remaining_s > 0:
            time.sleep(remaining_s)
        else:
            missed_deadlines += 1
            next_deadline_s = time.monotonic() + period_s
            time.sleep(period_s)
    if stop_requested:
        print("停止请求已处理：正在向 xArm 发送停止状态。")
    xarm.stop()
    xarm.disconnect()
    leader.disconnect()
    elapsed_s = time.monotonic() - started_s
    if tick_count == 0:
        print("多轴测试在发送首条流式目标前结束；未发送夹爪命令。")
        return
    print(
        f"多轴测试结束，已停止 xArm；未发送夹爪命令。实际 {tick_count / elapsed_s:.1f} Hz，"
        f"超时周期 {missed_deadlines}/{tick_count}。\n"
        f"leader 读取：平均 {1000 * leader_read_total_s / tick_count:.2f} ms，最大 {1000 * leader_read_max_s:.2f} ms；"
        f"xArm servo 发送：平均 {1000 * xarm_send_total_s / tick_count:.2f} ms，最大 {1000 * xarm_send_max_s:.2f} ms。"
    )


if __name__ == "__main__":
    main()
