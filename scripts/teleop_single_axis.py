#!/usr/bin/env python3
"""Deliberately constrained first-motion test: one selected xArm axis only."""

from __future__ import annotations

import argparse
import json
import signal
import time
from math import copysign
from pathlib import Path

from xarm6_gello_teleop.config import LeaderConfig, RelativeCalibration, XArmConfig
from xarm6_gello_teleop.drivers.dynamixel_leader import DynamixelLeader
from xarm6_gello_teleop.drivers.xarm6 import XArm6
from xarm6_gello_teleop.motion_profiles import (
    MOTION_PROFILES,
    RESPONSIVE_PROFILE,
    SAFE_PROFILE,
    motion_profile,
)
from xarm6_gello_teleop.safety import JointSafetyLimiter
from xarm6_gello_teleop.types import JOINT_NAMES, TeleopTarget


# Servo mode is streamed at 50 Hz so the controller receives smooth, small
# setpoint updates rather than the visibly stepped 10 Hz commands used first.
DEFAULT_RATE_HZ = 100.0
DEFAULT_MAX_DELTA_RAD = SAFE_PROFILE.max_delta_rad
DEFAULT_MAX_VELOCITY_RAD_S = SAFE_PROFILE.max_velocity_rad_s
MAX_ALLOWED_VELOCITY_RAD_S = RESPONSIVE_PROFILE.max_velocity_rad_s


def requested_axis_target(
    axis_index: int,
    zero_raw: dict[str, int],
    zero_xarm: tuple[float, float, float, float, float, float],
    raw: dict[str, int],
    calibration: RelativeCalibration,
) -> TeleopTarget:
    name = JOINT_NAMES[axis_index]
    raw_delta = float(raw[name] - zero_raw[name])
    deadband = calibration.joint_deadband_raw[axis_index]
    if abs(raw_delta) <= deadband:
        raw_delta = 0.0
    else:
        raw_delta = copysign(abs(raw_delta) - deadband, raw_delta)
    joint_delta = (
        raw_delta
        / calibration.counts_per_turn
        * calibration.sign[axis_index]
        * calibration.gain_rad_per_turn[axis_index]
    )
    joints = list(zero_xarm)
    joints[axis_index] += joint_delta
    return TeleopTarget(tuple(joints), 0.0)


def long_send_event(
    tick: int,
    elapsed_s: float,
    axis: str,
    raw: dict[str, int],
    zero_raw: dict[str, int],
    requested: TeleopTarget,
    limited: TeleopTarget,
    send_elapsed_s: float,
) -> dict[str, float | int | str]:
    axis_index = JOINT_NAMES.index(axis)
    return {
        "tick": tick,
        "elapsed_s": round(elapsed_s, 6),
        "axis": axis,
        "leader_raw": int(raw[axis]),
        "leader_raw_delta": int(raw[axis] - zero_raw[axis]),
        "requested_joint_rad": round(requested.joints_rad[axis_index], 6),
        "limited_joint_rad": round(limited.joints_rad[axis_index], 6),
        "send_ms": round(1000 * send_elapsed_s, 3),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="仅允许一个 xArm 关节运动的首次遥操作测试")
    parser.add_argument("--axis", choices=JOINT_NAMES, required=True)
    parser.add_argument("--leader", type=Path, required=True)
    parser.add_argument("--xarm", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--xarm-ip", required=True, help="只填 IP，不要加 :502")
    parser.add_argument("--duration-s", type=float, default=10.0, help="最长测试时长，默认 10 秒")
    parser.add_argument(
        "--profile",
        choices=tuple(MOTION_PROFILES),
        default=SAFE_PROFILE.name,
        help="safe=0.004 rad/周期、0.20 rad/s；responsive=0.005 rad/周期、0.25 rad/s",
    )
    parser.add_argument(
        "--max-velocity-rad-s",
        type=float,
        default=None,
        help="覆盖所选 profile 的最高目标变化率；不得高于该 profile 的上限",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="servo 目标流频率，默认 100 Hz，允许范围 20--100 Hz；路由器映射建议先用 40 Hz",
    )
    parser.add_argument(
        "--diagnostics-output",
        type=Path,
        help="可选：保存超过阈值的 xArm 发送周期 JSON；不会额外读取或控制硬件",
    )
    parser.add_argument(
        "--long-send-threshold-ms",
        type=float,
        default=30.0,
        help="记录 xArm 发送长尾的阈值，默认 30 ms",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    profile = motion_profile(args.profile)
    max_velocity_rad_s = profile.max_velocity_rad_s if args.max_velocity_rad_s is None else args.max_velocity_rad_s
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.duration_s <= 0 or args.long_send_threshold_ms <= 0:
        raise ValueError("duration-s 和 long-send-threshold-ms 必须为正数")
    if not 0 < max_velocity_rad_s <= profile.max_velocity_rad_s:
        raise ValueError("max-velocity-rad-s 必须在所选 profile 的范围内")
    if not 20 <= args.rate_hz <= 100:
        raise ValueError("rate-hz 必须在 20--100 Hz 内")
    axis_index = JOINT_NAMES.index(args.axis)
    leader = DynamixelLeader(LeaderConfig.from_file(args.leader))
    xarm_config = XArmConfig.from_file(args.xarm)
    calibration = RelativeCalibration.from_file(args.calibration)
    xarm = XArm6(args.xarm_ip)
    limiter = JointSafetyLimiter(
        xarm_config.joint_lower_rad,
        xarm_config.joint_upper_rad,
        profile.max_delta_rad,
        max_velocity_rad_s,
    )

    print("危险边界：此命令会使能 xArm，并只向一个关节发送低速 servo 目标。")
    print("其它五轴始终锁定在启动时的位置；夹爪不会被使能或发送命令。")
    print(
        f"测试轴：{args.axis}；{profile.name} 档；{args.rate_hz:.0f} Hz 目标流；每周期最多 "
        f"{profile.max_delta_rad:.3f} rad；最高目标变化率 {max_velocity_rad_s:.2f} rad/s；"
        f"最长 {args.duration_s:.0f} 秒。"
    )
    print("请清空工作区，确认 xArm reduced mode 与实体急停均已就绪。")
    input("确认现场安全后按 Enter，连接并执行只读预检：")
    leader.connect()
    xarm.connect()
    xarm.preflight()
    input("让 leader 与 xArm 保持同一安全起始姿态；按 Enter 记录零点并开始单轴测试：")
    zero_raw = leader.read_raw()
    zero_xarm = xarm.joint_positions()
    previous = TeleopTarget(zero_xarm, 0.0)
    print("零点已记录；现在使能 xArm servo mode，但先发送当前姿态作为保持目标。")
    xarm.arm_for_joint_servo_without_gripper()
    xarm.send_joint_target(previous.joints_rad)
    print("单轴低速测试已开始；只缓慢移动 leader 的测试轴，Ctrl-C 可立即停止。")

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
    long_send_events: list[dict[str, float | int | str]] = []
    while not stop_requested and time.monotonic() - started_s < args.duration_s:
        loop_s = time.monotonic()
        read_started_s = time.monotonic()
        raw = leader.read_raw()
        read_elapsed_s = time.monotonic() - read_started_s
        leader_read_total_s += read_elapsed_s
        leader_read_max_s = max(leader_read_max_s, read_elapsed_s)
        if stop_requested:
            break
        requested = requested_axis_target(axis_index, zero_raw, zero_xarm, raw, calibration)
        target = limiter.apply(requested, previous.joints_rad, loop_s - previous_tick_s)
        send_started_s = time.monotonic()
        xarm.send_joint_target(target.joints_rad)
        send_elapsed_s = time.monotonic() - send_started_s
        xarm_send_total_s += send_elapsed_s
        xarm_send_max_s = max(xarm_send_max_s, send_elapsed_s)
        if send_elapsed_s * 1000 >= args.long_send_threshold_ms:
            long_send_events.append(
                long_send_event(
                    tick_count + 1,
                    time.monotonic() - started_s,
                    args.axis,
                    raw,
                    zero_raw,
                    requested,
                    target,
                    send_elapsed_s,
                )
            )
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
            # Never burst stale servo targets after a slow controller reply.
            # Start a fresh period instead, then read the newest leader state.
            next_deadline_s = time.monotonic() + period_s
            time.sleep(period_s)
    if stop_requested:
        print("停止请求已处理：正在向 xArm 发送停止状态。")
    xarm.stop()
    xarm.disconnect()
    leader.disconnect()
    elapsed_s = time.monotonic() - started_s
    if tick_count == 0:
        print("单轴测试在发送首条流式目标前结束；未发送夹爪命令。")
        return
    if args.diagnostics_output is not None:
        diagnostics = {
            "axis": args.axis,
            "profile": profile.name,
            "rate_hz": args.rate_hz,
            "max_delta_rad": profile.max_delta_rad,
            "max_velocity_rad_s": max_velocity_rad_s,
            "long_send_threshold_ms": args.long_send_threshold_ms,
            "actual_rate_hz": tick_count / elapsed_s,
            "missed_deadlines": missed_deadlines,
            "tick_count": tick_count,
            "leader_read_average_ms": 1000 * leader_read_total_s / tick_count,
            "leader_read_max_ms": 1000 * leader_read_max_s,
            "xarm_send_average_ms": 1000 * xarm_send_total_s / tick_count,
            "xarm_send_max_ms": 1000 * xarm_send_max_s,
            "long_send_events": long_send_events,
        }
        args.diagnostics_output.parent.mkdir(parents=True, exist_ok=True)
        args.diagnostics_output.write_text(json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8")
        print(f"长尾诊断已保存：{args.diagnostics_output}（事件数 {len(long_send_events)}）")
    print(
        f"单轴测试时间结束，已停止 xArm；未发送任何夹爪命令。实际 {tick_count / elapsed_s:.1f} Hz，"
        f"超时周期 {missed_deadlines}/{tick_count}。\n"
        f"leader 读取：平均 {1000 * leader_read_total_s / tick_count:.2f} ms，最大 {1000 * leader_read_max_s:.2f} ms；"
        f"xArm servo 发送：平均 {1000 * xarm_send_total_s / tick_count:.2f} ms，最大 {1000 * xarm_send_max_s:.2f} ms。"
    )


if __name__ == "__main__":
    main()
