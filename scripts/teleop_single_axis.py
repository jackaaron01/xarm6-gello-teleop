#!/usr/bin/env python3
"""Deliberately constrained first-motion test: one selected xArm axis only."""

from __future__ import annotations

import argparse
import signal
import time
from math import copysign
from pathlib import Path

from xarm6_gello_teleop.config import LeaderConfig, RelativeCalibration, XArmConfig
from xarm6_gello_teleop.drivers.dynamixel_leader import DynamixelLeader
from xarm6_gello_teleop.drivers.xarm6 import XArm6
from xarm6_gello_teleop.safety import JointSafetyLimiter
from xarm6_gello_teleop.types import JOINT_NAMES, TeleopTarget


# Servo mode is streamed at 50 Hz so the controller receives smooth, small
# setpoint updates rather than the visibly stepped 10 Hz commands used first.
DEFAULT_RATE_HZ = 100.0
DEFAULT_MAX_DELTA_RAD = 0.004
DEFAULT_MAX_VELOCITY_RAD_S = 0.20
MAX_ALLOWED_VELOCITY_RAD_S = 0.25


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="仅允许一个 xArm 关节运动的首次遥操作测试")
    parser.add_argument("--axis", choices=JOINT_NAMES, required=True)
    parser.add_argument("--leader", type=Path, required=True)
    parser.add_argument("--xarm", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--xarm-ip", required=True, help="只填 IP，不要加 :502")
    parser.add_argument("--duration-s", type=float, default=10.0, help="最长测试时长，默认 10 秒")
    parser.add_argument(
        "--max-velocity-rad-s",
        type=float,
        default=DEFAULT_MAX_VELOCITY_RAD_S,
        help="单轴最高目标变化率，默认 0.20 rad/s，安全上限为 0.25",
    )
    parser.add_argument(
        "--rate-hz",
        type=float,
        default=DEFAULT_RATE_HZ,
        help="servo 目标流频率，默认 100 Hz，允许范围 20--100 Hz；路由器映射建议先用 40 Hz",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.duration_s <= 0 or not 0 < args.max_velocity_rad_s <= MAX_ALLOWED_VELOCITY_RAD_S:
        raise ValueError("duration-s 必须为正数，max-velocity-rad-s 必须在 (0, 0.25] 内")
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
        DEFAULT_MAX_DELTA_RAD,
        args.max_velocity_rad_s,
    )

    print("危险边界：此命令会使能 xArm，并只向一个关节发送低速 servo 目标。")
    print("其它五轴始终锁定在启动时的位置；夹爪不会被使能或发送命令。")
    print(
        f"测试轴：{args.axis}；{args.rate_hz:.0f} Hz 目标流；最高目标变化率 "
        f"{args.max_velocity_rad_s:.2f} rad/s；最长 {args.duration_s:.0f} 秒。"
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

    def stop_on_signal(signum: int, frame: object) -> None:
        xarm.stop()
        xarm.disconnect()
        leader.disconnect()
        raise SystemExit(f"收到信号 {signum}，已停止。")

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
    while time.monotonic() - started_s < args.duration_s:
        loop_s = time.monotonic()
        read_started_s = time.monotonic()
        raw = leader.read_raw()
        read_elapsed_s = time.monotonic() - read_started_s
        leader_read_total_s += read_elapsed_s
        leader_read_max_s = max(leader_read_max_s, read_elapsed_s)
        requested = requested_axis_target(axis_index, zero_raw, zero_xarm, raw, calibration)
        target = limiter.apply(requested, previous.joints_rad, loop_s - previous_tick_s)
        send_started_s = time.monotonic()
        xarm.send_joint_target(target.joints_rad)
        send_elapsed_s = time.monotonic() - send_started_s
        xarm_send_total_s += send_elapsed_s
        xarm_send_max_s = max(xarm_send_max_s, send_elapsed_s)
        previous = target
        previous_tick_s = loop_s
        tick_count += 1
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
    xarm.stop()
    xarm.disconnect()
    leader.disconnect()
    elapsed_s = time.monotonic() - started_s
    print(
        f"单轴测试时间结束，已停止 xArm；未发送任何夹爪命令。实际 {tick_count / elapsed_s:.1f} Hz，"
        f"超时周期 {missed_deadlines}/{tick_count}。\n"
        f"leader 读取：平均 {1000 * leader_read_total_s / tick_count:.2f} ms，最大 {1000 * leader_read_max_s:.2f} ms；"
        f"xArm servo 发送：平均 {1000 * xarm_send_total_s / tick_count:.2f} ms，最大 {1000 * xarm_send_max_s:.2f} ms。"
    )


if __name__ == "__main__":
    main()
