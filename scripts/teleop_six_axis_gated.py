#!/usr/bin/env python3
"""Phase-A sequential six-axis teleoperation for a coupled passive leader."""

from __future__ import annotations

import argparse
import signal
import time
from pathlib import Path

from xarm6_gello_teleop.config import LeaderConfig, RelativeCalibration, XArmConfig
from xarm6_gello_teleop.drivers.dynamixel_leader import DynamixelLeader
from xarm6_gello_teleop.drivers.xarm6 import XArm6
from xarm6_gello_teleop.phase_a_gate import DominantAxisGate
from xarm6_gello_teleop.safety import JointSafetyLimiter
from xarm6_gello_teleop.types import JOINT_NAMES, TeleopTarget


DEFAULT_RATE_HZ = 50.0
DEFAULT_MAX_DELTA_RAD = 0.004
DEFAULT_MAX_VELOCITY_RAD_S = 0.20
MAX_ALLOWED_VELOCITY_RAD_S = 0.25
DEFAULT_RELEASE_IDLE_S = 0.35


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="受主导关节门控保护的六轴 Phase-A 遥操作测试")
    parser.add_argument("--leader", type=Path, required=True)
    parser.add_argument("--xarm", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--xarm-ip", required=True, help="只填 IP，不要加 :502")
    parser.add_argument("--duration-s", type=float, default=20.0, help="最长测试时长，默认 20 秒")
    parser.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument("--max-velocity-rad-s", type=float, default=DEFAULT_MAX_VELOCITY_RAD_S)
    parser.add_argument("--release-idle-s", type=float, default=DEFAULT_RELEASE_IDLE_S)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.duration_s <= 0 or args.release_idle_s <= 0:
        raise ValueError("duration-s 和 release-idle-s 必须为正数")
    if not 20 <= args.rate_hz <= 100:
        raise ValueError("rate-hz 必须在 20--100 Hz 内")
    if not 0 < args.max_velocity_rad_s <= MAX_ALLOWED_VELOCITY_RAD_S:
        raise ValueError("max-velocity-rad-s 必须在 (0, 0.25] 内")

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

    print("危险边界：此命令会使能 xArm，并以主导关节门控方式测试 J1--J6。")
    print("任一时刻只有一个 leader 关节可改变目标；暂停后锁存该目标，才可选择下一轴。")
    print("夹爪不会被使能或发送命令；Ctrl-C 可立即停止。")
    print(
        f"{args.rate_hz:.0f} Hz；最高目标变化率 {args.max_velocity_rad_s:.2f} rad/s；"
        f"停顿 {args.release_idle_s:.2f} s 后切换下一轴；最长 {args.duration_s:.0f} 秒。"
    )
    input("请确认 reduced mode、空工作区与实体急停均已就绪；按 Enter 执行只读预检：")
    leader.connect()
    xarm.connect()
    xarm.preflight()
    input("让 xArm 与 leader 处于安全起始姿态；按 Enter 记录 session zero 并开始：")
    zero_raw = leader.read_raw()
    zero_xarm = xarm.joint_positions()
    gate = DominantAxisGate(calibration, zero_raw, zero_xarm, args.release_idle_s)
    previous = TeleopTarget(zero_xarm, 0.0)
    print("零点已记录；正在确认 xArm servo mode。")
    xarm.arm_for_joint_servo_without_gripper()
    xarm.send_joint_target(previous.joints_rad)
    print("六轴门控测试已开始：移动一个 leader 轴后停住约 0.35 秒，再移动下一轴。")

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
        update = gate.update(raw, loop_s)
        target = limiter.apply(update.requested, previous.joints_rad, loop_s - previous_tick_s)
        if update.activated_index is not None:
            print(f"[选择] {JOINT_NAMES[update.activated_index]}；其它五轴保持已锁存目标。")
        if update.released_index is not None:
            gate.latch_limited_joints(target.joints_rad)
            print(f"[锁存] {JOINT_NAMES[update.released_index]}；现在可移动下一轴。")
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
            next_deadline_s = time.monotonic() + period_s
            time.sleep(period_s)
    xarm.stop()
    xarm.disconnect()
    leader.disconnect()
    elapsed_s = time.monotonic() - started_s
    print(
        f"六轴门控测试结束，已停止 xArm；未发送夹爪命令。实际 {tick_count / elapsed_s:.1f} Hz，"
        f"超时周期 {missed_deadlines}/{tick_count}。\n"
        f"leader 读取：平均 {1000 * leader_read_total_s / tick_count:.2f} ms，最大 {1000 * leader_read_max_s:.2f} ms；"
        f"xArm servo 发送：平均 {1000 * xarm_send_total_s / tick_count:.2f} ms，最大 {1000 * xarm_send_max_s:.2f} ms。"
    )


if __name__ == "__main__":
    main()
