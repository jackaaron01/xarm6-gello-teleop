from __future__ import annotations

import argparse
import signal
import time

from .config import LeaderConfig, RelativeCalibration, XArmConfig
from .drivers.dynamixel_leader import DynamixelLeader
from .drivers.xarm6 import XArm6
from .relative_mapper import RelativeMapper
from .runtime import TeleopRuntime


def _add_hardware_arguments(parser: argparse.ArgumentParser, calibration: bool = False) -> None:
    parser.add_argument("--leader", required=True, help="Path to the GELLO hardware YAML")
    parser.add_argument("--xarm-ip", required=True, help="xArm controller IP address")
    parser.add_argument("--xarm", required=True, help="Path to xArm hardware YAML")
    if calibration:
        parser.add_argument("--calibration", required=True, help="Path to the verified relative calibration YAML")


def _runtime(args: argparse.Namespace) -> TeleopRuntime:
    leader_config = LeaderConfig.from_file(args.leader)
    xarm_config = XArmConfig.from_file(args.xarm)
    calibration = RelativeCalibration.from_file(args.calibration)
    if calibration.counts_per_turn != leader_config.counts_per_turn:
        raise ValueError("Calibration counts_per_turn differs from leader hardware profile")
    return TeleopRuntime(DynamixelLeader(leader_config), XArm6(args.xarm_ip), RelativeMapper(calibration), xarm_config)


def diagnose_leader(args: argparse.Namespace) -> None:
    leader = DynamixelLeader(LeaderConfig.from_file(args.leader))
    leader.connect()
    print(leader.read_raw())
    leader.disconnect()


def diagnose_xarm(args: argparse.Namespace) -> None:
    xarm = XArm6(args.xarm_ip)
    xarm.connect()
    xarm.preflight()
    print({"joint_positions_rad": xarm.joint_positions()})
    xarm.disconnect()


def teleop(args: argparse.Namespace) -> None:
    runtime = _runtime(args)
    runtime.connect()
    print("Connected in read-only state. Make the robot and passive leader physically safe.")
    if input("Type ALIGN after placing both at the intended session start pose: ").strip() != "ALIGN":
        runtime.disconnect()
        raise RuntimeError("Session aborted before alignment")
    runtime.align_session_zero()
    runtime.preflight()
    if input("Preflight passed. Type ARM to enable xArm servo mode (no motion target yet): ").strip() != "ARM":
        runtime.disconnect()
        raise RuntimeError("Session aborted before arming")
    runtime.arm()
    if input("Type START to begin low-speed teleoperation: ").strip() != "START":
        runtime.disconnect()
        raise RuntimeError("Session aborted before start")
    runtime.start()

    def stop_on_signal(signum: int, frame: object) -> None:
        runtime.stop()
        runtime.disconnect()
        raise SystemExit(f"Stopped by signal {signum}")

    signal.signal(signal.SIGINT, stop_on_signal)
    signal.signal(signal.SIGTERM, stop_on_signal)
    period_s = 1.0 / runtime.config.command_rate_hz
    while True:
        started_s = time.monotonic()
        runtime.tick()
        remaining_s = period_s - (time.monotonic() - started_s)
        if remaining_s > 0:
            time.sleep(remaining_s)


def main() -> None:
    parser = argparse.ArgumentParser(prog="xarm6-gello")
    commands = parser.add_subparsers(dest="command", required=True)
    leader_parser = commands.add_parser("diagnose-leader", help="Read GELLO raw positions with torque disabled")
    leader_parser.add_argument("--leader", required=True)
    leader_parser.set_defaults(func=diagnose_leader)
    xarm_parser = commands.add_parser("diagnose-xarm", help="Read xArm health and joint angles; never enables motion")
    xarm_parser.add_argument("--xarm-ip", required=True)
    xarm_parser.set_defaults(func=diagnose_xarm)
    teleop_parser = commands.add_parser("teleop", help="Run explicitly-confirmed joint-space teleoperation")
    _add_hardware_arguments(teleop_parser, calibration=True)
    teleop_parser.set_defaults(func=teleop)
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
