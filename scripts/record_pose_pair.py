#!/usr/bin/env python3
"""Record one read-only GELLO/xArm joint-space pose pair for calibration."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from statistics import median

from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
from xarm.wrapper import XArmAPI


PROTOCOL_VERSION = 2.0
PRESENT_POSITION_ADDRESS = 132
PRESENT_POSITION_LENGTH = 4
TORQUE_ENABLE_ADDRESS = 64
MOTORS = (
    ("shoulder_pan", 1, 1200),
    ("shoulder_lift", 2, 1200),
    ("elbow_flex", 3, 1200),
    ("wrist_1", 4, 1200),
    ("wrist_2", 5, 1200),
    ("wrist_3", 6, 1200),
    ("gripper", 7, 1190),
)


def signed_32bit(value: int) -> int:
    return value - 2**32 if value >= 2**31 else value


def open_leader(port_name: str, baudrate: int) -> tuple[PortHandler, PacketHandler, GroupSyncRead]:
    port = PortHandler(port_name)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise RuntimeError(f"无法打开 Dynamixel 串口：{port_name}")
    if not port.setBaudRate(baudrate):
        raise RuntimeError(f"无法设置 Dynamixel 波特率：{baudrate}")
    for name, motor_id, expected_model in MOTORS:
        model, result, error = packet.ping(port, motor_id)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"无法 ping {name}/ID {motor_id}：{packet.getTxRxResult(result)}")
        if error or model != expected_model:
            raise RuntimeError(f"{name}/ID {motor_id} 型号或状态异常：model={model}, error={error}")
        torque, result, error = packet.read1ByteTxRx(port, motor_id, TORQUE_ENABLE_ADDRESS)
        if result != COMM_SUCCESS or error:
            raise RuntimeError(f"无法读取 {name}/ID {motor_id} 的扭矩状态")
        if torque != 0:
            raise RuntimeError(f"{name}/ID {motor_id} 的扭矩已开启；只读采集要求所有 leader 电机扭矩关闭")
    reader = GroupSyncRead(port, packet, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH)
    for _, motor_id, _ in MOTORS:
        if not reader.addParam(motor_id):
            raise RuntimeError(f"无法将 ID {motor_id} 加入位置读取器")
    return port, packet, reader


def read_leader(reader: GroupSyncRead, packet: PacketHandler, samples: int) -> dict[str, int]:
    values: dict[str, list[int]] = {name: [] for name, _, _ in MOTORS}
    for _ in range(samples):
        result = reader.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"读取 Dynamixel 位置失败：{packet.getTxRxResult(result)}")
        for name, motor_id, _ in MOTORS:
            if not reader.isAvailable(motor_id, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH):
                raise RuntimeError(f"未收到 {name}/ID {motor_id} 的位置")
            values[name].append(
                signed_32bit(int(reader.getData(motor_id, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH)))
            )
        time.sleep(0.02)
    return {name: int(median(readings)) for name, readings in values.items()}


def read_xarm(ip: str) -> tuple[float, float, float, float, float, float]:
    arm = XArmAPI(ip, is_radian=True, do_not_open=True)
    arm.connect()
    if not arm.connected:
        raise RuntimeError(f"无法连接 xArm：{ip}")
    code, errors = arm.get_err_warn_code()
    if code != 0:
        raise RuntimeError(f"读取 xArm 错误状态失败，错误码：{code}")
    if any(int(value) != 0 for value in errors):
        raise RuntimeError(f"xArm 存在错误/警告 {errors}；请先在 xArm Studio 排除")
    code, angles = arm.get_servo_angle(is_radian=True)
    if code != 0:
        raise RuntimeError(f"读取 xArm 六轴角度失败，错误码：{code}")
    arm.disconnect()
    if len(angles) < 6:
        raise RuntimeError(f"xArm 返回的关节数量不足：{angles}")
    return tuple(float(value) for value in angles[:6])


def validate_target_pose(
    reference: dict[str, object],
    leader_raw: dict[str, int],
    xarm_joints: tuple[float, float, float, float, float, float],
    target_joint: str,
    minimum_target_raw_delta: int,
    minimum_target_xarm_delta_rad: float,
    maximum_other_xarm_delta_rad: float,
) -> dict[str, float]:
    if reference.get("schema") != "xarm6-gello-pose-pair/v1":
        raise ValueError("reference-p0 不是有效姿态对文件")
    target_index = tuple(name for name, _, _ in MOTORS).index(target_joint)
    reference_raw = reference["leader_raw"]
    reference_xarm = reference["xarm_joints_rad"]
    raw_delta = float(leader_raw[target_joint] - int(reference_raw[target_joint]))
    xarm_delta = float(xarm_joints[target_index] - float(reference_xarm[target_index]))
    other_max = max(
        abs(xarm_joints[index] - float(reference_xarm[index]))
        for index in range(6)
        if index != target_index
    )
    if abs(raw_delta) < minimum_target_raw_delta:
        raise RuntimeError(
            f"未保存：{target_joint} 的 leader 变化仅 {raw_delta:+.0f} counts，"
            f"需要至少 {minimum_target_raw_delta} counts"
        )
    if abs(xarm_delta) < minimum_target_xarm_delta_rad:
        raise RuntimeError(
            f"未保存：{target_joint} 的 xArm 变化仅 {xarm_delta:+.4f} rad；"
            "请先回到 P0，再只移动对应 xArm 轴"
        )
    if other_max > maximum_other_xarm_delta_rad:
        raise RuntimeError(
            f"未保存：除 {target_joint} 外的 xArm 轴最大变化为 {other_max:.4f} rad；"
            "请先完整回到 P0，再只移动目标轴"
        )
    return {
        "target_leader_delta_raw": raw_delta,
        "target_xarm_delta_rad": xarm_delta,
        "other_xarm_axis_max_delta_rad": other_max,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="只读记录一组 GELLO/xArm 姿态对")
    parser.add_argument("--label", required=True, help="例如 p0、shoulder_pan、wrist_3")
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--xarm-ip", required=True, help="只填 IP，不要加 :502")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--reference-p0", type=Path, help="可选：与此 P0 对比并校验单轴采样")
    parser.add_argument("--target-joint", choices=[name for name, _, _ in MOTORS], help="与 --reference-p0 配合使用")
    parser.add_argument("--minimum-target-raw-delta", type=int, default=80)
    parser.add_argument("--minimum-target-xarm-delta-deg", type=float, default=10.0)
    parser.add_argument("--maximum-other-xarm-delta-deg", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.samples <= 0:
        raise ValueError("samples 必须为正数")
    if (args.reference_p0 is None) != (args.target_joint is None):
        raise ValueError("reference-p0 和 target-joint 必须同时提供")
    if args.minimum_target_raw_delta <= 0 or args.minimum_target_xarm_delta_deg <= 0 or args.maximum_other_xarm_delta_deg <= 0:
        raise ValueError("姿态对校验阈值必须为正数")
    print("只读模式：不会写 Dynamixel，也不会使能或移动 xArm/夹爪。")
    input(
        f"请使被动 leader 的姿态与当前 xArm 尽量一致；确认记录标签“{args.label}”后，按 Enter 采集。"
    )
    port, packet, reader = open_leader(args.leader_port, args.baudrate)
    leader_raw = read_leader(reader, packet, args.samples)
    port.closePort()
    xarm_joints = read_xarm(args.xarm_ip)
    result = {
        "schema": "xarm6-gello-pose-pair/v1",
        "label": args.label,
        "recorded_at_utc": datetime.now(timezone.utc).isoformat(),
        "counts_per_turn": 4096,
        "leader_raw": leader_raw,
        "xarm_joints_rad": list(xarm_joints),
    }
    if args.reference_p0 is not None:
        reference = json.loads(args.reference_p0.read_text(encoding="utf-8"))
        validation = validate_target_pose(
            reference,
            leader_raw,
            xarm_joints,
            args.target_joint,
            args.minimum_target_raw_delta,
            args.minimum_target_xarm_delta_deg * 3.141592653589793 / 180.0,
            args.maximum_other_xarm_delta_deg * 3.141592653589793 / 180.0,
        )
        result["validation_against_p0"] = validation
        print(f"单轴样本校验通过：{validation}")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已保存只读姿态对：{args.output}")
    print({"leader_raw": leader_raw, "xarm_joints_rad": xarm_joints})


if __name__ == "__main__":
    main()
