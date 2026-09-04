#!/usr/bin/env python3
"""Read-only interactive check for a 6-axis Dynamixel GELLO leader.

This script only sends Protocol 2.0 ping/read packets. It never writes motor
IDs, torque, operating mode, baud rate, or goal position.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from statistics import median

from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler


PROTOCOL_VERSION = 2.0
PRESENT_POSITION_ADDRESS = 132
PRESENT_POSITION_LENGTH = 4
TORQUE_ENABLE_ADDRESS = 64
MOTOR_SEQUENCE = (
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_1", 4),
    ("wrist_2", 5),
    ("wrist_3", 6),
    ("gripper", 7),
)
EXPECTED_MODELS = {1: 1200, 2: 1200, 3: 1200, 4: 1200, 5: 1200, 6: 1200, 7: 1190}
PHYSICAL_LABELS = {
    "shoulder_pan": "J1 底座旋转轴",
    "shoulder_lift": "J2 大臂抬降轴",
    "elbow_flex": "J3 肘部轴",
    "wrist_1": "J4 腕部第一轴",
    "wrist_2": "J5 腕部第二轴",
    "wrist_3": "J6 末端旋转轴",
    "gripper": "夹爪触发器",
}


def signed_32bit(value: int) -> int:
    return value - 2**32 if value >= 2**31 else value


def position_delta_report(
    before: dict[int, int], after: dict[int, int], target_id: int, minimum_delta_raw: int
) -> dict[str, object]:
    deltas = {motor_id: after[motor_id] - before[motor_id] for _, motor_id in MOTOR_SEQUENCE}
    ranked_ids = sorted(deltas, key=lambda motor_id: abs(deltas[motor_id]), reverse=True)
    active_ids = [motor_id for motor_id in ranked_ids if abs(deltas[motor_id]) >= minimum_delta_raw]
    dominant_id = ranked_ids[0]
    dominant_abs_delta = abs(deltas[dominant_id])
    return {
        "target_id": target_id,
        "target_delta_raw": deltas[target_id],
        "active_ids": active_ids,
        "ranked_ids": ranked_ids,
        "dominant_id": dominant_id,
        "dominant_abs_delta_raw": dominant_abs_delta,
        "target_rank": ranked_ids.index(target_id) + 1,
        "deltas_raw": deltas,
        "target_is_dominant": dominant_id == target_id and dominant_abs_delta >= minimum_delta_raw,
    }


def open_bus(port_name: str, baudrate: int) -> tuple[PortHandler, PacketHandler, GroupSyncRead]:
    port = PortHandler(port_name)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise RuntimeError(f"无法打开串口：{port_name}")
    if not port.setBaudRate(baudrate):
        raise RuntimeError(f"无法设置波特率：{baudrate}")
    reader = GroupSyncRead(port, packet, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH)
    for _, motor_id in MOTOR_SEQUENCE:
        if not reader.addParam(motor_id):
            raise RuntimeError(f"无法把 ID {motor_id} 加入位置读取器")
    return port, packet, reader


def verify_bus(packet: PacketHandler, port: PortHandler, rounds: int) -> dict[int, list[int]]:
    observed: dict[int, list[int]] = {}
    for round_index in range(rounds):
        motors, result = packet.broadcastPing(port)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"第 {round_index + 1} 轮 ping 失败：{packet.getTxRxResult(result)}")
        print(f"第 {round_index + 1} 轮 ping，响应 ID：{sorted(motors)}")
        for motor_id, values in motors.items():
            observed.setdefault(int(motor_id), []).append(int(values[0]))

    expected_ids = set(EXPECTED_MODELS)
    if set(observed) != expected_ids:
        raise RuntimeError(f"预期 ID 为 {sorted(expected_ids)}，实际响应为 {sorted(observed)}")
    if any(len(models) != rounds for models in observed.values()):
        raise RuntimeError(f"ping 响应不稳定：{observed}")
    for motor_id, expected_model in EXPECTED_MODELS.items():
        models = observed[motor_id]
        if set(models) != {expected_model}:
            raise RuntimeError(f"ID {motor_id}：预期型号号 {expected_model}，实际为 {models}")
    return observed


def verify_torque_is_off(packet: PacketHandler, port: PortHandler) -> None:
    enabled_ids = []
    for _, motor_id in MOTOR_SEQUENCE:
        value, result, error = packet.read1ByteTxRx(port, motor_id, TORQUE_ENABLE_ADDRESS)
        if result != COMM_SUCCESS:
            raise RuntimeError(f"无法读取 ID {motor_id} 的扭矩状态：{packet.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"读取 ID {motor_id} 的扭矩状态时收到设备错误码 {error}")
        if value != 0:
            enabled_ids.append(motor_id)
    if enabled_ids:
        raise RuntimeError(f"ID {enabled_ids} 的扭矩处于开启状态。请先关闭扭矩，再手动移动被动 leader。")


def read_median_positions(reader: GroupSyncRead, packet: PacketHandler, samples: int) -> dict[int, int]:
    values_by_motor = {motor_id: [] for _, motor_id in MOTOR_SEQUENCE}
    for _ in range(samples):
        result = reader.txRxPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError(f"读取位置失败：{packet.getTxRxResult(result)}")
        for _, motor_id in MOTOR_SEQUENCE:
            if not reader.isAvailable(motor_id, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH):
                raise RuntimeError(f"没有收到 ID {motor_id} 的当前位置")
            raw = int(reader.getData(motor_id, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH))
            values_by_motor[motor_id].append(signed_32bit(raw))
        time.sleep(0.02)
    return {motor_id: int(median(values)) for motor_id, values in values_by_motor.items()}


def print_delta_report(name: str, report: dict[str, object], counts_per_turn: int) -> None:
    target_delta = int(report["target_delta_raw"])
    degrees = target_delta * 360.0 / counts_per_turn
    print(f"\n【{PHYSICAL_LABELS[name]} / {name}】")
    print(f"目标 ID {report['target_id']} 的变化：{target_delta:+d} counts（{degrees:+.2f}°）")
    print(f"所有编码器变化：{report['deltas_raw']}")
    print(f"按绝对变化量排序的 ID：{report['ranked_ids']}")
    print(f"主导编码器：ID {report['dominant_id']}；目标 ID 排名：第 {report['target_rank']} 位")
    if report["target_is_dominant"]:
        print("候选通过：目标 ID 的变化最大。请在固定相邻连杆的条件下再重复一次；两次主导 ID 相同才接受该映射。")
    elif target_delta == 0:
        print("结果未定：目标 ID 没有变化。请确认操作的是对应物理关节，并再移动大一些。")
    else:
        print(
            "结果未定：其它编码器的变化更大。被动臂的自由关节会随重力和连杆运动，这是常见现象。"
            "请用双手固定被测转轴两侧的连杆，只让该转轴小幅转动后重试。"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="只读检查 GELLO 关节顺序和编码器方向")
    parser.add_argument("--port", required=True, help="建议使用稳定的 /dev/serial/by-id/... 路径")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--counts-per-turn", type=int, default=4096)
    parser.add_argument("--ping-rounds", type=int, default=3)
    parser.add_argument("--samples", type=int, default=5, help="每次前后采样的中位数样本数")
    parser.add_argument("--minimum-delta-raw", type=int, default=80, help="判定发生动作的编码器阈值（counts）")
    parser.add_argument("--joint", choices=[name for name, _ in MOTOR_SEQUENCE], help="仅检查一个关节，不传则依次检查全部 7 轴")
    parser.add_argument("--output", type=Path, help="可选：保存观测结果的 JSON 文件")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.ping_rounds <= 0 or args.samples <= 0 or args.minimum_delta_raw <= 0:
        raise ValueError("ping-rounds、samples 和 minimum-delta-raw 必须为正数")

    port, packet, reader = open_bus(args.port, args.baudrate)
    print("只读模式：本程序不会写入 Dynamixel 的 ID、扭矩、模式、波特率或位置目标。")
    verify_bus(packet, port, args.ping_rounds)
    verify_torque_is_off(packet, port)
    print("7 个电机均符合预期 ID/型号，且扭矩全部关闭。")

    sequence = [(name, motor_id) for name, motor_id in MOTOR_SEQUENCE if args.joint in (None, name)]
    results = []
    for name, motor_id in sequence:
        physical_label = PHYSICAL_LABELS[name]
        input(
            f"\n【准备检查：{physical_label}（逻辑名 {name}，预期 ID {motor_id}）】\n"
            "请让整条 leader 保持静止约 2 秒，不要碰任何关节；准备好后按 Enter 记录基准位置。"
        )
        before = read_median_positions(reader, packet, args.samples)
        input(
            f"请用双手夹稳 {physical_label} 两侧的连杆，避免拖动整条机械臂。\n"
            "随后只转动该转轴约 10–30°，保持住不要松手；完成后按 Enter 记录结果。"
        )
        after = read_median_positions(reader, packet, args.samples)
        report = position_delta_report(before, after, motor_id, args.minimum_delta_raw)
        print_delta_report(name, report, args.counts_per_turn)
        results.append({"motor_name": name, **report})

    port.closePort()
    summary = {"port": args.port, "baudrate": args.baudrate, "counts_per_turn": args.counts_per_turn, "results": results}
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"结果已保存：{args.output}")


if __name__ == "__main__":
    main()
