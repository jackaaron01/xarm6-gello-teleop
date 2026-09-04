#!/usr/bin/env python3
"""Interactively calibrate the passive GELLO trigger and a standard xArm Gripper.

The Dynamixel portion is read-only.  The xArm portion never commands one of
the six arm joints, but, after two explicit confirmations, enables and moves
the *gripper only* slowly to two requested pulse endpoints.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from statistics import median

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from xarm.wrapper import XArmAPI


PROTOCOL_VERSION = 2.0
DXL_GRIPPER_ID = 7
DXL_GRIPPER_MODEL = 1190
DXL_PRESENT_POSITION_ADDRESS = 132
DXL_TORQUE_ENABLE_ADDRESS = 64


def signed_32bit(value: int) -> int:
    return value - 2**32 if value >= 2**31 else value


def require_code(code: int, operation: str) -> None:
    if code != 0:
        raise RuntimeError(f"xArm {operation} 失败，错误码：{code}")


def read_trigger_position(
    packet: PacketHandler, port: PortHandler, samples: int
) -> int:
    readings = []
    for _ in range(samples):
        value, result, error = packet.read4ByteTxRx(
            port, DXL_GRIPPER_ID, DXL_PRESENT_POSITION_ADDRESS
        )
        if result != COMM_SUCCESS:
            raise RuntimeError(f"无法读取扳机 ID 7 的位置：{packet.getTxRxResult(result)}")
        if error:
            raise RuntimeError(f"读取扳机 ID 7 时收到设备错误码：{error}")
        readings.append(signed_32bit(int(value)))
        time.sleep(0.02)
    return int(median(readings))


def connect_trigger(port_name: str, baudrate: int) -> tuple[PortHandler, PacketHandler]:
    port = PortHandler(port_name)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        raise RuntimeError(f"无法打开 Dynamixel 串口：{port_name}")
    if not port.setBaudRate(baudrate):
        raise RuntimeError(f"无法将 Dynamixel 波特率设为：{baudrate}")

    model, result, error = packet.ping(port, DXL_GRIPPER_ID)
    if result != COMM_SUCCESS:
        raise RuntimeError(f"无法 ping 扳机 ID 7：{packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"ping 扳机 ID 7 时收到设备错误码：{error}")
    if model != DXL_GRIPPER_MODEL:
        raise RuntimeError(f"扳机 ID 7 的型号应为 {DXL_GRIPPER_MODEL}，实际为 {model}")

    torque, result, error = packet.read1ByteTxRx(port, DXL_GRIPPER_ID, DXL_TORQUE_ENABLE_ADDRESS)
    if result != COMM_SUCCESS:
        raise RuntimeError(f"无法读取扳机 ID 7 的扭矩状态：{packet.getTxRxResult(result)}")
    if error:
        raise RuntimeError(f"读取扳机 ID 7 的扭矩状态时收到设备错误码：{error}")
    if torque != 0:
        raise RuntimeError("扳机 ID 7 的扭矩已开启。请先关闭扭矩后再校准被动 leader。")
    return port, packet


def observation(prompt: str) -> str:
    while True:
        value = input(prompt).strip().upper()
        if value in {"OPEN", "CLOSED"}:
            return value
        print("请输入 OPEN（张开）或 CLOSED（闭合）。")


def derive_calibration(
    trigger_open_raw: int,
    trigger_closed_raw: int,
    endpoint_a_pulse: int,
    endpoint_a_observation: str,
    endpoint_b_pulse: int,
    endpoint_b_observation: str,
) -> dict[str, object]:
    if trigger_open_raw == trigger_closed_raw:
        raise ValueError("扳机松开与扣下的位置相同，无法建立夹爪标定")
    if endpoint_a_observation == endpoint_b_observation:
        raise ValueError("两个 xArm 端点观察结果相同，无法判断夹爪开合方向")
    if endpoint_a_observation == "OPEN":
        open_pulse, closed_pulse = endpoint_a_pulse, endpoint_b_pulse
    else:
        open_pulse, closed_pulse = endpoint_b_pulse, endpoint_a_pulse
    return {
        "leader": {
            "motor_name": "gripper",
            "motor_id": DXL_GRIPPER_ID,
            "raw_open": trigger_open_raw,
            "raw_closed": trigger_closed_raw,
        },
        "xarm_standard_gripper": {
            "open_pulse": open_pulse,
            "closed_pulse": closed_pulse,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="校准 GELLO 扳机和标准 xArm Gripper（六轴不运动）")
    parser.add_argument("--leader-port", required=True, help="Dynamixel 稳定串口路径")
    parser.add_argument("--xarm-ip", required=True, help="xArm IP；只填主机 IP，不要加 :502")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--speed", type=int, default=200, help="xArm 夹爪速度（r/min），默认低速 200")
    parser.add_argument("--endpoint-a", type=int, default=0, help="首先测试的 xArm 夹爪 pulse")
    parser.add_argument("--endpoint-b", type=int, default=850, help="其次测试的 xArm 夹爪 pulse")
    parser.add_argument("--output", type=Path, default=Path("results/gripper_calibration.json"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if ":" in args.xarm_ip:
        raise ValueError("--xarm-ip 只能填写 IP，例如 192.168.1.100；不要附加 :502")
    if args.samples <= 0 or args.speed <= 0:
        raise ValueError("samples 和 speed 必须为正数")
    if args.endpoint_a == args.endpoint_b:
        raise ValueError("两个 xArm 夹爪端点必须不同")

    print("安全说明：Dynamixel 扳机只读；xArm 六个关节不会被使能或发送运动指令。")
    print("本脚本会在确认后使能并低速移动【标准 xArm Gripper】到两个端点。")
    print("请清空夹爪周围物体，手指远离夹爪，保持实体急停可达。")

    port, packet = connect_trigger(args.leader_port, args.baudrate)
    input("松开 GELLO 扳机并保持不动，按 Enter 记录“张开”端的 leader 位置。")
    trigger_open_raw = read_trigger_position(packet, port, args.samples)
    input("完全扣下 GELLO 扳机并保持不动，按 Enter 记录“闭合”端的 leader 位置。")
    trigger_closed_raw = read_trigger_position(packet, port, args.samples)
    port.closePort()
    print(
        f"leader 扳机：松开={trigger_open_raw}，扣下={trigger_closed_raw}，"
        f"变化={trigger_closed_raw - trigger_open_raw:+d} counts"
    )

    arm = XArmAPI(args.xarm_ip, is_radian=True, do_not_open=True)
    arm.connect()
    if not arm.connected:
        raise RuntimeError(f"无法连接 xArm：{args.xarm_ip}")
    code, errors = arm.get_err_warn_code()
    require_code(code, "get_err_warn_code")
    if any(int(value) != 0 for value in errors):
        raise RuntimeError(f"xArm 存在错误/警告 {errors}；请先在 xArm Studio 排除")
    code, initial_pulse = arm.get_gripper_position()
    require_code(code, "get_gripper_position")
    print(f"xArm 当前夹爪 pulse：{initial_pulse}")

    if input("确认现场安全后，输入 GRIPPER 以仅使能夹爪：").strip() != "GRIPPER":
        arm.disconnect()
        raise RuntimeError("未确认，已取消；没有发送夹爪运动命令。")
    require_code(arm.set_gripper_mode(0), "set_gripper_mode")
    require_code(arm.set_gripper_enable(True), "set_gripper_enable")
    require_code(arm.set_gripper_speed(args.speed), "set_gripper_speed")

    if input(f"输入 ENDPOINT_A 以低速移动夹爪到 pulse {args.endpoint_a}：").strip() != "ENDPOINT_A":
        arm.disconnect()
        raise RuntimeError("未确认第一个端点，已取消。")
    require_code(
        arm.set_gripper_position(args.endpoint_a, speed=args.speed, wait=True, timeout=30),
        f"set_gripper_position({args.endpoint_a})",
    )
    endpoint_a_observation = observation(
        "观察夹爪实际状态后，输入 OPEN（张开）或 CLOSED（闭合）："
    )

    if input(f"输入 ENDPOINT_B 以低速移动夹爪到 pulse {args.endpoint_b}：").strip() != "ENDPOINT_B":
        arm.disconnect()
        raise RuntimeError("未确认第二个端点，已取消。")
    require_code(
        arm.set_gripper_position(args.endpoint_b, speed=args.speed, wait=True, timeout=30),
        f"set_gripper_position({args.endpoint_b})",
    )
    endpoint_b_observation = observation(
        "观察夹爪实际状态后，输入 OPEN（张开）或 CLOSED（闭合）："
    )

    calibration = derive_calibration(
        trigger_open_raw,
        trigger_closed_raw,
        args.endpoint_a,
        endpoint_a_observation,
        args.endpoint_b,
        endpoint_b_observation,
    )
    calibration["xarm_standard_gripper"]["speed"] = args.speed
    calibration["xarm_ip"] = args.xarm_ip
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(calibration, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    arm.disconnect()

    xarm = calibration["xarm_standard_gripper"]
    leader = calibration["leader"]
    print(f"结果已保存：{args.output}")
    print("请将下面两段值发给我；我会把它们写入正式硬件/标定配置。")
    print(f"xArm: closed_pulse={xarm['closed_pulse']}, open_pulse={xarm['open_pulse']}, speed={xarm['speed']}")
    print(f"leader: raw_closed={leader['raw_closed']}, raw_open={leader['raw_open']}")
    print(f"夹爪最终停在 pulse {args.endpoint_b}；脚本没有发送任何六轴关节运动。")


if __name__ == "__main__":
    main()
