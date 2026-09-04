from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .types import JOINT_NAMES, six


def load_yaml(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as file:
        data = yaml.safe_load(file)
    if not isinstance(data, dict):
        raise ValueError(f"Expected a mapping in {path}")
    return data


@dataclass(frozen=True)
class MotorConfig:
    name: str
    motor_id: int
    model: str


@dataclass(frozen=True)
class LeaderConfig:
    serial_port: str
    baud_rate: int
    counts_per_turn: int
    motors: tuple[MotorConfig, ...]

    @classmethod
    def from_file(cls, path: str | Path) -> "LeaderConfig":
        data = load_yaml(path)
        motors = tuple(MotorConfig(item["name"], int(item["id"]), item["model"]) for item in data["motors"])
        names = tuple(motor.name for motor in motors)
        if names != (*JOINT_NAMES, "gripper"):
            raise ValueError(f"Motors must be ordered as six joints then gripper, got {names}")
        return cls(data["serial_port"], int(data["baud_rate"]), int(data["counts_per_turn"]), motors)


@dataclass(frozen=True)
class GripperConfig:
    closed_pulse: int
    open_pulse: int
    speed: int
    minimum_delta_pulse: int


@dataclass(frozen=True)
class XArmConfig:
    joint_lower_rad: tuple[float, float, float, float, float, float]
    joint_upper_rad: tuple[float, float, float, float, float, float]
    max_delta_rad: float
    max_velocity_rad_s: float
    command_rate_hz: float
    leader_timeout_s: float
    gripper: GripperConfig

    @classmethod
    def from_file(cls, path: str | Path) -> "XArmConfig":
        data = load_yaml(path)
        gripper = data["gripper"]
        return cls(
            six(data["joint_lower_rad"]), six(data["joint_upper_rad"]), float(data["max_delta_rad"]),
            float(data["max_velocity_rad_s"]), float(data["command_rate_hz"]), float(data["leader_timeout_s"]),
            GripperConfig(int(gripper["closed_pulse"]), int(gripper["open_pulse"]), int(gripper["speed"]), int(gripper["minimum_delta_pulse"])),
        )


@dataclass(frozen=True)
class RelativeCalibration:
    joint_names: tuple[str, ...]
    counts_per_turn: int
    sign: tuple[float, float, float, float, float, float]
    gain_rad_per_turn: tuple[float, float, float, float, float, float]
    joint_deadband_raw: tuple[float, float, float, float, float, float]
    gripper_motor_name: str
    gripper_raw_closed: float
    gripper_raw_open: float

    @classmethod
    def from_file(cls, path: str | Path) -> "RelativeCalibration":
        data = load_yaml(path)
        names = tuple(data["joint_names"])
        if names != JOINT_NAMES:
            raise ValueError(f"Calibration joint_names must be {JOINT_NAMES}, got {names}")
        gripper = data["gripper"]
        if float(gripper["raw_open"]) == float(gripper["raw_closed"]):
            raise ValueError("Gripper raw_open and raw_closed must differ")
        deadband = six(data["joint_deadband_raw"])
        if any(value < 0 for value in deadband):
            raise ValueError("joint_deadband_raw must not contain negative values")
        return cls(
            names, int(data["counts_per_turn"]), six(data["sign"]), six(data["gain_rad_per_turn"]), deadband,
            str(gripper["motor_name"]), float(gripper["raw_closed"]), float(gripper["raw_open"]),
        )
