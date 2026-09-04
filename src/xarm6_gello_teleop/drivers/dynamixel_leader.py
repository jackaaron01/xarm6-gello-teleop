from __future__ import annotations

from dataclasses import dataclass

from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler

from ..config import LeaderConfig


PROTOCOL_VERSION = 2.0
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
OPERATING_MODE_POSITION = 3
OPERATING_MODE_EXTENDED_POSITION = 4


@dataclass
class DynamixelLeader:
    """Passive six-axis GELLO reader using only the Dynamixel SDK."""

    config: LeaderConfig
    _port: PortHandler | None = None
    _packet: PacketHandler | None = None

    @property
    def connected(self) -> bool:
        return self._port is not None and self._packet is not None

    def connect(self) -> None:
        if self.connected:
            raise RuntimeError("Dynamixel leader is already connected")
        port = PortHandler(self.config.serial_port)
        if not port.openPort():
            raise RuntimeError(f"Cannot open Dynamixel port {self.config.serial_port}")
        if not port.setBaudRate(self.config.baud_rate):
            raise RuntimeError(f"Cannot set baud rate {self.config.baud_rate}")
        self._port = port
        self._packet = PacketHandler(PROTOCOL_VERSION)
        self.disable_torque()
        self._set_position_modes()

    def _require_connection(self) -> tuple[PortHandler, PacketHandler]:
        if self._port is None or self._packet is None:
            raise RuntimeError("Dynamixel leader is not connected")
        return self._port, self._packet

    def _check(self, communication_result: int, packet_error: int, operation: str) -> None:
        _, packet = self._require_connection()
        if communication_result != COMM_SUCCESS:
            raise RuntimeError(f"{operation}: {packet.getTxRxResult(communication_result)}")
        if packet_error:
            raise RuntimeError(f"{operation}: {packet.getRxPacketError(packet_error)}")

    def _write_byte(self, motor_id: int, address: int, value: int, operation: str) -> None:
        port, packet = self._require_connection()
        communication_result, packet_error = packet.write1ByteTxRx(port, motor_id, address, value)
        self._check(communication_result, packet_error, operation)

    def disable_torque(self) -> None:
        for motor in self.config.motors:
            self._write_byte(motor.motor_id, ADDR_TORQUE_ENABLE, 0, f"disable torque on {motor.name}")

    def _set_position_modes(self) -> None:
        for motor in self.config.motors:
            mode = OPERATING_MODE_POSITION if motor.name == "gripper" else OPERATING_MODE_EXTENDED_POSITION
            self._write_byte(motor.motor_id, ADDR_OPERATING_MODE, mode, f"set mode on {motor.name}")

    def read_raw(self) -> dict[str, int]:
        port, packet = self._require_connection()
        reader = GroupSyncRead(port, packet, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        for motor in self.config.motors:
            if not reader.addParam(motor.motor_id):
                raise RuntimeError(f"Cannot add Dynamixel ID {motor.motor_id} to sync read")
        communication_result = reader.txRxPacket()
        self._check(communication_result, 0, "read leader positions")
        positions = {}
        for motor in self.config.motors:
            if not reader.isAvailable(motor.motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                raise RuntimeError(f"No present position from {motor.name} (ID {motor.motor_id})")
            value = int(reader.getData(motor.motor_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))
            positions[motor.name] = value - 2**32 if value >= 2**31 else value
        reader.clearParam()
        return positions

    def disconnect(self) -> None:
        if self._port is not None:
            self._port.closePort()
        self._port = None
        self._packet = None
