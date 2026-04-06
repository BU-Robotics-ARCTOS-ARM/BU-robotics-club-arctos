"""Layer 2: MKS SERVO42D/57D CAN command protocol."""

from __future__ import annotations

import time

from arctos.can_interface import CANInterface


class MotorDriver:
    """CAN command interface for a single MKS SERVO motor.

    Speaks the MKS CAN protocol: builds command frames with CRC,
    sends them via CANInterface, and parses response frames.
    One instance per physical motor.
    """

    def __init__(self, can: CANInterface, can_id: int):
        self._can = can
        self._can_id = can_id

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def calc_crc(can_id: int, data: bytes) -> int:
        """CRC = (can_id + sum(data)) & 0xFF"""
        return (can_id + sum(data)) & 0xFF

    @staticmethod
    def _encode_uint16(value: int) -> bytes:
        return value.to_bytes(2, "big")

    @staticmethod
    def _encode_int24(value: int) -> bytes:
        if value < 0:
            value += 1 << 24
        return value.to_bytes(3, "big")

    # ------------------------------------------------------------------
    # CAN send / receive with CRC
    # ------------------------------------------------------------------

    def _send_command(self, data: bytes) -> None:
        """Append CRC and send a command frame."""
        crc = self.calc_crc(self._can_id, data)
        self._can.send(self._can_id, data + bytes([crc]))

    def _send_and_receive(self, data: bytes, timeout: float = 1.0) -> bytes | None:
        """Send a command and wait for the matching response.

        Returns the response payload (without CRC) or None on
        timeout / CRC mismatch.  Discards stale responses whose
        command echo byte does not match ``data[0]``.
        """
        self._send_command(data)
        expected_cmd = data[0]
        deadline = time.monotonic() + timeout

        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None

            result = self._can.receive(can_id=self._can_id, timeout=remaining)
            if result is None:
                return None

            _, resp = result
            if len(resp) < 2:
                return None

            payload = resp[:-1]
            received_crc = resp[-1]

            if self.calc_crc(self._can_id, payload) != received_crc:
                return None

            if payload[0] != expected_cmd:
                continue  # stale response from a prior command

            return payload

    # ------------------------------------------------------------------
    # Write commands
    # ------------------------------------------------------------------

    def enable(self, enabled: bool = True) -> bool:
        """Enable or disable motor (command 0xF3)."""
        val = 0x01 if enabled else 0x00
        resp = self._send_and_receive(bytes([0xF3, val]))
        if resp is None or len(resp) < 2:
            return False
        expected_status = 1 if enabled else 0
        return resp[1] == expected_status

    def move_to_axis(self, position: int, speed: int, acc: int) -> bool:
        """Absolute axis move (command 0xF5). Returns True if accepted."""
        data = (
            bytes([0xF5])
            + self._encode_uint16(speed)
            + bytes([acc])
            + self._encode_int24(position)
        )
        resp = self._send_and_receive(data)
        if resp is None or len(resp) < 2:
            return False
        return resp[1] == 2

    def set_speed(self, direction: int, speed: int, acc: int) -> bool:
        """Speed mode (command 0xF6). Returns True if accepted."""
        dir_speed = ((direction & 1) << 15) | (speed & 0x0FFF)
        data = bytes([0xF6]) + self._encode_uint16(dir_speed) + bytes([acc])
        resp = self._send_and_receive(data)
        if resp is None or len(resp) < 2:
            return False
        return resp[1] == 2

    def stop(self, acc: int = 0) -> bool:
        """Stop motor via speed mode with speed=0 (command 0xF6).

        acc=0 for immediate stop, acc>0 for deceleration ramp.
        """
        data = bytes([0xF6]) + self._encode_uint16(0) + bytes([acc])
        resp = self._send_and_receive(data)
        if resp is None or len(resp) < 2:
            return False
        return resp[1] == 2

    def emergency_stop(self) -> bool:
        """Emergency stop (command 0xF7). Returns True if confirmed."""
        resp = self._send_and_receive(bytes([0xF7]))
        if resp is None or len(resp) < 2:
            return False
        return resp[1] == 1
