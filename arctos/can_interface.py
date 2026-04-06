"""Layer 1: Raw CAN bus send/receive via python-can."""

import time
import can


class CANInterface:
    """Thin wrapper around python-can. Sends and receives raw CAN frames."""

    def __init__(self, channel="can0", bitrate=500000, interface="socketcan"):
        """Open CAN bus connection."""
        self._bus = can.interface.Bus(
            channel=channel, interface=interface, bitrate=bitrate
        )

    def send(self, can_id: int, data: bytes) -> None:
        """Send a raw CAN frame."""
        msg = can.Message(
            arbitration_id=can_id,
            data=list(data),
            is_extended_id=False,
        )
        self._bus.send(msg)

    def receive(
        self, can_id: int | None = None, timeout: float = 1.0
    ) -> tuple[int, bytes] | None:
        """Receive a CAN frame. Returns (can_id, data) or None on timeout.

        If can_id is provided, only returns frames matching that ID.
        """
        deadline = time.time() + timeout
        while True:
            remaining = deadline - time.time()
            if remaining <= 0:
                return None
            msg = self._bus.recv(timeout=remaining)
            if msg is None:
                return None
            if can_id is None or msg.arbitration_id == can_id:
                return (msg.arbitration_id, bytes(msg.data))

    def close(self) -> None:
        """Close the connection."""
        self._bus.shutdown()
