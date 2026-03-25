"""Layer 1: Raw CAN bus send/receive via python-can."""
import os
import time
import subprocess
import argparse
import can

# ─── Configuration ──────────────────────────────────────────────────────────
DEFAULT_CAN_ID    = 0x01       # Slave CAN ID (set on motor OLED: CanID)
DEFAULT_BITRATE   = 500000     # 500K (set on motor OLED: CanRate)
DEFAULT_INTERFACE = "socketcan" # Linux SocketCAN
DEFAULT_CHANNEL   = "vcan0"
RECV_TIMEOUT      = 1.0        # seconds

# ─── CAN Send / Receive ────────────────────────────────────────────────────
def open_canbus(channel=DEFAULT_CHANNEL, interface=DEFAULT_INTERFACE, bitrate=DEFAULT_BITRATE):
    """Open and return a CAN bus connection."""
    print(f"[*] Opening CAN bus: interface={interface}, channel={channel}")
    return can.interface.Bus(channel=channel, interface=interface, bitrate=bitrate)


def awaitresponse(bus, can_id, timeout=RECV_TIMEOUT):
    """
    Wait for a CAN response matching the given CAN ID.
    Returns the response data bytes as a list, or None on timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        resp = bus.recv(timeout=timeout)
        if resp and resp.arbitration_id == can_id:
            rdata = list(resp.data)
            print(f"  RX ← ID=0x{resp.arbitration_id:03X}  Data=[{' '.join(f'{b:02X}' for b in rdata)}]")
            return rdata
    print("  RX ← (no response / timeout)")
    return None


def send(bus, can_id, data, expect_response=True):
    """
    Send a CAN frame.
    Returns the response data bytes, or None on timeout.
    """
    payload = list(data)

    msg = can.Message(
        arbitration_id=can_id,
        data=payload,
        is_extended_id=False,
    )

    print(f"  TX → ID=0x{can_id:03X}  Data=[{' '.join(f'{b:02X}' for b in payload)}]")
    bus.send(msg)

    if not expect_response:
        return None

    return awaitresponse(bus, can_id)

def close_canbus(bus):
    """Close the CAN bus connection."""
    print("[*] Closing CAN bus")
    bus.shutdown()

