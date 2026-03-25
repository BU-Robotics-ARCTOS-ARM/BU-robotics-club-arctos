"""Layer 1: Raw CAN bus send/receive via python-can."""
import os
import time
import struct
import subprocess
import argparse
import can

# ─── Configuration ──────────────────────────────────────────────────────────
DEFAULT_CAN_ID    = 0x01       # Slave CAN ID (set on motor OLED: CanID)
DEFAULT_BITRATE   = 500000     # 500K (set on motor OLED: CanRate)
DEFAULT_INTERFACE = "socketcan" # Linux SocketCAN
DEFAULT_CHANNEL   = "vcan0"
RECV_TIMEOUT      = 1.0        # seconds

#remove this and put into motor driver
# # ─── CRC Calculation ────────────────────────────────────────────────────────
# def calc_crc(can_id, data_bytes):
#     """CHECKSUM 8-bit: (CAN_ID + sum of all data bytes) & 0xFF"""
#     return (can_id + sum(data_bytes)) & 0xFF

# ─── CAN Send / Receive ────────────────────────────────────────────────────
def open_canbus(channel=DEFAULT_CHANNEL, interface=DEFAULT_INTERFACE, bitrate=DEFAULT_BITRATE):
    """Open and return a CAN bus connection."""
    print(f"[*] Opening CAN bus: interface={interface}, channel={channel}")
    return can.interface.Bus(channel=channel, interface=interface, bitrate=bitrate)

def send_cmd(bus, can_id, data, expect_response=True):
    """
    Send a CAN frame with auto-appended CRC.
    Returns the response data bytes (excluding CRC), or None on timeout.
    """
    crc = calc_crc(can_id, data)
    payload = list(data) + [crc]

    msg = can.Message(
        arbitration_id=can_id,
        data=payload,
        is_extended_id=False,
    )

    print(f"  TX → ID=0x{can_id:03X}  Data=[{' '.join(f'{b:02X}' for b in payload)}]")
    bus.send(msg)

    if not expect_response:
        return None

    # Wait for response from same CAN ID
    deadline = time.time() + RECV_TIMEOUT
    while time.time() < deadline:
        resp = bus.recv(timeout=RECV_TIMEOUT)
        if resp and resp.arbitration_id == can_id:
            rdata = list(resp.data)
            print(f"  RX ← ID=0x{resp.arbitration_id:03X}  Data=[{' '.join(f'{b:02X}' for b in rdata)}]")
            return rdata
    print("  RX ← (no response / timeout)")
    return None




