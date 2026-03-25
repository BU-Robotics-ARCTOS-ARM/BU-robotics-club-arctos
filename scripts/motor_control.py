#!/usr/bin/env python3
"""Interactive CLI for controlling MKS SERVO42D/57D motors over CAN bus.

Usage:
    python scripts/motor_control.py
    python scripts/motor_control.py --channel slcan0 --interface slcan
    python scripts/motor_control.py --interface virtual   # no hardware needed

Ctrl+C sends emergency stop to ALL motors before exiting.
"""

from __future__ import annotations

import argparse
import signal
import sys

import can

# Allow running from project root: `python scripts/motor_control.py`
sys.path.insert(0, ".")
from arctos.config import (
    CAN_BITRATE,
    CAN_CHANNEL,
    DEFAULT_ACC,
    DEFAULT_SPEED,
    ENCODER_COUNTS_PER_REV,
    JOINTS,
)

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_bus: can.Bus | None = None
_selected_can_ids: list[int] = []
_selected_label: str = "None"

# ---------------------------------------------------------------------------
# CAN helpers
# ---------------------------------------------------------------------------

def calc_crc(can_id: int, data: bytes) -> int:
    return (can_id + sum(data)) & 0xFF


def send_command(bus: can.Bus, can_id: int, data_without_crc: bytes) -> None:
    crc = calc_crc(can_id, data_without_crc)
    payload = data_without_crc + bytes([crc])
    msg = can.Message(arbitration_id=can_id, data=payload, is_extended_id=False)
    bus.send(msg)


def receive_response(bus: can.Bus, expected_id: int, timeout: float = 1.0,
                     expected_cmd: int | None = None) -> bytes | None:
    """Wait for a response from expected_id. Returns data bytes (excluding CRC) or None.

    If expected_cmd is set, discards stale responses from other commands
    (e.g. move-completion messages queued after a prior 0xF5).
    """
    import time
    deadline = time.monotonic() + timeout
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        msg = bus.recv(timeout=remaining)
        if msg is None:
            return None
        if msg.arbitration_id != expected_id:
            continue  # skip messages from other motors
        data = msg.data
        if len(data) < 2:
            return None
        received_crc = data[-1]
        payload = bytes(data[:-1])
        if calc_crc(expected_id, payload) != received_crc:
            print(f"  [!] CRC mismatch from 0x{expected_id:02X}")
            return None
        if expected_cmd is not None and payload[0] != expected_cmd:
            continue  # discard stale response from a previous command
        return payload


def send_and_receive(bus: can.Bus, can_id: int, data_without_crc: bytes,
                     timeout: float = 1.0) -> bytes | None:
    send_command(bus, can_id, data_without_crc)
    expected_cmd = data_without_crc[0]
    return receive_response(bus, can_id, timeout, expected_cmd=expected_cmd)

# ---------------------------------------------------------------------------
# Data encoding / decoding  (all big-endian)
# ---------------------------------------------------------------------------

def encode_uint16(value: int) -> bytes:
    return value.to_bytes(2, "big")


def encode_int24(value: int) -> bytes:
    if value < 0:
        value += 1 << 24
    return value.to_bytes(3, "big")


def decode_int16(data: bytes) -> int:
    return int.from_bytes(data[:2], "big", signed=True)


def decode_int32(data: bytes) -> int:
    return int.from_bytes(data[:4], "big", signed=True)


def decode_int48(data: bytes) -> int:
    val = int.from_bytes(data[:6], "big")
    if val >= (1 << 47):
        val -= 1 << 48
    return val

# ---------------------------------------------------------------------------
# Joint lookup
# ---------------------------------------------------------------------------

def get_joint_by_can_id(can_id: int):
    for j in JOINTS:
        if j[1] == can_id:
            return j
    return None

# ---------------------------------------------------------------------------
# Emergency stop
# ---------------------------------------------------------------------------

def emergency_stop_all(bus: can.Bus) -> None:
    print("\n!!! EMERGENCY STOP ALL MOTORS !!!")
    for _, can_id, *_ in JOINTS:
        send_command(bus, can_id, bytes([0xF7]))
    print("Emergency stop sent to all 6 motors.")


def _sigint_handler(_signum, _frame):
    if _bus is not None:
        emergency_stop_all(_bus)
    sys.exit(1)

# ---------------------------------------------------------------------------
# Safety helpers
# ---------------------------------------------------------------------------

def confirm_dangerous(action: str) -> bool:
    resp = input(f"  WARNING: {action}\n  Type 'YES' to confirm: ")
    return resp.strip() == "YES"


def warn_high_speed(speed: int) -> bool:
    if speed > 1000:
        print(f"  [!] Speed {speed} RPM is above 1000. Emergency stop at this speed can damage hardware.")
        return confirm_dangerous(f"Proceed with speed={speed} RPM?")
    return True

# ---------------------------------------------------------------------------
# Input helpers
# ---------------------------------------------------------------------------

def input_int(prompt: str, default: int | None = None) -> int | None:
    suffix = f" [{default}]" if default is not None else ""
    raw = input(f"  {prompt}{suffix}: ").strip()
    if raw == "":
        return default
    try:
        return int(raw)
    except ValueError:
        print("  Invalid number.")
        return None

# ---------------------------------------------------------------------------
# Command functions — each returns a human-readable result string
# ---------------------------------------------------------------------------

# ---- Read commands --------------------------------------------------------

def cmd_read_encoder_cumulative(bus: can.Bus, can_id: int) -> str:
    """0x31: Read cumulative encoder position (int48)."""
    resp = send_and_receive(bus, can_id, bytes([0x31]))
    if resp is None:
        return "No response"
    if resp[0] != 0x31 or len(resp) < 7:
        return f"Unexpected response: {resp.hex()}"
    value = decode_int48(resp[1:7])
    joint = get_joint_by_can_id(can_id)
    if joint:
        angle = value * 360.0 / (joint[2] * ENCODER_COUNTS_PER_REV)
        return f"Encoder: {value} counts  (~{angle:.2f}° at joint)"
    return f"Encoder: {value} counts"


def cmd_read_encoder_carry(bus: can.Bus, can_id: int) -> str:
    """0x30: Read encoder carry (int32 carry + uint16 value)."""
    resp = send_and_receive(bus, can_id, bytes([0x30]))
    if resp is None:
        return "No response"
    if resp[0] != 0x30 or len(resp) < 7:
        return f"Unexpected response: {resp.hex()}"
    carry = decode_int32(resp[1:5])
    value = int.from_bytes(resp[5:7], "big")
    return f"Carry: {carry}, Value: {value}"


def cmd_read_speed(bus: can.Bus, can_id: int) -> str:
    """0x32: Read motor speed in RPM (int16)."""
    resp = send_and_receive(bus, can_id, bytes([0x32]))
    if resp is None:
        return "No response"
    if resp[0] != 0x32 or len(resp) < 3:
        return f"Unexpected response: {resp.hex()}"
    speed = decode_int16(resp[1:3])
    return f"Speed: {speed} RPM"


def cmd_read_status(bus: can.Bus, can_id: int) -> str:
    """0xF1: Query motor status."""
    STATUS_MAP = {
        0: "Fail", 1: "Stopped", 2: "Accelerating",
        3: "Decelerating", 4: "Full speed", 5: "Homing", 6: "Calibrating",
    }
    resp = send_and_receive(bus, can_id, bytes([0xF1]))
    if resp is None:
        return "No response"
    if resp[0] != 0xF1 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    code = resp[1]
    return f"Status: {STATUS_MAP.get(code, f'Unknown({code})')}"


def cmd_read_enable_status(bus: can.Bus, can_id: int) -> str:
    """0x3A: Read enable status."""
    resp = send_and_receive(bus, can_id, bytes([0x3A]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3A or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Enabled" if resp[1] == 1 else "Disabled"


def cmd_read_shaft_error(bus: can.Bus, can_id: int) -> str:
    """0x39: Read shaft angle error (int32, 51200 = 360°)."""
    resp = send_and_receive(bus, can_id, bytes([0x39]))
    if resp is None:
        return "No response"
    if resp[0] != 0x39 or len(resp) < 5:
        return f"Unexpected response: {resp.hex()}"
    error = decode_int32(resp[1:5])
    degrees = error * 360.0 / 51200.0
    return f"Shaft error: {error} ({degrees:.2f}°)"


def cmd_read_io_status(bus: can.Bus, can_id: int) -> str:
    """0x34: Read IO port status."""
    resp = send_and_receive(bus, can_id, bytes([0x34]))
    if resp is None:
        return "No response"
    if resp[0] != 0x34 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    s = resp[1]
    parts = []
    parts.append(f"IN1={'HIGH' if s & 1 else 'LOW'}")
    parts.append(f"IN2={'HIGH' if s & 2 else 'LOW'}")
    parts.append(f"OUT1={'HIGH' if s & 4 else 'LOW'}")
    parts.append(f"OUT2={'HIGH' if s & 8 else 'LOW'}")
    return "IO: " + ", ".join(parts)


# ---- Motion commands ------------------------------------------------------

def cmd_enable(bus: can.Bus, can_id: int, enable: bool) -> str:
    """0xF3: Enable or disable motor."""
    val = 0x01 if enable else 0x00
    resp = send_and_receive(bus, can_id, bytes([0xF3, val]))
    if resp is None:
        return "No response"
    if resp[0] != 0xF3 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "OK — Enabled" if resp[1] == 1 else "OK — Disabled"


def cmd_absolute_move(bus: can.Bus, can_id: int, speed: int, acc: int, position: int) -> str:
    """0xF5: Absolute axis move. Data: speed(uint16) + acc(uint8) + absAxis(int24)."""
    data = bytes([0xF5]) + encode_uint16(speed) + bytes([acc]) + encode_int24(position)
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return "No response"
    if resp[0] != 0xF5 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Move accepted" if resp[1] == 2 else f"Response: {resp[1]}"


def cmd_relative_move(bus: can.Bus, can_id: int, speed: int, acc: int, rel_position: int) -> str:
    """0xF4: Relative axis move. Data: speed(uint16) + acc(uint8) + relAxis(int24)."""
    data = bytes([0xF4]) + encode_uint16(speed) + bytes([acc]) + encode_int24(rel_position)
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return "No response"
    if resp[0] != 0xF4 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Move accepted" if resp[1] == 2 else f"Response: {resp[1]}"


def cmd_speed_mode(bus: can.Bus, can_id: int, direction: int, speed: int, acc: int) -> str:
    """0xF6: Speed mode. Data: dir|speed(uint16 — bit15=dir, bits11-0=speed) + acc(uint8)."""
    dir_speed = ((direction & 1) << 15) | (speed & 0x0FFF)
    data = bytes([0xF6]) + encode_uint16(dir_speed) + bytes([acc])
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return "No response"
    if resp[0] != 0xF6 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Speed mode set" if resp[1] == 2 else f"Response: {resp[1]}"


def cmd_emergency_stop(bus: can.Bus, can_id: int) -> str:
    """0xF7: Emergency stop."""
    resp = send_and_receive(bus, can_id, bytes([0xF7]))
    if resp is None:
        return "Stop sent (no response)"
    if resp[0] != 0xF7 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Stopped" if resp[1] == 1 else f"Response: {resp[1]}"


# ---- Homing commands ------------------------------------------------------

def cmd_go_home(bus: can.Bus, can_id: int) -> str:
    """0x91: Start homing sequence."""
    resp = send_and_receive(bus, can_id, bytes([0x91]))
    if resp is None:
        return "No response"
    if resp[0] != 0x91 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Homing started" if resp[1] == 2 else f"Response: {resp[1]}"


def cmd_read_home_status(bus: can.Bus, can_id: int) -> str:
    """0x3B: Read go-home status."""
    HOME_STATUS = {0: "In progress", 1: "Success", 2: "Failed"}
    resp = send_and_receive(bus, can_id, bytes([0x3B]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3B or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    code = resp[1]
    return f"Home status: {HOME_STATUS.get(code, f'Unknown({code})')}"


def cmd_set_zero(bus: can.Bus, can_id: int) -> str:
    """0x92: Set current position as zero."""
    resp = send_and_receive(bus, can_id, bytes([0x92]))
    if resp is None:
        return "No response"
    if resp[0] != 0x92 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Zero set" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_home_params(bus: can.Bus, can_id: int,
                        home_trig: int, home_dir: int,
                        home_speed: int, end_limit: int) -> str:
    """0x90: Set home parameters."""
    data = bytes([0x90, home_trig, home_dir]) + encode_uint16(home_speed) + bytes([end_limit])
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return "No response"
    if resp[0] != 0x90 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Home params set" if resp[1] == 1 else f"Response: {resp[1]}"


# ---- Configuration commands -----------------------------------------------

def cmd_set_work_mode(bus: can.Bus, can_id: int, mode: int) -> str:
    """0x82: Set work mode (0=CR_OPEN..5=SR_vFOC)."""
    resp = send_and_receive(bus, can_id, bytes([0x82, mode]))
    if resp is None:
        return "No response"
    if resp[0] != 0x82 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Mode set" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_current(bus: can.Bus, can_id: int, ma: int) -> str:
    """0x83: Set working current in mA."""
    data = bytes([0x83]) + encode_uint16(ma)
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return "No response"
    if resp[0] != 0x83 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return f"Current set to {ma} mA" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_en_active(bus: can.Bus, can_id: int, value: int) -> str:
    """0x85: Set EN pin active (0=L, 1=H, 2=Hold)."""
    resp = send_and_receive(bus, can_id, bytes([0x85, value]))
    if resp is None:
        return "No response"
    if resp[0] != 0x85 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    labels = {0: "Low", 1: "High", 2: "Hold"}
    return f"EN set to {labels.get(value, value)}" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_protection(bus: can.Bus, can_id: int, enable: bool) -> str:
    """0x88: Set stall protection."""
    val = 1 if enable else 0
    resp = send_and_receive(bus, can_id, bytes([0x88, val]))
    if resp is None:
        return "No response"
    if resp[0] != 0x88 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Protection " + ("enabled" if enable else "disabled") if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_calibrate_encoder(bus: can.Bus, can_id: int) -> str:
    """0x80: Calibrate encoder. Motor must be unloaded!"""
    resp = send_and_receive(bus, can_id, bytes([0x80, 0x00]), timeout=30.0)
    if resp is None:
        return "No response (calibration can take up to 30s)"
    if resp[0] != 0x80 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Calibration OK" if resp[1] == 1 else f"Calibration result: {resp[1]}"


def cmd_restore_defaults(bus: can.Bus, can_id: int) -> str:
    """0x3F: Restore factory defaults."""
    resp = send_and_receive(bus, can_id, bytes([0x3F]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3F or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Defaults restored" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_restart(bus: can.Bus, can_id: int) -> str:
    """0x41: Restart motor."""
    resp = send_and_receive(bus, can_id, bytes([0x41]))
    if resp is None:
        return "Restart sent (motor may not respond)"
    return "Restart acknowledged"


# ---- Diagnostics / Protection ---------------------------------------------

def cmd_read_protection_status(bus: can.Bus, can_id: int) -> str:
    """0x3E: Read stall protection status."""
    resp = send_and_receive(bus, can_id, bytes([0x3E]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3E or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "PROTECTED (stalled)" if resp[1] == 1 else "Not protected"


def cmd_release_protection(bus: can.Bus, can_id: int) -> str:
    """0x3D: Release stall protection."""
    resp = send_and_receive(bus, can_id, bytes([0x3D]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3D or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Protection released" if resp[1] == 1 else f"Response: {resp[1]}"


# ---------------------------------------------------------------------------
# Run a command for all selected motors
# ---------------------------------------------------------------------------

def run_for_selected(bus: can.Bus, func, *args) -> None:
    if not _selected_can_ids:
        print("  No motor selected. Use option [0] first.")
        return
    for can_id in _selected_can_ids:
        joint = get_joint_by_can_id(can_id)
        label = joint[0] if joint else f"0x{can_id:02X}"
        result = func(bus, can_id, *args)
        print(f"  [{label}] {result}")


def confirm_action_all() -> bool:
    """If multiple motors are selected, confirm before sending action commands."""
    if len(_selected_can_ids) > 1:
        return confirm_dangerous("This will act on ALL selected motors.")
    return True

# ---------------------------------------------------------------------------
# Menus
# ---------------------------------------------------------------------------

def menu_select_motor() -> None:
    global _selected_can_ids, _selected_label
    print("\n  --- Select Motor ---")
    for i, (name, can_id, *_rest) in enumerate(JOINTS, 1):
        print(f"  [{i}] {name:12s} (0x{can_id:02X})")
    print("  [a] All motors")
    print("  [b] Back")
    choice = input("  > ").strip().lower()
    if choice == "b":
        return
    if choice == "a":
        _selected_can_ids = [j[1] for j in JOINTS]
        _selected_label = "ALL motors"
        print(f"  Selected: {_selected_label}")
        return
    try:
        idx = int(choice) - 1
        if 0 <= idx < len(JOINTS):
            name, can_id, *_ = JOINTS[idx]
            _selected_can_ids = [can_id]
            _selected_label = f"{name} (0x{can_id:02X})"
            print(f"  Selected: {_selected_label}")
        else:
            print("  Invalid choice.")
    except ValueError:
        print("  Invalid choice.")


def menu_read(bus: can.Bus) -> None:
    while True:
        print("\n  --- Read / Status ---")
        print("  [1] Read encoder (cumulative, 0x31)")
        print("  [2] Read encoder (carry, 0x30)")
        print("  [3] Read speed RPM (0x32)")
        print("  [4] Read motor status (0xF1)")
        print("  [5] Read enable status (0x3A)")
        print("  [6] Read shaft angle error (0x39)")
        print("  [7] Read IO port status (0x34)")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        cmds = {
            "1": cmd_read_encoder_cumulative,
            "2": cmd_read_encoder_carry,
            "3": cmd_read_speed,
            "4": cmd_read_status,
            "5": cmd_read_enable_status,
            "6": cmd_read_shaft_error,
            "7": cmd_read_io_status,
        }
        if choice in cmds:
            run_for_selected(bus, cmds[choice])
        else:
            print("  Invalid choice.")


def menu_motion(bus: can.Bus) -> None:
    while True:
        print("\n  --- Motion Commands ---")
        print("  [1] Absolute axis move (0xF5)")
        print("  [2] Relative axis move (0xF4)")
        print("  [3] Speed mode (0xF6)")
        print("  [4] Emergency stop (0xF7)")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice == "1":
            if not confirm_action_all():
                continue
            speed = input_int("Speed (RPM)", DEFAULT_SPEED)
            if speed is None:
                continue
            if not warn_high_speed(speed):
                continue
            acc = input_int("Acceleration (0-255)", DEFAULT_ACC)
            if acc is None:
                continue
            position = input_int("Absolute position (encoder counts)")
            if position is None:
                continue
            run_for_selected(bus, cmd_absolute_move, speed, acc, position)
        elif choice == "2":
            if not confirm_action_all():
                continue
            speed = input_int("Speed (RPM)", DEFAULT_SPEED)
            if speed is None:
                continue
            if not warn_high_speed(speed):
                continue
            acc = input_int("Acceleration (0-255)", DEFAULT_ACC)
            if acc is None:
                continue
            rel = input_int("Relative position (encoder counts, negative=reverse)")
            if rel is None:
                continue
            run_for_selected(bus, cmd_relative_move, speed, acc, rel)
        elif choice == "3":
            if not confirm_action_all():
                continue
            print("  Direction: 0=CW, 1=CCW")
            direction = input_int("Direction", 0)
            if direction is None or direction not in (0, 1):
                print("  Must be 0 or 1.")
                continue
            speed = input_int("Speed (0-4095)", 300)
            if speed is None:
                continue
            if not warn_high_speed(speed):
                continue
            acc = input_int("Acceleration (0-255)", DEFAULT_ACC)
            if acc is None:
                continue
            run_for_selected(bus, cmd_speed_mode, direction, speed, acc)
        elif choice == "4":
            emergency_stop_all(bus)
        else:
            print("  Invalid choice.")


def menu_homing(bus: can.Bus) -> None:
    while True:
        print("\n  --- Homing ---")
        print("  [1] Go home (0x91)")
        print("  [2] Read go-home status (0x3B)")
        print("  [3] Set current position as zero (0x92)")
        print("  [4] Set home parameters (0x90)")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice == "1":
            if not confirm_action_all():
                continue
            run_for_selected(bus, cmd_go_home)
        elif choice == "2":
            run_for_selected(bus, cmd_read_home_status)
        elif choice == "3":
            if not confirm_action_all():
                continue
            run_for_selected(bus, cmd_set_zero)
        elif choice == "4":
            if not confirm_action_all():
                continue
            print("  homeTrig: 0=low level trigger, 1=high level trigger")
            trig = input_int("homeTrig", 0)
            if trig is None:
                continue
            print("  homeDir: 0=CW, 1=CCW")
            hdir = input_int("homeDir", 0)
            if hdir is None:
                continue
            hspeed = input_int("homeSpeed (RPM)", 100)
            if hspeed is None:
                continue
            print("  EndLimit: 0=no limit, 1=has end limit")
            elimit = input_int("EndLimit", 0)
            if elimit is None:
                continue
            run_for_selected(bus, cmd_set_home_params, trig, hdir, hspeed, elimit)
        else:
            print("  Invalid choice.")


def menu_enable(bus: can.Bus) -> None:
    while True:
        print("\n  --- Enable / Disable ---")
        print("  [1] Enable motor")
        print("  [2] Disable motor")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice == "1":
            if not confirm_action_all():
                continue
            run_for_selected(bus, cmd_enable, True)
        elif choice == "2":
            if not confirm_action_all():
                continue
            run_for_selected(bus, cmd_enable, False)
        else:
            print("  Invalid choice.")


def menu_config(bus: can.Bus) -> None:
    while True:
        print("\n  --- Configuration ---")
        print("  [1] Set work mode (0x82)")
        print("  [2] Set working current (0x83)")
        print("  [3] Set EN pin active (0x85)")
        print("  [4] Set stall protection (0x88)")
        print("  [5] Calibrate encoder (0x80)  [DANGEROUS]")
        print("  [6] Restore defaults (0x3F)   [DANGEROUS]")
        print("  [7] Restart motor (0x41)")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice == "1":
            if not confirm_action_all():
                continue
            print("  Modes: 0=CR_OPEN, 1=CR_CLOSE, 2=CR_vFOC, 3=SR_OPEN, 4=SR_CLOSE, 5=SR_vFOC")
            mode = input_int("Mode", 5)
            if mode is None or not (0 <= mode <= 5):
                print("  Must be 0-5.")
                continue
            run_for_selected(bus, cmd_set_work_mode, mode)
        elif choice == "2":
            if not confirm_action_all():
                continue
            ma = input_int("Current (mA)")
            if ma is None:
                continue
            run_for_selected(bus, cmd_set_current, ma)
        elif choice == "3":
            if not confirm_action_all():
                continue
            print("  0=Low, 1=High, 2=Hold (recommended for serial mode)")
            val = input_int("EN active", 2)
            if val is None or val not in (0, 1, 2):
                print("  Must be 0, 1, or 2.")
                continue
            run_for_selected(bus, cmd_set_en_active, val)
        elif choice == "4":
            if not confirm_action_all():
                continue
            print("  0=disable, 1=enable")
            val = input_int("Stall protection", 1)
            if val is None or val not in (0, 1):
                print("  Must be 0 or 1.")
                continue
            run_for_selected(bus, cmd_set_protection, bool(val))
        elif choice == "5":
            if not confirm_dangerous("Calibrate encoder. Motor must be UNLOADED (no belt/gearbox). This rotates the motor."):
                continue
            run_for_selected(bus, cmd_calibrate_encoder)
        elif choice == "6":
            if not confirm_dangerous("Restore factory defaults. Motor will need recalibration."):
                continue
            run_for_selected(bus, cmd_restore_defaults)
        elif choice == "7":
            if not confirm_action_all():
                continue
            run_for_selected(bus, cmd_restart)
        else:
            print("  Invalid choice.")


def menu_diagnostics(bus: can.Bus) -> None:
    while True:
        print("\n  --- Diagnostics / Protection ---")
        print("  [1] Read protection status (0x3E)")
        print("  [2] Release stall protection (0x3D)")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice == "1":
            run_for_selected(bus, cmd_read_protection_status)
        elif choice == "2":
            if not confirm_action_all():
                continue
            run_for_selected(bus, cmd_release_protection)
        else:
            print("  Invalid choice.")

# ---------------------------------------------------------------------------
# Main menu
# ---------------------------------------------------------------------------

def main_menu(bus: can.Bus) -> None:
    while True:
        print(f"\n{'=' * 42}")
        print(f"  Arctos Motor Control")
        print(f"  Selected: {_selected_label}")
        print(f"{'=' * 42}")
        print("  [0] Select motor")
        print("  [1] Read / Status")
        print("  [2] Motion commands")
        print("  [3] Homing")
        print("  [4] Enable / Disable")
        print("  [5] Configuration")
        print("  [6] Diagnostics / Protection")
        print("  [7] *** EMERGENCY STOP ALL ***")
        print("  [q] Quit")
        choice = input("> ").strip().lower()
        if choice == "q":
            return
        elif choice == "0":
            menu_select_motor()
        elif choice == "1":
            menu_read(bus)
        elif choice == "2":
            menu_motion(bus)
        elif choice == "3":
            menu_homing(bus)
        elif choice == "4":
            menu_enable(bus)
        elif choice == "5":
            menu_config(bus)
        elif choice == "6":
            menu_diagnostics(bus)
        elif choice == "7":
            emergency_stop_all(bus)
        else:
            print("  Invalid choice.")

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Arctos MKS motor control — interactive CLI")
    parser.add_argument("--channel", default=CAN_CHANNEL,
                        help=f"CAN channel (default: {CAN_CHANNEL})")
    parser.add_argument("--bitrate", type=int, default=CAN_BITRATE,
                        help=f"CAN bitrate (default: {CAN_BITRATE})")
    parser.add_argument("--interface", default="socketcan",
                        help="python-can interface type (default: socketcan)")
    args = parser.parse_args()

    global _bus
    try:
        _bus = can.Bus(channel=args.channel, interface=args.interface, bitrate=args.bitrate)
    except Exception as e:
        print(f"Failed to open CAN bus: {e}")
        sys.exit(1)

    signal.signal(signal.SIGINT, _sigint_handler)
    print(f"Connected to {args.channel} ({args.interface}) at {args.bitrate} bps")

    try:
        main_menu(_bus)
    finally:
        _bus.shutdown()
        print("CAN bus closed.")


if __name__ == "__main__":
    main()
