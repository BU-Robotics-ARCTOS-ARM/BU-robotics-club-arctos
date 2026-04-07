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

# Allow running from project root: `python scripts/motor_control.py`
sys.path.insert(0, ".")
from arctos.can_interface import CANInterface
from arctos.config import (
    CAN_BITRATE,
    CAN_CHANNEL,
    DEFAULT_ACC,
    DEFAULT_SPEED,
    ENCODER_COUNTS_PER_REV,
    JOINTS,
)
from arctos.motor_driver import MotorDriver

# ---------------------------------------------------------------------------
# Global state
# ---------------------------------------------------------------------------
_can: CANInterface | None = None
_drivers: dict[int, MotorDriver] = {}
_selected_can_ids: list[int] = []
_selected_label: str = "None"

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


def emergency_stop_all() -> None:
    print("\n!!! EMERGENCY STOP ALL MOTORS !!!")
    for _, can_id, *_ in JOINTS:
        _drivers[can_id].emergency_stop()
    print("Emergency stop sent to all 6 motors.")


def _sigint_handler(_signum, _frame):
    if _can is not None:
        emergency_stop_all()
    sys.exit(1)


# ---------------------------------------------------------------------------
# Safety helpers
# ---------------------------------------------------------------------------


def confirm_dangerous(action: str) -> bool:
    resp = input(f"  WARNING: {action}\n  Type 'YES' to confirm: ")
    return resp.strip() == "YES"


def warn_high_speed(speed: int) -> bool:
    if speed > 1000:
        print(
            f"  [!] Speed {speed} RPM is above 1000. Emergency stop at this speed can damage hardware."
        )
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


def cmd_read_encoder_cumulative(driver: MotorDriver) -> str:
    """0x31: Read cumulative encoder position (int48)."""
    value = driver.read_encoder()
    if value is None:
        return "No response"
    joint = get_joint_by_can_id(driver.can_id)
    if joint:
        angle = value * 360.0 / (joint[2] * ENCODER_COUNTS_PER_REV)
        return f"Encoder: {value} counts  (~{angle:.2f}° at joint)"
    return f"Encoder: {value} counts"


def cmd_read_encoder_carry(driver: MotorDriver) -> str:
    """0x30: Read encoder carry (int32 carry + uint16 value)."""
    resp = driver.raw_command(bytes([0x30]))
    if resp is None:
        return "No response"
    if resp[0] != 0x30 or len(resp) < 7:
        return f"Unexpected response: {resp.hex()}"
    carry = MotorDriver._decode_int32(resp[1:5])
    value = int.from_bytes(resp[5:7], "big")
    return f"Carry: {carry}, Value: {value}"


def cmd_read_speed(driver: MotorDriver) -> str:
    """0x32: Read motor speed in RPM (int16)."""
    speed = driver.read_speed()
    if speed is None:
        return "No response"
    return f"Speed: {speed} RPM"


def cmd_read_status(driver: MotorDriver) -> str:
    """0xF1: Query motor status."""
    STATUS_MAP = {
        0: "Fail",
        1: "Stopped",
        2: "Accelerating",
        3: "Decelerating",
        4: "Full speed",
        5: "Homing",
        6: "Calibrating",
    }
    code = driver.read_status()
    if code is None:
        return "No response"
    return f"Status: {STATUS_MAP.get(code, f'Unknown({code})')}"


def cmd_read_enable_status(driver: MotorDriver) -> str:
    """0x3A: Read enable status."""
    resp = driver.raw_command(bytes([0x3A]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3A or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Enabled" if resp[1] == 1 else "Disabled"


def cmd_read_shaft_error(driver: MotorDriver) -> str:
    """0x39: Read shaft angle error (int32, 51200 = 360°)."""
    resp = driver.raw_command(bytes([0x39]))
    if resp is None:
        return "No response"
    if resp[0] != 0x39 or len(resp) < 5:
        return f"Unexpected response: {resp.hex()}"
    error = MotorDriver._decode_int32(resp[1:5])
    degrees = error * 360.0 / 51200.0
    return f"Shaft error: {error} ({degrees:.2f}°)"


def cmd_read_io_status(driver: MotorDriver) -> str:
    """0x34: Read IO port status."""
    resp = driver.raw_command(bytes([0x34]))
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


def cmd_enable(driver: MotorDriver, enable: bool) -> str:
    """0xF3: Enable or disable motor."""
    ok = driver.enable(enable)
    if not ok:
        return "No response"
    return "OK — Enabled" if enable else "OK — Disabled"


def cmd_absolute_move(driver: MotorDriver, speed: int, acc: int, position: int) -> str:
    """0xF5: Absolute axis move."""
    ok = driver.move_to_axis(position, speed, acc)
    if not ok:
        return "No response"
    return "Move accepted"


def cmd_relative_move(
    driver: MotorDriver, speed: int, acc: int, rel_position: int
) -> str:
    """0xF4: Relative axis move. Data: speed(uint16) + acc(uint8) + relAxis(int24)."""
    data = (
        bytes([0xF4])
        + MotorDriver._encode_uint16(speed)
        + bytes([acc])
        + MotorDriver._encode_int24(rel_position)
    )
    resp = driver.raw_command(data)
    if resp is None:
        return "No response"
    if resp[0] != 0xF4 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Move accepted" if resp[1] == 2 else f"Response: {resp[1]}"


def cmd_speed_mode(driver: MotorDriver, direction: int, speed: int, acc: int) -> str:
    """0xF6: Speed mode."""
    ok = driver.set_speed(direction, speed, acc)
    if not ok:
        return "No response"
    return "Speed mode set"


def cmd_emergency_stop(driver: MotorDriver) -> str:
    """0xF7: Emergency stop."""
    ok = driver.emergency_stop()
    if not ok:
        return "Stop sent (no response)"
    return "Stopped"


# ---- Homing commands ------------------------------------------------------


def cmd_go_home(driver: MotorDriver) -> str:
    """0x91: Start homing sequence."""
    resp = driver.raw_command(bytes([0x91]))
    if resp is None:
        return "No response"
    if resp[0] != 0x91 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Homing started" if resp[1] == 2 else f"Response: {resp[1]}"


def cmd_read_home_status(driver: MotorDriver) -> str:
    """0x3B: Read go-home status."""
    HOME_STATUS = {0: "In progress", 1: "Success", 2: "Failed"}
    resp = driver.raw_command(bytes([0x3B]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3B or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    code = resp[1]
    return f"Home status: {HOME_STATUS.get(code, f'Unknown({code})')}"


def cmd_set_zero(driver: MotorDriver) -> str:
    """0x92: Set current position as zero."""
    resp = driver.raw_command(bytes([0x92]))
    if resp is None:
        return "No response"
    if resp[0] != 0x92 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Zero set" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_home_params(
    driver: MotorDriver, home_trig: int, home_dir: int, home_speed: int, end_limit: int
) -> str:
    """0x90: Set home parameters."""
    data = (
        bytes([0x90, home_trig, home_dir])
        + MotorDriver._encode_uint16(home_speed)
        + bytes([end_limit])
    )
    resp = driver.raw_command(data)
    if resp is None:
        return "No response"
    if resp[0] != 0x90 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Home params set" if resp[1] == 1 else f"Response: {resp[1]}"


# ---- Configuration commands -----------------------------------------------


def cmd_set_work_mode(driver: MotorDriver, mode: int) -> str:
    """0x82: Set work mode (0=CR_OPEN..5=SR_vFOC)."""
    resp = driver.raw_command(bytes([0x82, mode]))
    if resp is None:
        return "No response"
    if resp[0] != 0x82 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Mode set" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_current(driver: MotorDriver, ma: int) -> str:
    """0x83: Set working current in mA."""
    data = bytes([0x83]) + MotorDriver._encode_uint16(ma)
    resp = driver.raw_command(data)
    if resp is None:
        return "No response"
    if resp[0] != 0x83 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return f"Current set to {ma} mA" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_set_en_active(driver: MotorDriver, value: int) -> str:
    """0x85: Set EN pin active (0=L, 1=H, 2=Hold)."""
    resp = driver.raw_command(bytes([0x85, value]))
    if resp is None:
        return "No response"
    if resp[0] != 0x85 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    labels = {0: "Low", 1: "High", 2: "Hold"}
    return (
        f"EN set to {labels.get(value, value)}"
        if resp[1] == 1
        else f"Response: {resp[1]}"
    )


def cmd_set_protection(driver: MotorDriver, enable: bool) -> str:
    """0x88: Set stall protection."""
    val = 1 if enable else 0
    resp = driver.raw_command(bytes([0x88, val]))
    if resp is None:
        return "No response"
    if resp[0] != 0x88 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return (
        "Protection " + ("enabled" if enable else "disabled")
        if resp[1] == 1
        else f"Response: {resp[1]}"
    )


def cmd_calibrate_encoder(driver: MotorDriver) -> str:
    """0x80: Calibrate encoder. Motor must be unloaded!"""
    resp = driver.raw_command(bytes([0x80, 0x00]), timeout=30.0)
    if resp is None:
        return "No response (calibration can take up to 30s)"
    if resp[0] != 0x80 or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Calibration OK" if resp[1] == 1 else f"Calibration result: {resp[1]}"


def cmd_restore_defaults(driver: MotorDriver) -> str:
    """0x3F: Restore factory defaults."""
    resp = driver.raw_command(bytes([0x3F]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3F or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Defaults restored" if resp[1] == 1 else f"Response: {resp[1]}"


def cmd_restart(driver: MotorDriver) -> str:
    """0x41: Restart motor."""
    resp = driver.raw_command(bytes([0x41]))
    if resp is None:
        return "Restart sent (motor may not respond)"
    return "Restart acknowledged"


# ---- Diagnostics / Protection ---------------------------------------------


def cmd_read_protection_status(driver: MotorDriver) -> str:
    """0x3E: Read stall protection status."""
    resp = driver.raw_command(bytes([0x3E]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3E or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "PROTECTED (stalled)" if resp[1] == 1 else "Not protected"


def cmd_release_protection(driver: MotorDriver) -> str:
    """0x3D: Release stall protection."""
    resp = driver.raw_command(bytes([0x3D]))
    if resp is None:
        return "No response"
    if resp[0] != 0x3D or len(resp) < 2:
        return f"Unexpected response: {resp.hex()}"
    return "Protection released" if resp[1] == 1 else f"Response: {resp[1]}"


# ---------------------------------------------------------------------------
# Run a command for all selected motors
# ---------------------------------------------------------------------------


def run_for_selected(func, *args) -> None:
    if not _selected_can_ids:
        print("  No motor selected. Use option [0] first.")
        return
    for can_id in _selected_can_ids:
        driver = _drivers[can_id]
        joint = get_joint_by_can_id(can_id)
        label = joint[0] if joint else f"0x{can_id:02X}"
        result = func(driver, *args)
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


def menu_read() -> None:
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
            run_for_selected(cmds[choice])
        else:
            print("  Invalid choice.")


def menu_motion() -> None:
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
            run_for_selected(cmd_absolute_move, speed, acc, position)
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
            run_for_selected(cmd_relative_move, speed, acc, rel)
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
            run_for_selected(cmd_speed_mode, direction, speed, acc)
        elif choice == "4":
            emergency_stop_all()
        else:
            print("  Invalid choice.")


def menu_homing() -> None:
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
            run_for_selected(cmd_go_home)
        elif choice == "2":
            run_for_selected(cmd_read_home_status)
        elif choice == "3":
            if not confirm_action_all():
                continue
            run_for_selected(cmd_set_zero)
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
            run_for_selected(cmd_set_home_params, trig, hdir, hspeed, elimit)
        else:
            print("  Invalid choice.")


def menu_enable() -> None:
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
            run_for_selected(cmd_enable, True)
        elif choice == "2":
            if not confirm_action_all():
                continue
            run_for_selected(cmd_enable, False)
        else:
            print("  Invalid choice.")


def menu_config() -> None:
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
            print(
                "  Modes: 0=CR_OPEN, 1=CR_CLOSE, 2=CR_vFOC, 3=SR_OPEN, 4=SR_CLOSE, 5=SR_vFOC"
            )
            mode = input_int("Mode", 5)
            if mode is None or not (0 <= mode <= 5):
                print("  Must be 0-5.")
                continue
            run_for_selected(cmd_set_work_mode, mode)
        elif choice == "2":
            if not confirm_action_all():
                continue
            ma = input_int("Current (mA)")
            if ma is None:
                continue
            run_for_selected(cmd_set_current, ma)
        elif choice == "3":
            if not confirm_action_all():
                continue
            print("  0=Low, 1=High, 2=Hold (recommended for serial mode)")
            val = input_int("EN active", 2)
            if val is None or val not in (0, 1, 2):
                print("  Must be 0, 1, or 2.")
                continue
            run_for_selected(cmd_set_en_active, val)
        elif choice == "4":
            if not confirm_action_all():
                continue
            print("  0=disable, 1=enable")
            val = input_int("Stall protection", 1)
            if val is None or val not in (0, 1):
                print("  Must be 0 or 1.")
                continue
            run_for_selected(cmd_set_protection, bool(val))
        elif choice == "5":
            if not confirm_dangerous(
                "Calibrate encoder. Motor must be UNLOADED (no belt/gearbox). This rotates the motor."
            ):
                continue
            run_for_selected(cmd_calibrate_encoder)
        elif choice == "6":
            if not confirm_dangerous(
                "Restore factory defaults. Motor will need recalibration."
            ):
                continue
            run_for_selected(cmd_restore_defaults)
        elif choice == "7":
            if not confirm_action_all():
                continue
            run_for_selected(cmd_restart)
        else:
            print("  Invalid choice.")


def menu_diagnostics() -> None:
    while True:
        print("\n  --- Diagnostics / Protection ---")
        print("  [1] Read protection status (0x3E)")
        print("  [2] Release stall protection (0x3D)")
        print("  [b] Back")
        choice = input("  > ").strip().lower()
        if choice == "b":
            return
        if choice == "1":
            run_for_selected(cmd_read_protection_status)
        elif choice == "2":
            if not confirm_action_all():
                continue
            run_for_selected(cmd_release_protection)
        else:
            print("  Invalid choice.")


# ---------------------------------------------------------------------------
# Main menu
# ---------------------------------------------------------------------------


def main_menu() -> None:
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
            menu_read()
        elif choice == "2":
            menu_motion()
        elif choice == "3":
            menu_homing()
        elif choice == "4":
            menu_enable()
        elif choice == "5":
            menu_config()
        elif choice == "6":
            menu_diagnostics()
        elif choice == "7":
            emergency_stop_all()
        else:
            print("  Invalid choice.")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(
        description="Arctos MKS motor control — interactive CLI"
    )
    parser.add_argument(
        "--channel", default=CAN_CHANNEL, help=f"CAN channel (default: {CAN_CHANNEL})"
    )
    parser.add_argument(
        "--bitrate",
        type=int,
        default=CAN_BITRATE,
        help=f"CAN bitrate (default: {CAN_BITRATE})",
    )
    parser.add_argument(
        "--interface",
        default="socketcan",
        help="python-can interface type (default: socketcan)",
    )
    args = parser.parse_args()

    global _can, _drivers
    try:
        _can = CANInterface(
            channel=args.channel,
            bitrate=args.bitrate,
            interface=args.interface,
        )
    except Exception as e:
        print(f"Failed to open CAN bus: {e}")
        sys.exit(1)

    for _, can_id, *_ in JOINTS:
        _drivers[can_id] = MotorDriver(_can, can_id)

    signal.signal(signal.SIGINT, _sigint_handler)
    print(f"Connected to {args.channel} ({args.interface}) at {args.bitrate} bps")

    try:
        main_menu()
    finally:
        _can.close()
        print("CAN bus closed.")


if __name__ == "__main__":
    main()
