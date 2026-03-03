#!/usr/bin/env python3
"""
MKS SERVO42D/57D CAN Bus Test Script for Fedora Linux + CANable
================================================================
Controls an MKS SERVO42/57D closed-loop stepper motor over CAN bus
using a CANable (slcan/gs_usb) adapter.

Prerequisites (Fedora):
    sudo dnf install python3-pip can-utils
    pip3 install python-can

Hardware wiring:
    CANable CANH  →  Motor CANH
    CANable CANL  →  Motor CANL
    CANable GND   →  Motor GND  (shared ground recommended)
    Motor powered by 12-24V supply

Motor setup (via OLED menu or this script):
    Mode    → SR_vFOC  (bus FOC mode)
    CanRate → 500K     (default)
    CanID   → 01       (default)

Usage:
    1. Plug in CANable, then run:
         sudo python3 mks_servo_can_test.py
    2. Follow the interactive menu.
"""

import os
import sys
import time
import struct
import subprocess
import argparse

try:
    import can
except ImportError:
    print("ERROR: python-can not installed. Run:")
    print("  pip3 install python-can")
    sys.exit(1)


# ─── Configuration ──────────────────────────────────────────────────────────
DEFAULT_CAN_ID    = 0x01       # Slave CAN ID (set on motor OLED: CanID)
DEFAULT_BITRATE   = 500000     # 500K (set on motor OLED: CanRate)
DEFAULT_INTERFACE = "socketcan" # Linux SocketCAN
DEFAULT_CHANNEL   = "can0"
RECV_TIMEOUT      = 1.0        # seconds


# ─── CRC Calculation ────────────────────────────────────────────────────────
def calc_crc(can_id, data_bytes):
    """CHECKSUM 8-bit: (CAN_ID + sum of all data bytes) & 0xFF"""
    return (can_id + sum(data_bytes)) & 0xFF


# ─── CAN Send / Receive ────────────────────────────────────────────────────
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


# ─── Command Helpers ────────────────────────────────────────────────────────

def read_encoder(bus, can_id):
    """Read cumulative multi-turn encoder value (cmd 0x31)."""
    resp = send_cmd(bus, can_id, [0x31])
    if resp and resp[0] == 0x31:
        # Bytes 1-6 = int48 value, byte 7 = CRC
        raw = bytes(resp[1:7])
        value = int.from_bytes(raw, 'big', signed=True)
        turns = value / 16384.0
        print(f"  → Encoder value: {value}  ({turns:.3f} turns)")
        return value
    return None


def read_speed(bus, can_id):
    """Read real-time motor speed in RPM (cmd 0x32)."""
    resp = send_cmd(bus, can_id, [0x32])
    if resp and resp[0] == 0x32:
        rpm = int.from_bytes(resp[1:3], 'big', signed=True)
        print(f"  → Speed: {rpm} RPM")
        return rpm
    return None


def read_enable_status(bus, can_id):
    """Read motor enable status (cmd 0x3A)."""
    resp = send_cmd(bus, can_id, [0x3A])
    if resp and resp[0] == 0x3A:
        enabled = resp[1]
        print(f"  → Motor {'ENABLED' if enabled else 'DISABLED'}")
        return enabled
    return None


def read_version(bus, can_id):
    """Read hardware/firmware version (cmd 0x40)."""
    resp = send_cmd(bus, can_id, [0x40])
    if resp and resp[0] == 0x40:
        cal = (resp[1] >> 4) & 0x0F
        hw  = resp[1] & 0x0F
        fw  = resp[2:4]
        hw_names = {1:"S42D_485", 2:"S42D_CAN", 3:"S57D_485", 4:"S57D_CAN",
                    5:"S28D_RS485", 6:"S28D_CAN", 7:"S35D_RS485", 8:"S35D_CAN"}
        print(f"  → Calibrated: {'Yes' if cal else 'No'}")
        print(f"  → Hardware:   {hw_names.get(hw, f'Unknown({hw})')}")
        print(f"  → Firmware:   {fw[0]}.{fw[1]}")
        return True
    return None


def set_work_mode(bus, can_id, mode=0x05):
    """Set working mode (cmd 0x82). Default: 0x05 = SR_vFOC (Bus FOC)."""
    modes = {0:"CR_OPEN", 1:"CR_CLOSE", 2:"CR_vFOC",
             3:"SR_OPEN", 4:"SR_CLOSE", 5:"SR_vFOC"}
    resp = send_cmd(bus, can_id, [0x82, mode])
    if resp and resp[0] == 0x82:
        ok = resp[1] == 1
        print(f"  → Set mode {modes.get(mode, '?')}: {'OK' if ok else 'FAIL'}")
        return ok
    return False


def set_enable(bus, can_id, enable=True):
    """Enable/disable motor (cmd 0xF3). Only valid in bus mode."""
    resp = send_cmd(bus, can_id, [0xF3, 0x01 if enable else 0x00])
    if resp and resp[0] == 0xF3:
        ok = resp[1] == 1
        print(f"  → Motor {'enabled' if enable else 'disabled'}: {'OK' if ok else 'FAIL'}")
        return ok
    return False


def run_speed(bus, can_id, direction=0, speed=300, acc=2):
    """
    Speed mode continuous run (cmd 0xF6).
    direction: 0=CCW, 1=CW
    speed:     0-3000 RPM
    acc:       0-255 (0=instant, higher=slower accel)
    """
    # Byte2: bit7=dir, bits 3-0 + Byte3 = speed (12-bit)
    byte2 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    byte3 = speed & 0xFF
    resp = send_cmd(bus, can_id, [0xF6, byte2, byte3, acc])
    if resp and resp[0] == 0xF6:
        status = resp[1]
        status_msg = {0: "FAIL", 1: "Running", 2: "Completed"}
        print(f"  → Speed mode: {status_msg.get(status, f'Unknown({status})')}")
        return status
    return None


def stop_speed(bus, can_id, acc=2):
    """Stop in speed mode (cmd 0xF6 with speed=0). acc=0 for instant stop."""
    byte2 = 0x00
    byte3 = 0x00
    resp = send_cmd(bus, can_id, [0xF6, byte2, byte3, acc])
    if resp and resp[0] == 0xF6:
        status = resp[1]
        print(f"  → Stop: status={status}")
        return status
    return None


def run_relative_pulses(bus, can_id, direction=0, speed=300, acc=2, pulses=3200):
    """
    Position mode - relative pulse movement (cmd 0xFD).
    direction: 0=CCW, 1=CW
    speed:     0-3000 RPM
    acc:       0-255
    pulses:    number of microstep pulses (3200 = 1 turn at 16 microsteps)
    """
    byte2 = ((direction & 1) << 7) | ((speed >> 8) & 0x0F)
    byte3 = speed & 0xFF
    p = pulses & 0xFFFFFF
    byte5 = (p >> 16) & 0xFF
    byte6 = (p >> 8) & 0xFF
    byte7 = p & 0xFF
    resp = send_cmd(bus, can_id, [0xFD, byte2, byte3, acc, byte5, byte6, byte7])
    if resp and resp[0] == 0xFD:
        status = resp[1]
        status_msg = {0: "FAIL", 1: "Running", 2: "Completed", 3: "Limit stop"}
        print(f"  → Relative pulse move: {status_msg.get(status, f'Unknown({status})')}")
        return status
    return None


def emergency_stop(bus, can_id):
    """Emergency stop (cmd 0xF7)."""
    resp = send_cmd(bus, can_id, [0xF7])
    if resp and resp[0] == 0xF7:
        ok = resp[1] == 1
        print(f"  → Emergency stop: {'OK' if ok else 'FAIL'}")
        return ok
    return False


def set_current_position_zero(bus, can_id):
    """Set current position as zero point (cmd 0x92)."""
    resp = send_cmd(bus, can_id, [0x92])
    if resp and resp[0] == 0x92:
        ok = resp[1] == 1
        print(f"  → Set zero point: {'OK' if ok else 'FAIL'}")
        return ok
    return False


def read_motor_status(bus, can_id):
    """Read motor running status (cmd 0xF1)."""
    resp = send_cmd(bus, can_id, [0xF1])
    if resp and resp[0] == 0xF1:
        status = resp[1]
        names = {0: "Query fail", 1: "Stopped", 2: "Accelerating",
                 3: "Decelerating", 4: "Full speed", 5: "Homing", 6: "Calibrating"}
        print(f"  → Motor status: {names.get(status, f'Unknown({status})')}")
        return status
    return None


# ─── CAN Interface Setup ───────────────────────────────────────────────────

def find_canable_device():
    """Try to find a CANable device (ttyACMx for slcan, or gs_usb for candleLight)."""
    # Check if can0 is already up
    result = subprocess.run(["ip", "link", "show", "can0"],
                            capture_output=True, text=True)
    if result.returncode == 0 and "UP" in result.stdout:
        print("[✓] can0 interface already UP")
        return "socketcan", "can0"

    # Check for gs_usb device (candleLight firmware - most common for CANable)
    result = subprocess.run(["ip", "link", "show", "can0"],
                            capture_output=True, text=True)
    if result.returncode == 0:
        print("[i] can0 exists but is DOWN, bringing it up...")
        return "socketcan", "can0"

    # Check for slcan device
    import glob
    acm_devices = glob.glob("/dev/ttyACM*")
    if acm_devices:
        print(f"[i] Found serial device: {acm_devices[0]} (slcan mode)")
        return "slcan", acm_devices[0]

    return None, None


def setup_can_interface(bitrate=DEFAULT_BITRATE):
    """Set up the CAN interface. Returns (interface, channel) tuple."""
    iface, channel = find_canable_device()

    if iface is None:
        print("\n[!] No CANable device found!")
        print("    Make sure your CANable is plugged in.")
        print("    For candleLight firmware: it should show up as a network device.")
        print("    For slcan firmware: it should show up as /dev/ttyACM*")
        print("\n    To set up manually:")
        print(f"      sudo ip link set can0 type can bitrate {bitrate}")
        print(f"      sudo ip link set can0 up")
        sys.exit(1)

    if iface == "socketcan":
        # Bring up socketcan interface
        print(f"[*] Setting up {channel} at {bitrate} bps...")
        subprocess.run(["ip", "link", "set", channel, "down"],
                       capture_output=True)
        result = subprocess.run(
            ["ip", "link", "set", channel, "type", "can", "bitrate", str(bitrate)],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"[!] Failed to set bitrate: {result.stderr}")
            sys.exit(1)
        result = subprocess.run(
            ["ip", "link", "set", channel, "up"],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"[!] Failed to bring up {channel}: {result.stderr}")
            sys.exit(1)
        print(f"[✓] {channel} is UP at {bitrate} bps")
        return "socketcan", channel

    elif iface == "slcan":
        print(f"[*] Using slcan on {channel}")
        return "slcan", channel

    return iface, channel


# ─── Interactive Menu ───────────────────────────────────────────────────────

def print_menu():
    print("""
╔══════════════════════════════════════════════════════════════╗
║          MKS SERVO42D/57D CAN Test Utility                  ║
╠══════════════════════════════════════════════════════════════╣
║  SETUP                                                      ║
║   1  Read version info                                      ║
║   2  Set mode to SR_vFOC (bus FOC) — REQUIRED FIRST         ║
║   3  Enable motor                                           ║
║   4  Disable motor                                          ║
║                                                             ║
║  READ                                                       ║
║   5  Read encoder position                                  ║
║   6  Read current speed (RPM)                               ║
║   7  Read enable status                                     ║
║   8  Read motor running status                              ║
║                                                             ║
║  MOVE                                                       ║
║   9  Spin CW  (speed mode, 300 RPM)                         ║
║  10  Spin CCW (speed mode, 300 RPM)                         ║
║  11  Custom speed mode                                      ║
║  12  Move 1 turn CW  (relative pulse mode)                  ║
║  13  Move 1 turn CCW (relative pulse mode)                  ║
║  14  Move N turns (relative pulse mode)                     ║
║                                                             ║
║  STOP                                                       ║
║  15  Slow stop (decelerate)                                 ║
║  16  Emergency stop                                         ║
║                                                             ║
║  OTHER                                                      ║
║  17  Set current position as zero                           ║
║  18  Quick demo (setup → spin → stop)                       ║
║  19  Send raw CAN frame                                     ║
║   0  Quit                                                   ║
╚══════════════════════════════════════════════════════════════╝""")


def quick_demo(bus, can_id):
    """Full demo: set mode, enable, spin, wait, stop."""
    print("\n── Quick Demo ──────────────────────────────")
    print("[1/5] Setting mode to SR_vFOC...")
    set_work_mode(bus, can_id, 0x05)
    time.sleep(0.3)

    print("[2/5] Enabling motor...")
    set_enable(bus, can_id, True)
    time.sleep(0.3)

    print("[3/5] Spinning CW at 200 RPM for 3 seconds...")
    run_speed(bus, can_id, direction=1, speed=200, acc=10)

    for i in range(3, 0, -1):
        print(f"       ...running ({i}s remaining)")
        time.sleep(1)

    print("[4/5] Decelerating to stop...")
    stop_speed(bus, can_id, acc=10)
    time.sleep(1)

    print("[5/5] Reading final position...")
    read_encoder(bus, can_id)
    print("── Demo complete ──────────────────────────\n")


def send_raw_frame(bus, can_id):
    """Send a raw CAN frame (user enters hex bytes, CRC auto-added)."""
    print("  Enter data bytes in hex (space-separated), CRC will be added.")
    print("  Example: F6 01 2C 02")
    raw = input("  Bytes > ").strip()
    if not raw:
        return
    try:
        data = [int(b, 16) for b in raw.split()]
    except ValueError:
        print("  Invalid hex input!")
        return
    resp = send_cmd(bus, can_id, data)
    if resp:
        print(f"  Response: [{' '.join(f'{b:02X}' for b in resp)}]")


def main():
    parser = argparse.ArgumentParser(description="MKS SERVO42D/57D CAN Test")
    parser.add_argument("--id", type=lambda x: int(x, 0), default=DEFAULT_CAN_ID,
                        help=f"Motor CAN ID (default: 0x{DEFAULT_CAN_ID:02X})")
    parser.add_argument("--bitrate", type=int, default=DEFAULT_BITRATE,
                        help=f"CAN bitrate (default: {DEFAULT_BITRATE})")
    parser.add_argument("--channel", default=None,
                        help="CAN channel (default: auto-detect)")
    parser.add_argument("--interface", default=None,
                        help="CAN interface type (default: auto-detect)")
    args = parser.parse_args()

    can_id = args.id

    print("=" * 62)
    print("  MKS SERVO42D/57D CAN Bus Test — Fedora Linux + CANable")
    print("=" * 62)

    # Setup interface
    if args.interface and args.channel:
        iface, channel = args.interface, args.channel
    else:
        if os.geteuid() != 0:
            print("\n[!] This script needs root to configure CAN interfaces.")
            print("    Run with: sudo python3 mks_servo_can_test.py")
            print("    (Or set up can0 manually and pass --interface/--channel)\n")
            sys.exit(1)
        iface, channel = setup_can_interface(args.bitrate)

    # Open CAN bus
    print(f"[*] Opening CAN bus: interface={iface}, channel={channel}")
    try:
        bus = can.Bus(interface=iface, channel=channel, bitrate=args.bitrate)
    except Exception as e:
        print(f"[!] Failed to open CAN bus: {e}")
        sys.exit(1)
    print(f"[✓] CAN bus open. Motor CAN ID = 0x{can_id:02X}\n")

    try:
        while True:
            print_menu()
            choice = input("\nSelect [0-19] > ").strip()

            if choice == "0":
                print("Goodbye!")
                break
            elif choice == "1":
                read_version(bus, can_id)
            elif choice == "2":
                set_work_mode(bus, can_id, 0x05)
            elif choice == "3":
                set_enable(bus, can_id, True)
            elif choice == "4":
                set_enable(bus, can_id, False)
            elif choice == "5":
                read_encoder(bus, can_id)
            elif choice == "6":
                read_speed(bus, can_id)
            elif choice == "7":
                read_enable_status(bus, can_id)
            elif choice == "8":
                read_motor_status(bus, can_id)
            elif choice == "9":
                run_speed(bus, can_id, direction=1, speed=300, acc=10)
            elif choice == "10":
                run_speed(bus, can_id, direction=0, speed=300, acc=10)
            elif choice == "11":
                d = int(input("  Direction (0=CCW, 1=CW) > ") or "1")
                s = int(input("  Speed RPM (0-3000) > ") or "300")
                a = int(input("  Acceleration (0-255, 0=instant) > ") or "10")
                run_speed(bus, can_id, direction=d, speed=s, acc=a)
            elif choice == "12":
                run_relative_pulses(bus, can_id, direction=1, speed=300, acc=10, pulses=3200)
            elif choice == "13":
                run_relative_pulses(bus, can_id, direction=0, speed=300, acc=10, pulses=3200)
            elif choice == "14":
                d = int(input("  Direction (0=CCW, 1=CW) > ") or "1")
                n = float(input("  Number of turns > ") or "1")
                s = int(input("  Speed RPM (0-3000) > ") or "300")
                a = int(input("  Acceleration (0-255) > ") or "10")
                pulses = int(n * 3200)  # 16 microsteps × 200 steps = 3200/turn
                run_relative_pulses(bus, can_id, direction=d, speed=s, acc=a, pulses=pulses)
            elif choice == "15":
                stop_speed(bus, can_id, acc=10)
            elif choice == "16":
                emergency_stop(bus, can_id)
            elif choice == "17":
                set_current_position_zero(bus, can_id)
            elif choice == "18":
                quick_demo(bus, can_id)
            elif choice == "19":
                send_raw_frame(bus, can_id)
            else:
                print("  Invalid choice, try again.")

            print()

    except KeyboardInterrupt:
        print("\n[*] Interrupted — stopping motor...")
        try:
            emergency_stop(bus, can_id)
        except:
            pass

    finally:
        bus.shutdown()
        print("[✓] CAN bus closed.")


if __name__ == "__main__":
    main()