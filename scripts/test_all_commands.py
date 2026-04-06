#!/usr/bin/env python3
"""Automated test of all MKS motor commands across all 6 motors.

Runs through read, enable/disable, motion, and cleanup commands on each motor
and reports PASS/FAIL for every command. Fully non-interactive.

Usage:
    python scripts/test_all_commands.py --channel slcan0 --interface slcan
    python scripts/test_all_commands.py --interface virtual --channel test

Ctrl+C sends emergency stop to ALL motors before exiting.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import can

sys.path.insert(0, ".")
from arctos.config import CAN_BITRATE, CAN_CHANNEL, ENCODER_COUNTS_PER_REV, JOINTS
from scripts.motor_control import (
    calc_crc,
    decode_int16,
    decode_int48,
    encode_int24,
    encode_uint16,
    get_joint_by_can_id,
    send_and_receive,
    send_command,
)

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
_bus: can.Bus | None = None


def _sigint_handler(_signum, _frame):
    if _bus is not None:
        print("\n!!! EMERGENCY STOP ALL MOTORS !!!")
        for _, can_id, *_ in JOINTS:
            send_command(_bus, can_id, bytes([0xF7]))
        print("Emergency stop sent to all 6 motors.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

class SkipMotor(Exception):
    """Raised when user chooses to skip remaining tests for a motor."""


class TestResult:
    def __init__(self, name: str, passed: bool | None, detail: str):
        self.name = name
        self.passed = passed  # True=pass, False=fail, None=skipped
        self.detail = detail

    def __str__(self):
        if self.passed is None:
            tag = "\033[33m[SKIP]\033[0m"
        elif self.passed:
            tag = "\033[32m[PASS]\033[0m"
        else:
            tag = "\033[31m[FAIL]\033[0m"
        return f"  {tag} {self.name:40s}: {self.detail}"


def test_read(bus: can.Bus, can_id: int, cmd_byte: int, name: str,
              min_resp_len: int) -> TestResult:
    """Send a read command and check for a valid response."""
    resp = send_and_receive(bus, can_id, bytes([cmd_byte]))
    if resp is None:
        return TestResult(name, False, "No response")
    if resp[0] != cmd_byte:
        return TestResult(name, False, f"Wrong echo: 0x{resp[0]:02X}")
    if len(resp) < min_resp_len:
        return TestResult(name, False, f"Short response: {len(resp)} bytes")
    return TestResult(name, True, f"OK ({resp[1:].hex()})")


def test_enable(bus: can.Bus, can_id: int, enable: bool) -> TestResult:
    """Send enable/disable and verify response."""
    label = "Enable motor (0xF3)" if enable else "Disable motor (0xF3)"
    val = 0x01 if enable else 0x00
    resp = send_and_receive(bus, can_id, bytes([0xF3, val]))
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF3 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    return TestResult(label, True, "Enabled" if enable else "Disabled")


def test_verify_enable_status(bus: can.Bus, can_id: int, expect_enabled: bool) -> TestResult:
    """Read enable status and verify it matches expectation."""
    label = f"Verify {'enabled' if expect_enabled else 'disabled'} (0x3A)"
    resp = send_and_receive(bus, can_id, bytes([0x3A]))
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0x3A or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    actual = resp[1] == 1
    if actual == expect_enabled:
        return TestResult(label, True, "Enabled" if actual else "Disabled")
    return TestResult(label, False,
                      f"Expected {'enabled' if expect_enabled else 'disabled'}, "
                      f"got {'enabled' if actual else 'disabled'}")


def test_set_zero(bus: can.Bus, can_id: int) -> TestResult:
    """Set current position as zero (0x92)."""
    resp = send_and_receive(bus, can_id, bytes([0x92]))
    if resp is None:
        return TestResult("Set zero (0x92)", False, "No response")
    if resp[0] != 0x92 or len(resp) < 2:
        return TestResult("Set zero (0x92)", False, f"Unexpected: {resp.hex()}")
    ok = resp[1] == 1
    return TestResult("Set zero (0x92)", ok, "OK" if ok else f"Response: {resp[1]}")


def test_absolute_move(bus: can.Bus, can_id: int, position: int,
                       speed: int = 100, acc: int = 2) -> TestResult:
    """Absolute axis move (0xF5) to a given position."""
    label = f"Absolute move to {position} (0xF5)"
    data = bytes([0xF5]) + encode_uint16(speed) + bytes([acc]) + encode_int24(position)
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF5 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    ok = resp[1] in (1, 2)  # 1=complete, 2=running
    return TestResult(label, ok, "Accepted" if ok else f"Response: {resp[1]}")


def test_relative_move(bus: can.Bus, can_id: int, rel: int,
                       speed: int = 100, acc: int = 2) -> TestResult:
    """Relative axis move (0xF4)."""
    label = f"Relative move {rel:+d} (0xF4)"
    data = bytes([0xF4]) + encode_uint16(speed) + bytes([acc]) + encode_int24(rel)
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF4 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    ok = resp[1] in (1, 2)  # 1=complete, 2=running
    return TestResult(label, ok, "Accepted" if ok else f"Response: {resp[1]}")


def test_read_encoder_value(bus: can.Bus, can_id: int, expected_near: int,
                            tolerance: int = 200) -> TestResult:
    """Read encoder (0x31) and check it's near an expected value."""
    label = f"Read encoder ~{expected_near} (0x31)"
    resp = send_and_receive(bus, can_id, bytes([0x31]))
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0x31 or len(resp) < 7:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    value = decode_int48(resp[1:7])
    diff = abs(value - expected_near)
    ok = diff <= tolerance
    return TestResult(label, ok, f"{value} counts (delta={diff})")


def test_read_status_stopped(bus: can.Bus, can_id: int) -> TestResult:
    """Read motor status (0xF1), expect stopped (1)."""
    label = "Verify stopped (0xF1)"
    STATUS_MAP = {0: "Fail", 1: "Stopped", 2: "Accel", 3: "Decel",
                  4: "Full speed", 5: "Homing", 6: "Cal"}
    resp = send_and_receive(bus, can_id, bytes([0xF1]))
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF1 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    code = resp[1]
    ok = code == 1
    return TestResult(label, ok, STATUS_MAP.get(code, f"Unknown({code})"))


def test_speed_mode(bus: can.Bus, can_id: int, direction: int = 0,
                    speed: int = 50, acc: int = 2) -> TestResult:
    """Speed mode (0xF6) — start spinning."""
    label = f"Speed mode dir={direction} spd={speed} (0xF6)"
    dir_speed = ((direction & 1) << 15) | (speed & 0x0FFF)
    data = bytes([0xF6]) + encode_uint16(dir_speed) + bytes([acc])
    resp = send_and_receive(bus, can_id, data)
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF6 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    ok = resp[1] in (1, 2)  # 1=complete, 2=running
    return TestResult(label, ok, "Set" if ok else f"Response: {resp[1]}")


def test_emergency_stop(bus: can.Bus, can_id: int) -> TestResult:
    """Emergency stop (0xF7)."""
    resp = send_and_receive(bus, can_id, bytes([0xF7]))
    if resp is None:
        return TestResult("Emergency stop (0xF7)", False, "No response")
    if resp[0] != 0xF7 or len(resp) < 2:
        return TestResult("Emergency stop (0xF7)", False, f"Unexpected: {resp.hex()}")
    ok = resp[1] == 1
    return TestResult("Emergency stop (0xF7)", ok, "Stopped" if ok else f"Response: {resp[1]}")


def test_read_speed_zero(bus: can.Bus, can_id: int) -> TestResult:
    """Read speed (0x32) and verify it's 0."""
    resp = send_and_receive(bus, can_id, bytes([0x32]))
    if resp is None:
        return TestResult("Verify speed=0 (0x32)", False, "No response")
    if resp[0] != 0x32 or len(resp) < 3:
        return TestResult("Verify speed=0 (0x32)", False, f"Unexpected: {resp.hex()}")
    speed = decode_int16(resp[1:3])
    ok = speed == 0
    return TestResult("Verify speed=0 (0x32)", ok, f"{speed} RPM")


# ---------------------------------------------------------------------------
# Run all tests for one motor
# ---------------------------------------------------------------------------

_TEST_NAMES = [
    "Read encoder cumulative (0x31)",
    "Read encoder carry (0x30)",
    "Read speed (0x32)",
    "Read motor status (0xF1)",
    "Read enable status (0x3A)",
    "Read shaft angle error (0x39)",
    "Read IO port status (0x34)",
    "Read protection status (0x3E)",
    "Read go-home status (0x3B)",
    "Enable motor (0xF3)",
    "Verify enabled (0x3A)",
    "Disable motor (0xF3)",
    "Verify disabled (0x3A)",
    "Enable motor (0xF3)",
    "Set zero (0x92)",
    "Absolute move to 150000 (0xF5)",
    "Verify stopped (0xF1)",
    "Read encoder ~150000 (0x31)",
    "Relative move -150000 (0xF4)",
    "Read encoder ~0 (0x31)",
    "Speed mode dir=0 spd=50 (0xF6)",
    "Emergency stop (0xF7)",
    "Verify speed=0 (0x32)",
    "Disable motor (0xF3)",
]


def test_motor_gen(bus: can.Bus, can_id: int):
    """Generator that yields one TestResult at a time."""
    # Phase 1: Read-only
    yield test_read(bus, can_id, 0x31, "Read encoder cumulative (0x31)", 7)
    yield test_read(bus, can_id, 0x30, "Read encoder carry (0x30)", 7)
    yield test_read(bus, can_id, 0x32, "Read speed (0x32)", 3)
    yield test_read(bus, can_id, 0xF1, "Read motor status (0xF1)", 2)
    yield test_read(bus, can_id, 0x3A, "Read enable status (0x3A)", 2)
    yield test_read(bus, can_id, 0x39, "Read shaft angle error (0x39)", 5)
    yield test_read(bus, can_id, 0x34, "Read IO port status (0x34)", 2)
    yield test_read(bus, can_id, 0x3E, "Read protection status (0x3E)", 2)
    yield test_read(bus, can_id, 0x3B, "Read go-home status (0x3B)", 2)

    # Phase 2: Enable / Disable cycle
    yield test_enable(bus, can_id, True)
    yield test_verify_enable_status(bus, can_id, True)
    yield test_enable(bus, can_id, False)
    yield test_verify_enable_status(bus, can_id, False)
    yield test_enable(bus, can_id, True)  # re-enable for motion
    time.sleep(0.5)  # motor needs time to initialize after re-enable

    # Phase 3: Motion
    yield test_set_zero(bus, can_id)
    yield test_absolute_move(bus, can_id, 150000)
    time.sleep(7.0)
    yield test_read_status_stopped(bus, can_id)
    yield test_read_encoder_value(bus, can_id, 150000)
    yield test_relative_move(bus, can_id, -150000)
    time.sleep(7.0)
    yield test_read_encoder_value(bus, can_id, 0)
    yield test_speed_mode(bus, can_id, direction=0, speed=50)
    time.sleep(0.5)
    yield test_emergency_stop(bus, can_id)
    time.sleep(1.0)
    yield test_read_speed_zero(bus, can_id)

    # Phase 4: Cleanup
    yield test_enable(bus, can_id, False)


def run_motor_tests(bus: can.Bus, can_id: int, label: str) -> list[TestResult]:
    """Run all tests for a motor, allowing the user to skip on failure."""
    results: list[TestResult] = []
    gen = test_motor_gen(bus, can_id)
    test_idx = 0

    try:
        for result in gen:
            results.append(result)
            print(result)
            test_idx += 1

            if not result.passed:
                choice = input("  Press Enter to continue, or 's' to skip motor: ").strip().lower()
                if choice == "s":
                    raise SkipMotor
    except SkipMotor:
        # Mark remaining tests as skipped
        for remaining_name in _TEST_NAMES[test_idx:]:
            skipped = TestResult(remaining_name, None, "Skipped")
            results.append(skipped)
            print(skipped)

    return results


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Automated MKS motor command test")
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
    print(f"Connected to {args.channel} ({args.interface}) at {args.bitrate} bps\n")

    all_results: dict[str, list[TestResult]] = {}
    total_pass = 0
    total_skip = 0
    total_tests = 0

    for name, can_id, *_ in JOINTS:
        label = f"{name} (0x{can_id:02X})"
        print(f"{'=' * 50}")
        print(f"  Testing {label}")
        print(f"{'=' * 50}")

        results = run_motor_tests(_bus, can_id, label)
        all_results[label] = results

        passed = sum(1 for r in results if r.passed is True)
        skipped = sum(1 for r in results if r.passed is None)
        total_pass += passed
        total_skip += skipped
        total_tests += len(results)
        skip_str = f", {skipped} skipped" if skipped else ""
        print(f"\n  {passed}/{len(results)} passed{skip_str}\n")

    # Summary
    print(f"\n{'=' * 50}")
    print("  SUMMARY")
    print(f"{'=' * 50}")
    for label, results in all_results.items():
        passed = sum(1 for r in results if r.passed is True)
        skipped = sum(1 for r in results if r.passed is None)
        failed = len(results) - passed - skipped
        if failed == 0 and skipped == 0:
            status = "\033[32mALL PASS\033[0m"
        elif failed == 0:
            status = f"\033[33m{skipped} SKIPPED\033[0m"
        else:
            status = f"\033[31m{failed} FAILED\033[0m"
            if skipped:
                status += f", \033[33m{skipped} SKIPPED\033[0m"
        print(f"  {label:25s}: {passed}/{len(results)} — {status}")
    skip_str = f", {total_skip} skipped" if total_skip else ""
    print(f"\n  Total: {total_pass}/{total_tests} passed{skip_str}")
    print(f"{'=' * 50}")

    _bus.shutdown()
    sys.exit(0 if total_pass == total_tests else 1)


if __name__ == "__main__":
    main()
