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

sys.path.insert(0, ".")
from arctos.can_interface import CANInterface
from arctos.config import CAN_BITRATE, CAN_CHANNEL, JOINTS
from arctos.motor_driver import MotorDriver
from scripts.motor_control import get_joint_by_can_id

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
_can: CANInterface | None = None
_drivers: dict[int, MotorDriver] = {}


def _sigint_handler(_signum, _frame):
    if _can is not None:
        print("\n!!! EMERGENCY STOP ALL MOTORS !!!")
        for _, can_id, *_ in JOINTS:
            _drivers[can_id].emergency_stop()
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


def test_read(
    driver: MotorDriver, cmd_byte: int, name: str, min_resp_len: int
) -> TestResult:
    """Send a read command and check for a valid response."""
    resp = driver.raw_command(bytes([cmd_byte]))
    if resp is None:
        return TestResult(name, False, "No response")
    if resp[0] != cmd_byte:
        return TestResult(name, False, f"Wrong echo: 0x{resp[0]:02X}")
    if len(resp) < min_resp_len:
        return TestResult(name, False, f"Short response: {len(resp)} bytes")
    return TestResult(name, True, f"OK ({resp[1:].hex()})")


def test_enable(driver: MotorDriver, enable: bool) -> TestResult:
    """Send enable/disable and verify response."""
    label = "Enable motor (0xF3)" if enable else "Disable motor (0xF3)"
    val = 0x01 if enable else 0x00
    resp = driver.raw_command(bytes([0xF3, val]))
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF3 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    if resp[1] != 1:
        return TestResult(label, False, f"Status: {resp[1]}")
    return TestResult(label, True, "Enabled" if enable else "Disabled")


def test_verify_enable_status(driver: MotorDriver, expect_enabled: bool) -> TestResult:
    """Read enable status and verify it matches expectation."""
    label = f"Verify {'enabled' if expect_enabled else 'disabled'} (0x3A)"
    resp = driver.raw_command(bytes([0x3A]))
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0x3A or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    actual = resp[1] == 1
    if actual == expect_enabled:
        return TestResult(label, True, "Enabled" if actual else "Disabled")
    return TestResult(
        label,
        False,
        f"Expected {'enabled' if expect_enabled else 'disabled'}, "
        f"got {'enabled' if actual else 'disabled'}",
    )


def test_set_zero(driver: MotorDriver) -> TestResult:
    """Set current position as zero (0x92)."""
    resp = driver.raw_command(bytes([0x92]))
    if resp is None:
        return TestResult("Set zero (0x92)", False, "No response")
    if resp[0] != 0x92 or len(resp) < 2:
        return TestResult("Set zero (0x92)", False, f"Unexpected: {resp.hex()}")
    ok = resp[1] == 1
    return TestResult("Set zero (0x92)", ok, "OK" if ok else f"Response: {resp[1]}")


def test_absolute_move(
    driver: MotorDriver, position: int, speed: int = 100, acc: int = 2
) -> TestResult:
    """Absolute axis move (0xF5) to a given position."""
    label = f"Absolute move to {position} (0xF5)"
    ok = driver.move_to_axis(position, speed, acc)
    if not ok:
        return TestResult(label, False, "No response")
    return TestResult(label, True, "Accepted")


def test_relative_move(
    driver: MotorDriver, rel: int, speed: int = 100, acc: int = 2
) -> TestResult:
    """Relative axis move (0xF4)."""
    label = f"Relative move {rel:+d} (0xF4)"
    data = (
        bytes([0xF4])
        + MotorDriver._encode_uint16(speed)
        + bytes([acc])
        + MotorDriver._encode_int24(rel)
    )
    resp = driver.raw_command(data)
    if resp is None:
        return TestResult(label, False, "No response")
    if resp[0] != 0xF4 or len(resp) < 2:
        return TestResult(label, False, f"Unexpected: {resp.hex()}")
    ok = resp[1] in (1, 2)  # 1=complete, 2=running
    return TestResult(label, ok, "Accepted" if ok else f"Response: {resp[1]}")


def test_read_encoder_value(
    driver: MotorDriver, expected_near: int, tolerance: int = 200
) -> TestResult:
    """Read encoder (0x31) and check it's near an expected value."""
    label = f"Read encoder ~{expected_near} (0x31)"
    value = driver.read_encoder()
    if value is None:
        return TestResult(label, False, "No response")
    diff = abs(value - expected_near)
    ok = diff <= tolerance
    return TestResult(label, ok, f"{value} counts (delta={diff})")


def test_read_status_stopped(driver: MotorDriver) -> TestResult:
    """Read motor status (0xF1), expect stopped (1)."""
    label = "Verify stopped (0xF1)"
    STATUS_MAP = {
        0: "Fail",
        1: "Stopped",
        2: "Accel",
        3: "Decel",
        4: "Full speed",
        5: "Homing",
        6: "Cal",
    }
    code = driver.read_status()
    if code is None:
        return TestResult(label, False, "No response")
    ok = code == 1
    return TestResult(label, ok, STATUS_MAP.get(code, f"Unknown({code})"))


def test_speed_mode(
    driver: MotorDriver, direction: int = 0, speed: int = 50, acc: int = 2
) -> TestResult:
    """Speed mode (0xF6) — start spinning."""
    label = f"Speed mode dir={direction} spd={speed} (0xF6)"
    ok = driver.set_speed(direction, speed, acc)
    if not ok:
        return TestResult(label, False, "No response")
    return TestResult(label, True, "Set")


def test_emergency_stop(driver: MotorDriver) -> TestResult:
    """Emergency stop (0xF7)."""
    ok = driver.emergency_stop()
    if not ok:
        return TestResult("Emergency stop (0xF7)", False, "No response")
    return TestResult("Emergency stop (0xF7)", True, "Stopped")


def test_read_speed_zero(driver: MotorDriver) -> TestResult:
    """Read speed (0x32) and verify it's near 0."""
    speed = driver.read_speed()
    if speed is None:
        return TestResult("Verify speed=0 (0x32)", False, "No response")
    ok = abs(speed) <= 20
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


def test_motor_gen(driver: MotorDriver):
    """Generator that yields one TestResult at a time."""
    # Phase 1: Read-only
    yield test_read(driver, 0x31, "Read encoder cumulative (0x31)", 7)
    yield test_read(driver, 0x30, "Read encoder carry (0x30)", 7)
    yield test_read(driver, 0x32, "Read speed (0x32)", 3)
    yield test_read(driver, 0xF1, "Read motor status (0xF1)", 2)
    yield test_read(driver, 0x3A, "Read enable status (0x3A)", 2)
    yield test_read(driver, 0x39, "Read shaft angle error (0x39)", 5)
    yield test_read(driver, 0x34, "Read IO port status (0x34)", 2)
    yield test_read(driver, 0x3E, "Read protection status (0x3E)", 2)
    yield test_read(driver, 0x3B, "Read go-home status (0x3B)", 2)

    # Phase 2: Enable / Disable cycle
    yield test_enable(driver, True)
    yield test_verify_enable_status(driver, True)
    yield test_enable(driver, False)
    yield test_verify_enable_status(driver, False)
    yield test_enable(driver, True)  # re-enable for motion
    time.sleep(0.5)  # motor needs time to initialize after re-enable

    # Phase 3: Motion
    yield test_set_zero(driver)
    yield test_absolute_move(driver, 150000)
    time.sleep(7.0)
    yield test_read_status_stopped(driver)
    yield test_read_encoder_value(driver, 150000)
    yield test_relative_move(driver, -150000)
    time.sleep(7.0)
    yield test_read_encoder_value(driver, 0)
    yield test_speed_mode(driver, direction=0, speed=50)
    time.sleep(0.5)
    yield test_emergency_stop(driver)
    time.sleep(1.0)
    yield test_read_speed_zero(driver)

    # Phase 4: Cleanup
    yield test_enable(driver, False)


def run_motor_tests(driver: MotorDriver, label: str) -> list[TestResult]:
    """Run all tests for a motor, allowing the user to skip on failure."""
    results: list[TestResult] = []
    gen = test_motor_gen(driver)
    test_idx = 0

    try:
        for result in gen:
            results.append(result)
            print(result)
            test_idx += 1

            if not result.passed:
                choice = (
                    input("  Press Enter to continue, or 's' to skip motor: ")
                    .strip()
                    .lower()
                )
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

        results = run_motor_tests(_drivers[can_id], label)
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

    _can.close()
    sys.exit(0 if total_pass == total_tests else 1)


if __name__ == "__main__":
    main()
