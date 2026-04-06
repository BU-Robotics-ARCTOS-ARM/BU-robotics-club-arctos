"""Tests for arctos.motor_driver.MotorDriver — write commands."""

import threading

import pytest

from arctos.can_interface import CANInterface
from arctos.motor_driver import MotorDriver

CHANNEL = "test_motor"
IFACE = "virtual"
CAN_ID = 0x01


def _make_can():
    return CANInterface(channel=CHANNEL, bitrate=500000, interface=IFACE)


@pytest.fixture
def can_pair():
    driver_can = _make_can()
    motor_can = _make_can()
    yield driver_can, motor_can
    driver_can.close()
    motor_can.close()


@pytest.fixture
def motor(can_pair):
    driver_can, _ = can_pair
    return MotorDriver(driver_can, CAN_ID)


def _simulate_response(motor_can, response_payload: bytes):
    """Thread target: receive one command, reply with response_payload + CRC."""
    result = motor_can.receive(can_id=CAN_ID, timeout=2.0)
    if result is None:
        return
    crc = MotorDriver.calc_crc(CAN_ID, response_payload)
    motor_can.send(CAN_ID, response_payload + bytes([crc]))


def _simulate_raw_response(motor_can, raw_bytes: bytes):
    """Thread target: receive one command, reply with raw_bytes verbatim."""
    result = motor_can.receive(can_id=CAN_ID, timeout=2.0)
    if result is None:
        return
    motor_can.send(CAN_ID, raw_bytes)


def _run_with_simulated_response(motor_can, response_payload, fn):
    """Start a motor simulator thread, call fn, wait for thread."""
    t = threading.Thread(target=_simulate_response, args=(motor_can, response_payload))
    t.start()
    result = fn()
    t.join(timeout=3.0)
    return result


# ---- calc_crc ------------------------------------------------------------


def test_calc_crc():
    assert (
        MotorDriver.calc_crc(0x01, bytes([0xF3, 0x01])) == (0x01 + 0xF3 + 0x01) & 0xFF
    )


def test_calc_crc_wraps_at_256():
    assert MotorDriver.calc_crc(0xFF, bytes([0xFF])) == (0xFF + 0xFF) & 0xFF


# ---- enable ---------------------------------------------------------------


def test_enable_on(can_pair, motor):
    _, motor_can = can_pair
    result = _run_with_simulated_response(
        motor_can, bytes([0xF3, 0x01]), lambda: motor.enable(True)
    )
    assert result is True


def test_enable_off(can_pair, motor):
    _, motor_can = can_pair
    result = _run_with_simulated_response(
        motor_can, bytes([0xF3, 0x00]), lambda: motor.enable(False)
    )
    assert result is True


def test_enable_no_response(motor):
    assert motor.enable(True) is False


# ---- move_to_axis ---------------------------------------------------------


def test_move_to_axis_accepted(can_pair, motor):
    _, motor_can = can_pair
    result = _run_with_simulated_response(
        motor_can, bytes([0xF5, 0x02]), lambda: motor.move_to_axis(150000, 300, 2)
    )
    assert result is True


def test_move_to_axis_negative_position(can_pair, motor):
    """Negative positions use two's-complement int24 encoding."""
    _, motor_can = can_pair
    captured = []

    def sim():
        res = motor_can.receive(can_id=CAN_ID, timeout=2.0)
        if res is None:
            return
        captured.append(res[1])  # raw bytes received
        resp = bytes([0xF5, 0x02])
        crc = MotorDriver.calc_crc(CAN_ID, resp)
        motor_can.send(CAN_ID, resp + bytes([crc]))

    t = threading.Thread(target=sim)
    t.start()
    result = motor.move_to_axis(-1000, 300, 2)
    t.join(timeout=3.0)

    assert result is True
    # Verify the int24 encoding of -1000
    raw = captured[0]
    # Frame: [0xF5, speed_hi, speed_lo, acc, pos_b2, pos_b1, pos_b0, CRC]
    pos_bytes = raw[4:7]
    expected = (-1000 + (1 << 24)).to_bytes(3, "big")
    assert pos_bytes == expected


def test_move_to_axis_no_response(motor):
    assert motor.move_to_axis(0, 300, 2) is False


# ---- set_speed ------------------------------------------------------------


def test_set_speed(can_pair, motor):
    _, motor_can = can_pair
    captured = []

    def sim():
        res = motor_can.receive(can_id=CAN_ID, timeout=2.0)
        if res is None:
            return
        captured.append(res[1])
        resp = bytes([0xF6, 0x02])
        crc = MotorDriver.calc_crc(CAN_ID, resp)
        motor_can.send(CAN_ID, resp + bytes([crc]))

    t = threading.Thread(target=sim)
    t.start()
    result = motor.set_speed(direction=1, speed=300, acc=2)
    t.join(timeout=3.0)

    assert result is True
    # Verify direction bit packing: ((1 & 1) << 15) | (300 & 0x0FFF) = 0x812C
    raw = captured[0]
    assert raw[1:3] == bytes([0x81, 0x2C])


def test_set_speed_direction_zero(can_pair, motor):
    _, motor_can = can_pair
    captured = []

    def sim():
        res = motor_can.receive(can_id=CAN_ID, timeout=2.0)
        if res is None:
            return
        captured.append(res[1])
        resp = bytes([0xF6, 0x02])
        crc = MotorDriver.calc_crc(CAN_ID, resp)
        motor_can.send(CAN_ID, resp + bytes([crc]))

    t = threading.Thread(target=sim)
    t.start()
    result = motor.set_speed(direction=0, speed=300, acc=2)
    t.join(timeout=3.0)

    assert result is True
    raw = captured[0]
    # direction=0 -> bit15=0, speed=300=0x012C
    assert raw[1:3] == bytes([0x01, 0x2C])


# ---- stop -----------------------------------------------------------------


def test_stop_immediate(can_pair, motor):
    _, motor_can = can_pair
    captured = []

    def sim():
        res = motor_can.receive(can_id=CAN_ID, timeout=2.0)
        if res is None:
            return
        captured.append(res[1])
        resp = bytes([0xF6, 0x02])
        crc = MotorDriver.calc_crc(CAN_ID, resp)
        motor_can.send(CAN_ID, resp + bytes([crc]))

    t = threading.Thread(target=sim)
    t.start()
    result = motor.stop(acc=0)
    t.join(timeout=3.0)

    assert result is True
    raw = captured[0]
    # speed=0, acc=0
    assert raw[1:4] == bytes([0x00, 0x00, 0x00])


def test_stop_deceleration(can_pair, motor):
    _, motor_can = can_pair
    captured = []

    def sim():
        res = motor_can.receive(can_id=CAN_ID, timeout=2.0)
        if res is None:
            return
        captured.append(res[1])
        resp = bytes([0xF6, 0x02])
        crc = MotorDriver.calc_crc(CAN_ID, resp)
        motor_can.send(CAN_ID, resp + bytes([crc]))

    t = threading.Thread(target=sim)
    t.start()
    result = motor.stop(acc=5)
    t.join(timeout=3.0)

    assert result is True
    raw = captured[0]
    # speed=0, acc=5
    assert raw[1:4] == bytes([0x00, 0x00, 0x05])


# ---- emergency_stop -------------------------------------------------------


def test_emergency_stop(can_pair, motor):
    _, motor_can = can_pair
    result = _run_with_simulated_response(
        motor_can, bytes([0xF7, 0x01]), lambda: motor.emergency_stop()
    )
    assert result is True


def test_emergency_stop_no_response(motor):
    assert motor.emergency_stop() is False


# ---- error handling -------------------------------------------------------


def test_response_crc_mismatch(can_pair, motor):
    """Motor sends a response with wrong CRC — method returns False."""
    _, motor_can = can_pair
    bad_response = bytes([0xF3, 0x01, 0xFF])  # 0xFF is wrong CRC

    t = threading.Thread(target=_simulate_raw_response, args=(motor_can, bad_response))
    t.start()
    result = motor.enable(True)
    t.join(timeout=3.0)

    assert result is False
