import threading
import pytest
import can
from arctos import can_interface


CHANNEL = "test_ch"
IFACE = "virtual"
BITRATE = 500000
CAN_ID = 0x01


def _open_virtual():
    return can_interface.open_canbus(channel=CHANNEL, interface=IFACE, bitrate=BITRATE)


def test_open_canbus_virtual():
    """Test that open_canbus returns a working Bus using python-can's virtual interface."""
    bus = _open_virtual()
    try:
        assert isinstance(bus, can.BusABC)
    finally:
        bus.shutdown()


def test_send_and_receive():
    """Send a frame on bus1, verify bus2 receives it with correct ID and data."""
    bus1 = _open_virtual()
    bus2 = _open_virtual()
    try:
        data = [0x01, 0x02, 0x03]
        can_interface.send(bus1, CAN_ID, data, expect_response=False)
        msg = bus2.recv(timeout=1.0)
        assert msg is not None
        assert msg.arbitration_id == CAN_ID
        assert list(msg.data) == data
    finally:
        bus1.shutdown()
        bus2.shutdown()


def test_awaitresponse():
    """Inject a message on bus2 and verify awaitresponse on bus1 returns it."""
    bus1 = _open_virtual()
    bus2 = _open_virtual()
    try:
        response_data = [0xAA, 0xBB, 0xCC]
        bus2.send(can.Message(arbitration_id=CAN_ID, data=response_data, is_extended_id=False))
        result = can_interface.awaitresponse(bus1, CAN_ID, timeout=1.0)
        assert result == response_data
    finally:
        bus1.shutdown()
        bus2.shutdown()


def test_send_with_response():
    """Full round-trip: bus2 echoes a response back, send() returns it."""
    bus1 = _open_virtual()
    bus2 = _open_virtual()
    try:
        send_data = [0x10, 0x20]
        reply_data = [0x30, 0x40]

        def echo():
            msg = bus2.recv(timeout=2.0)
            if msg:
                bus2.send(can.Message(arbitration_id=msg.arbitration_id, data=reply_data, is_extended_id=False))

        t = threading.Thread(target=echo)
        t.start()

        result = can_interface.send(bus1, CAN_ID, send_data, expect_response=True)
        t.join(timeout=3.0)
        assert result == reply_data
    finally:
        bus1.shutdown()
        bus2.shutdown()


def test_awaitresponse_timeout():
    """awaitresponse returns None when no message arrives."""
    bus = _open_virtual()
    try:
        result = can_interface.awaitresponse(bus, CAN_ID, timeout=0.3)
        assert result is None
    finally:
        bus.shutdown()


def test_close_canbus():
    """close_canbus shuts down the bus."""
    bus = _open_virtual()
    can_interface.close_canbus(bus)
    # After shutdown, sending should raise
    with pytest.raises(can.CanOperationError):
        bus.send(can.Message(arbitration_id=CAN_ID, data=[0x00], is_extended_id=False))
