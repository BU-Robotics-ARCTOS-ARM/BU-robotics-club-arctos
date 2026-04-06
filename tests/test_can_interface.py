import threading
import pytest
import can
from arctos.can_interface import CANInterface


CHANNEL = "test_ch"
IFACE = "virtual"
BITRATE = 500000
CAN_ID = 0x01


def _make_can():
    return CANInterface(channel=CHANNEL, bitrate=BITRATE, interface=IFACE)


@pytest.fixture
def can_iface():
    ci = _make_can()
    yield ci
    ci.close()


@pytest.fixture
def can_pair():
    ci1 = _make_can()
    ci2 = _make_can()
    yield ci1, ci2
    ci1.close()
    ci2.close()


def test_send_and_receive(can_pair):
    """Send a frame on ci1, verify ci2 receives it with correct ID and data."""
    ci1, ci2 = can_pair
    data = [0x01, 0x02, 0x03]
    ci1.send(CAN_ID, data)
    result = ci2.receive(can_id=CAN_ID, timeout=1.0)
    assert result is not None
    arb_id, rdata = result
    assert arb_id == CAN_ID
    assert list(rdata) == data


def test_receive_any(can_pair):
    """receive() with no can_id filter returns any frame."""
    ci1, ci2 = can_pair
    ci1.send(0x05, [0xAA])
    result = ci2.receive(timeout=1.0)
    assert result is not None
    arb_id, rdata = result
    assert arb_id == 0x05
    assert list(rdata) == [0xAA]


def test_send_with_response(can_pair):
    """Full round-trip: ci2 echoes a response back, ci1 receives it."""
    ci1, ci2 = can_pair
    send_data = [0x10, 0x20]
    reply_data = [0x30, 0x40]

    def echo():
        result = ci2.receive(timeout=2.0)
        if result:
            ci2.send(result[0], reply_data)

    t = threading.Thread(target=echo)
    t.start()

    ci1.send(CAN_ID, send_data)
    result = ci1.receive(can_id=CAN_ID, timeout=2.0)
    t.join(timeout=3.0)
    assert result is not None
    assert list(result[1]) == reply_data


def test_receive_timeout(can_iface):
    """receive() returns None when no message arrives."""
    result = can_iface.receive(can_id=CAN_ID, timeout=0.3)
    assert result is None


def test_close():
    """close() shuts down the bus; subsequent sends raise."""
    ci = _make_can()
    ci.close()
    with pytest.raises(can.CanOperationError):
        ci.send(CAN_ID, [0x00])


def test_receive_filters_by_can_id(can_pair):
    """receive(can_id=X) skips non-matching frames and returns the matching one."""
    ci1, ci2 = can_pair
    ci1.send(0x99, [0xFF])
    ci1.send(CAN_ID, [0xAB])
    result = ci2.receive(can_id=CAN_ID, timeout=1.0)
    assert result is not None
    arb_id, rdata = result
    assert arb_id == CAN_ID
    assert list(rdata) == [0xAB]


def test_send_empty_data(can_pair):
    """Send and receive a frame with empty data."""
    ci1, ci2 = can_pair
    ci1.send(CAN_ID, [])
    result = ci2.receive(can_id=CAN_ID, timeout=1.0)
    assert result is not None
    assert list(result[1]) == []


def test_send_max_length_frame(can_pair):
    """Send and receive a frame with maximum 8 bytes of data."""
    ci1, ci2 = can_pair
    data = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
    ci1.send(CAN_ID, data)
    result = ci2.receive(can_id=CAN_ID, timeout=1.0)
    assert result is not None
    assert list(result[1]) == data
