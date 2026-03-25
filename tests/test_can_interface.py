import pytest
from arctos import can_interface

def test_open_canbus_virtual():
    """Test that open_canbus returns a working Bus using python-can's virtual interface."""
    bus = can_interface.open_canbus(channel="test", interface="virtual", bitrate=500000)
    try:
        assert isinstance(bus, can_interface.can.BusABC)
    finally:
        bus.shutdown()

# test can get response


# test can timeout


# test can close