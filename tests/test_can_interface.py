'''from arctos.can_interface import *
import pytest 
import can
EFAULT_CAN_ID    = 0x01       # Slave CAN ID (set on motor OLED: CanID)
DEFAULT_BITRATE   = 500000     # 500K (set on motor OLED: CanRate)
DEFAULT_INTERFACE = "socketcan" # Linux SocketCAN
DEFAULT_CHANNEL   = "vcan0"
RECV_TIMEOUT      = 1.0        # seconds




def test_can_bus_opens():
    """Test that vcan0 opens successfully"""
    bus1 = can.interface.Bus(channel=DEFAULT_CHANNEL, interface=DEFAULT_INTERFACE, bitrate = DEFAULT_BITRATE)
    assert bus1 is not None, 'bus did not open'
    yield bus1
    bus1.shutdown()

def test_timeout():
    """Test timeout message is printed"""
    bus1 = can.interface.Bus(channel=DEFAULT_CHANNEL, interface=DEFAULT_INTERFACE, bitrate = DEFAULT_BITRATE)
    # ✅ Fixed: use can_bus fixture, removed undefined parameters
    can_id = DEFAULT_CAN_ID
    data = [0x00, 0x01, 0x02]
    resp = send_cmd(bus1, can_id, data)
    assert resp is None'''
import pytest
from arctos import can_interface

DEFAULT_CAN_ID = 0x01
DEFAULT_BITRATE = 500000
DEFAULT_INTERFACE = "socketcan"
DEFAULT_CHANNEL = "vcan0"

@pytest.fixture
def can_bus():
    """Fixture providing CAN bus connection"""
    bus = can.interface.Bus(
        channel=DEFAULT_CHANNEL,
        interface=DEFAULT_INTERFACE,
        bitrate=DEFAULT_BITRATE
    )
    yield bus
    bus.shutdown()  # Automatic cleanup
#def test_can_bus_opens(can_bus):
 #   """Test that vcan0 opens successfully"""
  #  assert can_bus is not None

'''def test_timeout(can_bus):
    """Test that send_cmd returns None on timeout"""
    can_id = DEFAULT_CAN_ID
    data = [0x00, 0x01, 0x02]
    
    resp = can_interface.send_cmd(can_bus, can_id, data)
    
    assert resp is None'''
