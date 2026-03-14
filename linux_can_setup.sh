#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  MKS SERVO CAN Setup Script for Fedora Linux + CANable2
# ═══════════════════════════════════════════════════════════
# Run once to install dependencies and configure CAN interface.
# Usage: sudo bash setup_can.sh

set -e

# slcand speed codes: s4=125K s5=250K s6=500K s8=1M
SLCAN_SPEED="s6"   # 500K (must match motor CanRate setting)
SERIAL_DEV="/dev/ttyACM0"

echo "══════════════════════════════════════════════"
echo "  Fedora CAN Setup for MKS SERVO + CANable2"
echo "══════════════════════════════════════════════"

# 1. Install system packages
echo "[1/4] Installing system packages..."
sudo dnf install -y can-utils 2>/dev/null

# 2. Install python-can
echo "[2/4] Installing python-can..."
uv add python-can 2>/dev/null || pip3 install python-can --break-system-packages

# 3. Check CANable2 is connected
echo "[3/4] Looking for CANable2..."
if [ ! -e "$SERIAL_DEV" ]; then
    echo "[!] $SERIAL_DEV not found. Is CANable2 plugged in?"
    echo "    Check: ls /dev/ttyACM*"
    exit 1
fi
echo "  Found: $SERIAL_DEV"

# 4. Bring up CAN interface via slcand
echo "[4/4] Starting slcand on $SERIAL_DEV (500K)..."
# Kill any existing slcand first
killall slcand 2>/dev/null || true
ip link set can0 down 2>/dev/null || true

slcand -o -c -${SLCAN_SPEED} ${SERIAL_DEV} can0
sleep 0.5
ip link set can0 up

echo ""
echo "══════════════════════════════════════════════"
echo "  ✓ Setup complete!"
echo ""
echo "  Verify with:  candump can0"
echo "  Run test:     sudo python3 move_90deg.py"
echo "══════════════════════════════════════════════"

ip -details link show can0