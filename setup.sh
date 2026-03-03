#!/bin/bash
# ═══════════════════════════════════════════════════════════
#  MKS SERVO CAN Setup Script for Fedora Linux + CANable
# ═══════════════════════════════════════════════════════════
# Run once to install dependencies and configure CAN interface.
# Usage: sudo bash setup_can.sh

set -e

BITRATE=${1:-500000}

echo "══════════════════════════════════════════════"
echo "  Fedora CAN Setup for MKS SERVO + CANable"
echo "══════════════════════════════════════════════"

# 1. Install system packages
echo "[1/4] Installing system packages..."
dnf install -y python3-pip can-utils kernel-modules-extra 2>/dev/null || true

# 2. Install python-can
echo "[2/4] Installing python-can..."
pip3 install python-can 2>/dev/null || pip3 install python-can --break-system-packages

# 3. Load kernel modules
echo "[3/4] Loading CAN kernel modules..."
modprobe can
modprobe can_raw
modprobe can_dev
modprobe gs_usb 2>/dev/null || true    # For candleLight firmware CANable
modprobe slcan 2>/dev/null || true     # For slcan firmware CANable

# 4. Bring up CAN interface
echo "[4/4] Configuring can0 at ${BITRATE} bps..."
ip link set can0 down 2>/dev/null || true
ip link set can0 type can bitrate ${BITRATE}
ip link set can0 up

echo ""
echo "══════════════════════════════════════════════"
echo "  ✓ Setup complete!"
echo ""
echo "  Verify with:  candump can0"
echo "  Run test:     sudo python3 mks_servo_can_test.py"
echo "══════════════════════════════════════════════"

# Show interface status
ip -details link show can0