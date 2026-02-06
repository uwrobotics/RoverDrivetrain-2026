#!/bin/bash
# Script to setup CAN interface for ODrive communication
# Usage: sudo ./setup_can.sh [interface] [bitrate]
#   interface: CAN interface name (default: can0)
#   bitrate:   CAN bitrate in bps (default: 250000)

set -e

INTERFACE=${1:-can0}
BITRATE=${2:-250000}

echo "Setting up CAN interface: $INTERFACE at $BITRATE bps"

# Bring down interface if it's up
ip link set $INTERFACE down 2>/dev/null || true

# Set CAN interface parameters
ip link set $INTERFACE type can bitrate $BITRATE

# Bring up the interface
ip link set $INTERFACE up

# Show interface status
ip -details link show $INTERFACE

echo ""
echo "CAN interface $INTERFACE is ready!"
echo "You can monitor traffic with: candump $INTERFACE"
