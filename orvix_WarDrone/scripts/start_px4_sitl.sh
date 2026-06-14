#!/bin/bash
# Start PX4 SITL with Gazebo for simulation testing.
#
# Prerequisites:
#   - PX4-Autopilot cloned and built
#   - Gazebo installed
#
# Usage:
#   ./scripts/start_px4_sitl.sh
#   ./scripts/start_px4_sitl.sh <vehicle_model>
#
# Default vehicle: x500 (quadcopter with depth camera)

set -e

VEHICLE="${1:-x500}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4-Autopilot not found at $PX4_DIR"
    echo "Set PX4_DIR environment variable or clone PX4:"
    echo "  git clone https://github.com/PX4/PX4-Autopilot.git --recursive"
    exit 1
fi

echo "Starting PX4 SITL with vehicle: $VEHICLE"
echo "PX4 directory: $PX4_DIR"
echo ""
echo "Once running, connect with:"
echo "  - QGroundControl: auto-connects on UDP 14550"
echo "  - MAVSDK: udp://:14540"
echo "  - ROS 2: ros2 launch wardrone_bringup sitl_driver_only.launch.py"
echo ""

cd "$PX4_DIR"
make px4_sitl gz_"$VEHICLE"
