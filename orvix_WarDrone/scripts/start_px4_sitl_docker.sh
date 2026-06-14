#!/bin/bash
set -e

echo "=== Starting PX4 SITL (headless) ==="
echo "MAVLink available on UDP ports 14540, 14550"
echo ""

cd "${PX4_HOME}"

export PX4_SYS_AUTOSTART=4001
export PX4_SIM_MODEL=none_iris

exec ./build/px4_sitl_default/bin/px4 \
    -s etc/init.d-posix/rcS \
    -w sitl_workspace \
    -d
