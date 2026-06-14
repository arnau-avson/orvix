#!/bin/bash
set -e

echo "=== Starting PX4 SITL (SIH, headless) ==="
echo "MAVLink broadcast enabled for Docker networking"
echo ""

BUILD_DIR="${PX4_HOME}/build/px4_sitl_default"

export PX4_SYS_AUTOSTART=10040
export PX4_SIM_MODEL=sihsim_quadx

exec "${BUILD_DIR}/bin/px4" \
    -s "${BUILD_DIR}/etc/init.d-posix/rcS" \
    -w "${BUILD_DIR}" \
    -d
