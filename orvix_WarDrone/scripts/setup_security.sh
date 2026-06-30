#!/bin/bash
# Generate SROS2 security keys for WarDrone nodes
# Usage: ./setup_security.sh [keystore_dir]

KEYSTORE_DIR="${1:-${HOME}/wardrone_keystore}"

echo "Creating SROS2 keystore at ${KEYSTORE_DIR}"
ros2 security create_keystore "${KEYSTORE_DIR}"

NODES=(
    mavsdk_bridge
    waypoint_navigator
    safety_monitor
    mission_controller
    obstacle_detector
    obstacle_avoidance
    flight_logger
    wind_estimator
    camera
    detector
    tracker
    vio_bridge
)

for node in "${NODES[@]}"; do
    echo "Creating enclave for /wardrone/${node}"
    ros2 security create_enclave "${KEYSTORE_DIR}" "/wardrone/${node}"
done

echo ""
echo "Keystore created at ${KEYSTORE_DIR}"
echo ""
echo "Add these to your .bashrc or launch environment:"
echo "  export ROS_SECURITY_KEYSTORE=${KEYSTORE_DIR}"
echo "  export ROS_SECURITY_ENABLE=true"
echo "  export ROS_SECURITY_STRATEGY=Enforce"
