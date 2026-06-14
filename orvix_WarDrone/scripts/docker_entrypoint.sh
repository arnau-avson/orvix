#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

# Handle commands
case "${1:-bash}" in
    colcon-build)
        echo "=== Building ROS 2 workspace ==="
        cd /ws
        colcon build --symlink-install
        echo "=== Build complete ==="
        ;;

    test-unit)
        echo "=== Running unit tests (no ROS 2 needed) ==="
        cd /ws
        python3 -m pytest src/wardrone_driver/test/test_mavsdk_client.py -v
        python3 -m pytest src/wardrone_navigation/test/test_mission_loader.py -v
        python3 -m pytest src/wardrone_navigation/test/test_waypoint_navigator.py -v
        python3 -m pytest src/wardrone_vision/test/test_yolo_wrapper.py -v
        python3 -m pytest src/wardrone_vision/test/test_object_tracker.py -v
        python3 -m pytest src/wardrone_vision/test/test_tracker_node.py -v
        python3 -m pytest src/wardrone_mission/test/test_state_machine.py -v
        python3 -m pytest src/wardrone_vio/test/test_vio_evaluator.py -v
        echo "=== Unit tests complete ==="
        ;;

    test-colcon)
        echo "=== Building + running colcon tests ==="
        cd /ws
        colcon build --symlink-install
        source install/setup.bash
        colcon test
        colcon test-result --verbose
        echo "=== Colcon tests complete ==="
        ;;

    test-all)
        echo "=== Running ALL tests ==="
        cd /ws
        # Unit tests first (fast)
        echo "--- Unit tests ---"
        python3 -m pytest src/wardrone_driver/test/test_mavsdk_client.py -v
        python3 -m pytest src/wardrone_navigation/test/test_mission_loader.py -v
        python3 -m pytest src/wardrone_navigation/test/test_waypoint_navigator.py -v
        python3 -m pytest src/wardrone_vision/test/test_yolo_wrapper.py -v
        python3 -m pytest src/wardrone_vision/test/test_object_tracker.py -v
        python3 -m pytest src/wardrone_vision/test/test_tracker_node.py -v
        python3 -m pytest src/wardrone_mission/test/test_state_machine.py -v
        python3 -m pytest src/wardrone_vio/test/test_vio_evaluator.py -v
        # Colcon tests
        echo "--- Colcon build + test ---"
        colcon build --symlink-install
        source install/setup.bash
        colcon test
        colcon test-result --verbose
        echo "=== All tests complete ==="
        ;;

    launch-driver)
        echo "=== Launching MAVSDK bridge (S1) ==="
        cd /ws
        colcon build --symlink-install
        source install/setup.bash
        ros2 launch wardrone_bringup sitl_driver_only.launch.py \
            connection_url:="udp://px4-sitl:14540"
        ;;

    launch-navigation)
        echo "=== Launching navigation stack (S2) ==="
        cd /ws
        colcon build --symlink-install
        source install/setup.bash
        ros2 launch wardrone_bringup sitl_navigation.launch.py \
            connection_url:="udp://px4-sitl:14540"
        ;;

    launch-full)
        echo "=== Launching full stack (S5) ==="
        cd /ws
        colcon build --symlink-install
        source install/setup.bash
        ros2 launch wardrone_bringup sitl_full.launch.py \
            connection_url:="udp://px4-sitl:14540"
        ;;

    bash)
        cd /ws
        exec bash
        ;;

    *)
        exec "$@"
        ;;
esac
