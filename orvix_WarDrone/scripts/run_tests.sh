#!/bin/bash
# Run all tests for the WarDrone ROS 2 workspace.
#
# Usage:
#   ./scripts/run_tests.sh          # Run all tests
#   ./scripts/run_tests.sh unit     # Run only unit tests (no ROS 2 needed)
#   ./scripts/run_tests.sh colcon   # Run colcon tests (needs ROS 2)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

MODE="${1:-all}"

if [ "$MODE" = "unit" ] || [ "$MODE" = "all" ]; then
    echo "=== Running unit tests (pytest) ==="
    echo ""

    # Test pure Python modules that don't need ROS 2
    python -m pytest src/wardrone_driver/test/ -v
    python -m pytest src/wardrone_navigation/test/ -v
    python -m pytest src/wardrone_vision/test/ -v
    python -m pytest src/wardrone_mission/test/ -v
    python -m pytest src/wardrone_vio/test/ -v

    echo ""
    echo "=== Unit tests complete ==="
fi

if [ "$MODE" = "colcon" ] || [ "$MODE" = "all" ]; then
    echo ""
    echo "=== Building workspace ==="
    colcon build --symlink-install

    echo ""
    echo "=== Running colcon tests ==="
    source install/setup.bash
    colcon test
    colcon test-result --verbose
fi

echo ""
echo "=== All tests complete ==="
