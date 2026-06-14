"""Integration tests for MavsdkBridgeNode.

These tests require a ROS 2 environment. Run with:
    colcon test --packages-select wardrone_driver
"""
import pytest


class TestBridgeNodeStructure:
    """Basic structural tests that don't require ROS 2 runtime."""

    def test_import_bridge_node(self):
        """Verify the bridge node module can be imported."""
        # This will fail if there are syntax errors
        # Note: actual instantiation requires rclpy.init()
        from wardrone_driver import mavsdk_bridge_node
        assert hasattr(mavsdk_bridge_node, 'MavsdkBridgeNode')
        assert hasattr(mavsdk_bridge_node, 'main')

    def test_import_mavsdk_client(self):
        from wardrone_driver.mavsdk_client import MavsdkClient, TelemetryData, FlightMode
        assert MavsdkClient is not None

    def test_euler_to_quaternion(self):
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        import math
        q = euler_to_quaternion(0.0, 0.0, 0.0)
        assert abs(q.w - 1.0) < 1e-6
        assert abs(q.x) < 1e-6
        assert abs(q.y) < 1e-6
        assert abs(q.z) < 1e-6
