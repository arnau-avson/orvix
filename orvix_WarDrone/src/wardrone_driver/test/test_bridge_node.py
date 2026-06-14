"""Integration tests for MavsdkBridgeNode.

These tests require a ROS 2 environment. Run with:
    colcon test --packages-select wardrone_driver
"""
import math

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
        q = euler_to_quaternion(0.0, 0.0, 0.0)
        assert abs(q.w - 1.0) < 1e-6
        assert abs(q.x) < 1e-6
        assert abs(q.y) < 1e-6
        assert abs(q.z) < 1e-6


class TestEulerToQuaternion:
    """Thorough tests for the euler_to_quaternion pure function."""

    @staticmethod
    def _quat_norm(q):
        """Compute the norm of a quaternion."""
        return math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2)

    def test_identity_rotation(self):
        """All-zero Euler angles must produce the identity quaternion (1,0,0,0)."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, 0.0, 0.0)
        assert abs(q.w - 1.0) < 1e-9
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z) < 1e-9

    def test_pure_yaw_90(self):
        """A 90-degree yaw (pi/2) must produce quaternion (cos(pi/4), 0, 0, sin(pi/4))."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, 0.0, math.pi / 2.0)
        expected_w = math.cos(math.pi / 4.0)
        expected_z = math.sin(math.pi / 4.0)
        assert abs(q.w - expected_w) < 1e-9
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z - expected_z) < 1e-9

    def test_pure_roll_90(self):
        """A 90-degree roll (pi/2) must produce quaternion (cos(pi/4), sin(pi/4), 0, 0)."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(math.pi / 2.0, 0.0, 0.0)
        expected_w = math.cos(math.pi / 4.0)
        expected_x = math.sin(math.pi / 4.0)
        assert abs(q.w - expected_w) < 1e-9
        assert abs(q.x - expected_x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z) < 1e-9

    def test_pure_pitch_90(self):
        """A 90-degree pitch (pi/2) must produce quaternion (cos(pi/4), 0, sin(pi/4), 0)."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, math.pi / 2.0, 0.0)
        expected_w = math.cos(math.pi / 4.0)
        expected_y = math.sin(math.pi / 4.0)
        assert abs(q.w - expected_w) < 1e-9
        assert abs(q.x) < 1e-9
        assert abs(q.y - expected_y) < 1e-9
        assert abs(q.z) < 1e-9

    def test_yaw_180(self):
        """A 180-degree yaw (pi) must produce quaternion (0, 0, 0, 1)."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, 0.0, math.pi)
        assert abs(q.w) < 1e-9
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z - 1.0) < 1e-9

    def test_negative_yaw_90(self):
        """A -90-degree yaw (-pi/2) must produce quaternion (cos(pi/4), 0, 0, -sin(pi/4))."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, 0.0, -math.pi / 2.0)
        expected_w = math.cos(math.pi / 4.0)
        expected_z = -math.sin(math.pi / 4.0)
        assert abs(q.w - expected_w) < 1e-9
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z - expected_z) < 1e-9

    def test_quaternion_normalization_identity(self):
        """Identity quaternion must have unit norm."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, 0.0, 0.0)
        assert abs(self._quat_norm(q) - 1.0) < 1e-9

    def test_quaternion_normalization_single_axis(self):
        """Quaternions from single-axis rotations must have unit norm."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        for angle in [0.0, math.pi / 6, math.pi / 4, math.pi / 3, math.pi / 2, math.pi]:
            q = euler_to_quaternion(angle, 0.0, 0.0)
            assert abs(self._quat_norm(q) - 1.0) < 1e-9, f"Roll={angle}"
            q = euler_to_quaternion(0.0, angle, 0.0)
            assert abs(self._quat_norm(q) - 1.0) < 1e-9, f"Pitch={angle}"
            q = euler_to_quaternion(0.0, 0.0, angle)
            assert abs(self._quat_norm(q) - 1.0) < 1e-9, f"Yaw={angle}"

    def test_quaternion_normalization_combined(self):
        """Quaternions from combined Euler angles must have unit norm."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        test_angles = [
            (0.1, 0.2, 0.3),
            (math.pi / 4, math.pi / 4, math.pi / 4),
            (-math.pi / 3, math.pi / 6, -math.pi / 2),
            (math.pi, math.pi / 2, -math.pi),
            (1.234, -0.567, 2.891),
        ]
        for roll, pitch, yaw in test_angles:
            q = euler_to_quaternion(roll, pitch, yaw)
            norm = self._quat_norm(q)
            assert abs(norm - 1.0) < 1e-9, (
                f"Norm={norm} for roll={roll}, pitch={pitch}, yaw={yaw}"
            )

    def test_small_angles(self):
        """Very small Euler angles should produce quaternion close to identity."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        eps = 1e-8
        q = euler_to_quaternion(eps, eps, eps)
        assert abs(q.w - 1.0) < 1e-6
        assert abs(q.x) < 1e-6
        assert abs(q.y) < 1e-6
        assert abs(q.z) < 1e-6

    def test_full_rotation_yaw_360(self):
        """A full 360-degree yaw (2*pi) must return to approximately (+-1, 0, 0, 0)."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        q = euler_to_quaternion(0.0, 0.0, 2.0 * math.pi)
        # 2*pi rotation gives quaternion (-1, 0, 0, 0) -- same orientation as identity
        assert abs(abs(q.w) - 1.0) < 1e-9
        assert abs(q.x) < 1e-9
        assert abs(q.y) < 1e-9
        assert abs(q.z) < 1e-9

    def test_opposite_rotations_cancel(self):
        """Applying a rotation then its inverse should get back near identity (quaternion product)."""
        from wardrone_driver.mavsdk_bridge_node import euler_to_quaternion
        # For a pure yaw of pi/3 and -pi/3, quaternion product should give identity.
        q1 = euler_to_quaternion(0.0, 0.0, math.pi / 3.0)
        q2 = euler_to_quaternion(0.0, 0.0, -math.pi / 3.0)
        # Quaternion product: q1 * q2
        w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        assert abs(abs(w) - 1.0) < 1e-9
        assert abs(x) < 1e-9
        assert abs(y) < 1e-9
        assert abs(z) < 1e-9


class TestCoordinateTransforms:
    """Tests for MavsdkClient static coordinate transform methods."""

    def test_enu_to_ned_identity(self):
        """ENU (0,0,0) must map to NED (0,0,0)."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        n, e, d = MavsdkClient.enu_to_ned(0.0, 0.0, 0.0)
        assert n == 0.0
        assert e == 0.0
        assert d == 0.0

    def test_enu_to_ned_axes_swap(self):
        """ENU (east=1, north=2, up=3) must map to NED (north=2, east=1, down=-3)."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        n, e, d = MavsdkClient.enu_to_ned(1.0, 2.0, 3.0)
        assert n == 2.0
        assert e == 1.0
        assert d == -3.0

    def test_ned_to_enu_identity(self):
        """NED (0,0,0) must map to ENU (0,0,0)."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        e, n, u = MavsdkClient.ned_to_enu(0.0, 0.0, 0.0)
        assert e == 0.0
        assert n == 0.0
        assert u == 0.0

    def test_ned_to_enu_axes_swap(self):
        """NED (north=2, east=1, down=3) must map to ENU (east=1, north=2, up=-3)."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        e, n, u = MavsdkClient.ned_to_enu(2.0, 1.0, 3.0)
        assert e == 1.0
        assert n == 2.0
        assert u == -3.0

    def test_enu_ned_roundtrip(self):
        """Converting ENU->NED->ENU must return the original values."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        x, y, z = 5.5, -3.2, 10.1
        n, e, d = MavsdkClient.enu_to_ned(x, y, z)
        x2, y2, z2 = MavsdkClient.ned_to_enu(n, e, d)
        assert abs(x2 - x) < 1e-12
        assert abs(y2 - y) < 1e-12
        assert abs(z2 - z) < 1e-12

    def test_yaw_enu_to_ned(self):
        """ENU yaw 0 (pointing East) must map to NED yaw pi/2."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        yaw_ned = MavsdkClient.yaw_enu_to_ned(0.0)
        assert abs(yaw_ned - math.pi / 2.0) < 1e-12

    def test_yaw_ned_to_enu(self):
        """NED yaw 0 (pointing North) must map to ENU yaw pi/2."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        yaw_enu = MavsdkClient.yaw_ned_to_enu(0.0)
        assert abs(yaw_enu - math.pi / 2.0) < 1e-12

    def test_yaw_roundtrip(self):
        """Converting yaw ENU->NED->ENU must return the original value."""
        from wardrone_driver.mavsdk_client import MavsdkClient
        original = 1.234
        yaw_ned = MavsdkClient.yaw_enu_to_ned(original)
        recovered = MavsdkClient.yaw_ned_to_enu(yaw_ned)
        assert abs(recovered - original) < 1e-12


class TestTelemetryDataDefaults:
    """Tests for the TelemetryData dataclass default values."""

    def test_default_values(self):
        """A fresh TelemetryData must have sane zero/false defaults."""
        from wardrone_driver.mavsdk_client import TelemetryData, FlightMode
        t = TelemetryData()
        assert t.latitude_deg == 0.0
        assert t.longitude_deg == 0.0
        assert t.relative_altitude_m == 0.0
        assert t.battery_remaining_pct == 0.0
        assert t.is_armed is False
        assert t.is_in_air is False
        assert t.flight_mode == FlightMode.UNKNOWN

    def test_custom_values(self):
        """TelemetryData must accept and store custom field values."""
        from wardrone_driver.mavsdk_client import TelemetryData, FlightMode
        t = TelemetryData(
            latitude_deg=41.3851,
            longitude_deg=2.1734,
            relative_altitude_m=50.0,
            battery_remaining_pct=85.0,
            is_armed=True,
            is_in_air=True,
            flight_mode=FlightMode.OFFBOARD,
        )
        assert t.latitude_deg == 41.3851
        assert t.longitude_deg == 2.1734
        assert t.relative_altitude_m == 50.0
        assert t.battery_remaining_pct == 85.0
        assert t.is_armed is True
        assert t.is_in_air is True
        assert t.flight_mode == FlightMode.OFFBOARD


class TestFlightModeEnum:
    """Tests for the FlightMode enum."""

    def test_all_modes_exist(self):
        from wardrone_driver.mavsdk_client import FlightMode
        expected = ['HOLD', 'OFFBOARD', 'RTL', 'LAND', 'MISSION',
                    'TAKEOFF', 'MANUAL', 'STABILIZED', 'UNKNOWN']
        for name in expected:
            assert hasattr(FlightMode, name), f"FlightMode.{name} is missing"

    def test_mode_values_match_names(self):
        """Each FlightMode value must equal its name."""
        from wardrone_driver.mavsdk_client import FlightMode
        for mode in FlightMode:
            assert mode.value == mode.name

    def test_mode_count(self):
        from wardrone_driver.mavsdk_client import FlightMode
        assert len(FlightMode) == 9
