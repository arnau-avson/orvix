"""Wind estimator node.

Estimates wind vector from the difference between commanded velocity
(rotated from body to NED frame using yaw) and actual GPS ground velocity.

When the drone is in flight, the wind vector is:
    wind_NED = actual_velocity_NED - commanded_velocity_NED

A running average filter smooths the estimates over a configurable window.
"""

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from wardrone_interfaces.msg import Telemetry, WindEstimate


# --- Pure functions (testable without rclpy) ---

def body_to_ned(vx_body: float, vy_body: float, vz_body: float, yaw_rad: float):
    """Rotate body-frame velocity to NED using yaw (assumes small roll/pitch)."""
    cos_y = math.cos(yaw_rad)
    sin_y = math.sin(yaw_rad)
    v_north = vx_body * cos_y - vy_body * sin_y
    v_east = vx_body * sin_y + vy_body * cos_y
    v_down = vz_body
    return v_north, v_east, v_down


def estimate_wind(actual_ned: tuple, commanded_ned: tuple) -> tuple:
    """Wind = actual - commanded (residual is wind)."""
    return (
        actual_ned[0] - commanded_ned[0],
        actual_ned[1] - commanded_ned[1],
        actual_ned[2] - commanded_ned[2],
    )


def wind_speed_and_direction(wind_n: float, wind_e: float) -> tuple:
    """Compute wind speed (m/s) and direction (meteorological: where wind comes FROM)."""
    speed = math.sqrt(wind_n ** 2 + wind_e ** 2)
    if speed < 0.01:
        return 0.0, 0.0
    # Direction wind blows TO
    to_dir = math.degrees(math.atan2(wind_e, wind_n))
    # Meteorological convention: where it comes FROM
    from_dir = (to_dir + 180.0) % 360.0
    return speed, from_dir


class RunningAverageFilter:
    """Running average over a time window."""

    def __init__(self, window_s: float):
        self._window_s = window_s
        self._samples = deque()

    def add_sample(self, timestamp: float, values: tuple):
        self._samples.append((timestamp, values))
        self._prune(timestamp)

    def get_average(self, now: float) -> tuple:
        self._prune(now)
        if not self._samples:
            return (0.0, 0.0, 0.0)
        n = len(self._samples)
        dim = len(self._samples[0][1])
        sums = [0.0] * dim
        for _, vals in self._samples:
            for i in range(dim):
                sums[i] += vals[i]
        return tuple(s / n for s in sums)

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def _prune(self, now: float):
        cutoff = now - self._window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()


class WindEstimatorNode(Node):

    def __init__(self):
        super().__init__('wind_estimator')

        # Parameters
        self.declare_parameter('wind_filter_window_s', 10.0)
        self.declare_parameter('max_safe_wind_m_s', 8.0)
        self.declare_parameter('estimation_rate_hz', 2.0)
        self.declare_parameter('min_altitude_for_estimation_m', 3.0)

        window = self.get_parameter('wind_filter_window_s').value
        self._max_safe_wind = self.get_parameter('max_safe_wind_m_s').value
        rate = self.get_parameter('estimation_rate_hz').value
        self._min_alt = self.get_parameter('min_altitude_for_estimation_m').value

        # State
        self._filter = RunningAverageFilter(window)
        self._cmd_vx = 0.0
        self._cmd_vy = 0.0
        self._cmd_vz = 0.0
        self._yaw_rad = 0.0
        self._actual_vn = 0.0
        self._actual_ve = 0.0
        self._actual_vd = 0.0
        self._altitude = 0.0
        self._is_in_air = False
        self._wind_warning_sent = False

        # Publishers
        self._pub_wind = self.create_publisher(WindEstimate, '/wardrone/wind_estimate', 10)
        self._pub_safety = self.create_publisher(String, '/wardrone/safety/event', 10)

        # Subscribers
        self.create_subscription(Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)
        self.create_subscription(Twist, '/wardrone/cmd_velocity', self._on_cmd_velocity, 10)

        # Timer
        self.create_timer(1.0 / rate, self._estimate_tick)

        self.get_logger().info('Wind Estimator ready')

    def _on_telemetry(self, msg: Telemetry):
        self._actual_vn = msg.velocity_north_m_s
        self._actual_ve = msg.velocity_east_m_s
        self._actual_vd = msg.velocity_down_m_s
        self._yaw_rad = math.radians(msg.yaw_deg)
        self._altitude = msg.relative_altitude_m
        self._is_in_air = msg.is_in_air

    def _on_cmd_velocity(self, msg: Twist):
        self._cmd_vx = msg.linear.x   # forward
        self._cmd_vy = msg.linear.y   # left (body frame)
        self._cmd_vz = msg.linear.z   # up (body frame, positive up)

    def _estimate_tick(self):
        if not self._is_in_air or self._altitude < self._min_alt:
            return

        # Convert commanded body velocity to NED
        # Note: cmd_vz in body is up-positive, NED down is positive
        cmd_ned = body_to_ned(self._cmd_vx, self._cmd_vy, -self._cmd_vz, self._yaw_rad)
        actual_ned = (self._actual_vn, self._actual_ve, self._actual_vd)

        raw_wind = estimate_wind(actual_ned, cmd_ned)
        now = time.time()
        self._filter.add_sample(now, raw_wind)

        avg_wind = self._filter.get_average(now)
        speed, direction = wind_speed_and_direction(avg_wind[0], avg_wind[1])

        # Confidence based on number of samples
        confidence = min(1.0, self._filter.sample_count / 20.0)

        is_warning = speed > self._max_safe_wind

        # Publish wind estimate
        msg = WindEstimate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.wind_north_m_s = float(avg_wind[0])
        msg.wind_east_m_s = float(avg_wind[1])
        msg.wind_down_m_s = float(avg_wind[2])
        msg.wind_speed_m_s = float(speed)
        msg.wind_direction_deg = float(direction)
        msg.confidence = float(confidence)
        msg.is_warning = is_warning
        self._pub_wind.publish(msg)

        # Safety event (fire once)
        if is_warning and not self._wind_warning_sent:
            self._wind_warning_sent = True
            event = String()
            event.data = 'WIND_WARNING'
            self._pub_safety.publish(event)
            self.get_logger().warn(
                f'Wind warning: {speed:.1f} m/s from {direction:.0f}deg (max {self._max_safe_wind} m/s)'
            )
        elif not is_warning and self._wind_warning_sent:
            self._wind_warning_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = WindEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
