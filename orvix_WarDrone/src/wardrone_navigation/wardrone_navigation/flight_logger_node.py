"""Flight logger node (black box).

Subscribes to telemetry, state, safety events, mission state, obstacles,
and wind estimates. Writes CSV log files to disk.

Logging starts when the vehicle arms and stops when it disarms.
Each flight session creates a new timestamped CSV file.
"""

import csv
import os
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from wardrone_interfaces.msg import Telemetry, MissionState as MissionStateMsg, ObstacleArray


CSV_COLUMNS = [
    'timestamp', 'lat', 'lon', 'abs_alt', 'rel_alt',
    'vel_n', 'vel_e', 'vel_d', 'roll', 'pitch', 'yaw',
    'battery_pct', 'battery_v', 'gps_sats', 'gps_fix',
    'flight_mode', 'is_armed', 'is_in_air',
    'mission_state', 'safety_event', 'obstacle_max_threat',
    'wind_speed_m_s',
]


class FlightLoggerNode(Node):

    def __init__(self):
        super().__init__('flight_logger')

        # Parameters
        self.declare_parameter('log_dir', '~/wardrone_logs/')
        self.declare_parameter('log_rate_hz', 2.0)
        self.declare_parameter('enable_logging', True)

        self._log_dir = os.path.expanduser(self.get_parameter('log_dir').value)
        self._log_rate = self.get_parameter('log_rate_hz').value
        self._enabled = self.get_parameter('enable_logging').value

        # State caches
        self._telemetry = None
        self._mission_state = ""
        self._last_safety_event = ""
        self._obstacle_max_threat = 0
        self._wind_speed = 0.0

        # File state
        self._csv_file = None
        self._csv_writer = None
        self._is_logging = False
        self._was_armed = False

        # Subscribers
        self.create_subscription(Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)
        self.create_subscription(String, '/wardrone/safety/event', self._on_safety_event, 10)
        self.create_subscription(MissionStateMsg, '/wardrone/mission/state',
                                 self._on_mission_state, 10)
        self.create_subscription(ObstacleArray, '/wardrone/obstacles',
                                 self._on_obstacles, 10)

        # Optional wind subscription (may not exist if wind_estimator not running)
        try:
            from wardrone_interfaces.msg import WindEstimate
            self.create_subscription(WindEstimate, '/wardrone/wind_estimate',
                                     self._on_wind, 10)
        except ImportError:
            pass

        # Log timer
        self.create_timer(1.0 / self._log_rate, self._log_tick)

        self.get_logger().info(f'Flight Logger ready (dir={self._log_dir}, enabled={self._enabled})')

    def _on_telemetry(self, msg: Telemetry):
        self._telemetry = msg
        is_armed = msg.is_armed

        if is_armed and not self._was_armed:
            self._start_logging()
        elif not is_armed and self._was_armed:
            self._stop_logging()
        self._was_armed = is_armed

    def _on_safety_event(self, msg: String):
        self._last_safety_event = msg.data

    def _on_mission_state(self, msg: MissionStateMsg):
        self._mission_state = msg.state

    def _on_obstacles(self, msg: ObstacleArray):
        self._obstacle_max_threat = msg.max_threat_level

    def _on_wind(self, msg):
        self._wind_speed = msg.wind_speed_m_s

    def _start_logging(self):
        if not self._enabled or self._is_logging:
            return
        os.makedirs(self._log_dir, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(self._log_dir, f"flight_{timestamp}.csv")
        self._csv_file = open(filepath, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_COLUMNS)
        self._is_logging = True
        self.get_logger().info(f"Flight log started: {filepath}")

    def _stop_logging(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        self._is_logging = False
        self.get_logger().info("Flight log stopped")

    def _log_tick(self):
        if not self._is_logging or self._telemetry is None:
            return
        t = self._telemetry
        row = [
            time.time(),
            t.latitude_deg, t.longitude_deg,
            t.absolute_altitude_m, t.relative_altitude_m,
            t.velocity_north_m_s, t.velocity_east_m_s, t.velocity_down_m_s,
            t.roll_deg, t.pitch_deg, t.yaw_deg,
            t.battery_remaining_pct, t.battery_voltage_v,
            t.gps_num_satellites, t.gps_fix_type,
            t.flight_mode, t.is_armed, t.is_in_air,
            self._mission_state, self._last_safety_event,
            self._obstacle_max_threat, self._wind_speed,
        ]
        self._csv_writer.writerow(row)
        self._csv_file.flush()
        self._last_safety_event = ""


def main(args=None):
    rclpy.init(args=args)
    node = FlightLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
