"""Safety monitor node.

Monitors telemetry for safety-critical conditions:
- Low battery → warning or RTL
- Critical battery → emergency land
- Telemetry link loss → RTL
- GPS quality degradation → warning

Publishes safety events and can trigger flight mode changes.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from wardrone_interfaces.msg import Telemetry, VehicleState
from wardrone_interfaces.srv import SetFlightMode


class SafetyMonitorNode(Node):

    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('battery_warning_pct', 30.0)
        self.declare_parameter('battery_critical_pct', 15.0)
        self.declare_parameter('link_timeout_s', 5.0)
        self.declare_parameter('telemetry_timeout_s', 3.0)
        self.declare_parameter('gps_min_satellites', 6)
        self.declare_parameter('action_on_warning', 'RTL')
        self.declare_parameter('action_on_critical', 'LAND')

        self._battery_warning = self.get_parameter('battery_warning_pct').value
        self._battery_critical = self.get_parameter('battery_critical_pct').value
        self._link_timeout = self.get_parameter('link_timeout_s').value
        self._telem_timeout = self.get_parameter('telemetry_timeout_s').value
        self._gps_min_sats = self.get_parameter('gps_min_satellites').value
        self._action_warning = self.get_parameter('action_on_warning').value
        self._action_critical = self.get_parameter('action_on_critical').value

        # State
        self._last_telemetry_time = time.time()
        self._battery_pct = 100.0
        self._gps_sats = 0
        self._gps_fix = 0
        self._is_armed = False
        self._is_in_air = False
        self._warning_sent = False
        self._critical_sent = False
        self._link_lost_sent = False
        self._gps_warning_sent = False

        cb_group = ReentrantCallbackGroup()

        # Publisher
        self._pub_safety = self.create_publisher(String, '/wardrone/safety/event', 10)

        # Subscribers
        self.create_subscription(Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)
        self.create_subscription(VehicleState, '/wardrone/state', self._on_state, 10)
        self.create_subscription(BatteryState, '/wardrone/battery', self._on_battery, 10)

        # Service client for flight mode changes
        self._set_mode_client = self.create_client(
            SetFlightMode, '/wardrone/set_flight_mode', callback_group=cb_group
        )

        # Monitor timer (2 Hz)
        self.create_timer(0.5, self._monitor_tick)

        self.get_logger().info('Safety Monitor ready')

    def _on_telemetry(self, msg: Telemetry):
        self._last_telemetry_time = time.time()
        self._battery_pct = msg.battery_remaining_pct
        self._gps_sats = msg.gps_num_satellites
        self._gps_fix = msg.gps_fix_type
        self._is_armed = msg.is_armed
        self._is_in_air = msg.is_in_air

    def _on_state(self, msg: VehicleState):
        self._is_armed = msg.is_armed
        self._is_in_air = msg.is_in_air

    def _on_battery(self, msg: BatteryState):
        self._last_telemetry_time = time.time()
        if msg.percentage > 0:
            self._battery_pct = msg.percentage * 100.0

    def _publish_event(self, event_type: str):
        msg = String()
        msg.data = event_type
        self._pub_safety.publish(msg)
        self.get_logger().warn(f'SAFETY EVENT: {event_type}')

    def _request_flight_mode(self, mode: str):
        if not self._set_mode_client.service_is_ready():
            self.get_logger().error(f'Cannot set mode {mode}: service not available')
            return
        request = SetFlightMode.Request()
        request.mode = mode
        self._set_mode_client.call_async(request)

    def _monitor_tick(self):
        if not self._is_armed or not self._is_in_air:
            # Reset flags when on ground
            self._warning_sent = False
            self._critical_sent = False
            self._link_lost_sent = False
            self._gps_warning_sent = False
            return

        # --- Battery checks ---
        if self._battery_pct <= self._battery_critical and not self._critical_sent:
            self._publish_event('CRITICAL_BATTERY')
            self._critical_sent = True
            self._request_flight_mode(self._action_critical)

        elif self._battery_pct <= self._battery_warning and not self._warning_sent:
            self._publish_event('LOW_BATTERY')
            self._warning_sent = True
            self._request_flight_mode(self._action_warning)

        # --- Link loss check ---
        elapsed = time.time() - self._last_telemetry_time
        if elapsed > self._telem_timeout and not self._link_lost_sent:
            self._publish_event('LINK_LOST')
            self._link_lost_sent = True
            self._request_flight_mode('RTL')
        elif elapsed <= self._telem_timeout:
            self._link_lost_sent = False

        # --- GPS check ---
        if self._gps_sats < self._gps_min_sats and self._gps_fix < 3 and not self._gps_warning_sent:
            self._publish_event('GPS_DEGRADED')
            self._gps_warning_sent = True
        elif self._gps_sats >= self._gps_min_sats:
            self._gps_warning_sent = False


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
