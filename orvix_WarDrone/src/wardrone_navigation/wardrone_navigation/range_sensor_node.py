"""Range sensor node -- serial laser/ultrasonic rangefinder driver.

Reads distance from a serial range sensor (TFmini-S protocol by default)
and publishes obstacles to /wardrone/obstacles/range.  A separate
obstacle_merger_node combines this with /wardrone/obstacles/vision into
the unified /wardrone/obstacles topic consumed by the avoidance node.

Supported protocols:
    - TFmini-S (default): 9-byte UART frames at 115200 baud
      Frame: [0x59][0x59][Dist_L][Dist_H][Str_L][Str_H][Temp_L][Temp_H][Checksum]
      Distance in cm, range 0.3–12 m, 100 Hz output

The sensor is typically mounted forward-facing (sector=FRONT) but can be
configured for any sector (e.g., BOTTOM for ground proximity).

Hardware connection:
    Sensor TX -> Raspberry Pi UART RX (GPIO 15 / /dev/ttyAMA0)
    Sensor VCC -> 5V, GND -> GND
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from wardrone_interfaces.msg import Obstacle, ObstacleArray, Telemetry


# ---------------------------------------------------------------------------
# Pure functions (testable without rclpy)
# ---------------------------------------------------------------------------

# TFmini-S frame constants
TFMINI_HEADER = 0x59
TFMINI_FRAME_SIZE = 9


def parse_tfmini_frame(data: bytes) -> dict:
    """Parse a 9-byte TFmini-S frame.

    Returns dict with 'distance_m', 'strength', 'valid' keys,
    or {'valid': False} if the frame is malformed.
    """
    if len(data) < TFMINI_FRAME_SIZE:
        return {'valid': False, 'error': 'short_frame'}

    if data[0] != TFMINI_HEADER or data[1] != TFMINI_HEADER:
        return {'valid': False, 'error': 'bad_header'}

    # Checksum: low byte of sum of first 8 bytes
    checksum = sum(data[:8]) & 0xFF
    if checksum != data[8]:
        return {'valid': False, 'error': 'bad_checksum'}

    dist_cm = data[2] | (data[3] << 8)
    strength = data[4] | (data[5] << 8)

    # Distance 0 or very low strength = invalid reading
    if dist_cm == 0 or strength < 100:
        return {'valid': False, 'error': 'weak_signal'}

    return {
        'valid': True,
        'distance_m': dist_cm / 100.0,
        'strength': strength,
    }


def find_tfmini_frame(buffer: bytes) -> tuple:
    """Find the next valid TFmini frame in a byte buffer.

    Returns (frame_data, remaining_buffer) where frame_data is 9 bytes
    or None if no complete frame found.
    """
    while len(buffer) >= TFMINI_FRAME_SIZE:
        # Look for header pair 0x59 0x59
        idx = buffer.find(bytes([TFMINI_HEADER, TFMINI_HEADER]))
        if idx < 0:
            # No header found, discard all but last byte
            return None, buffer[-1:] if buffer else b''
        if idx > 0:
            buffer = buffer[idx:]
        if len(buffer) < TFMINI_FRAME_SIZE:
            break

        frame = buffer[:TFMINI_FRAME_SIZE]
        parsed = parse_tfmini_frame(frame)
        if parsed['valid']:
            return frame, buffer[TFMINI_FRAME_SIZE:]
        else:
            # Bad frame at this position, skip header and try again
            buffer = buffer[2:]

    return None, buffer


def distance_to_threat_level(
    distance_m: float,
    dist_emergency: float = 2.0,
    dist_critical: float = 4.0,
    dist_warning: float = 8.0,
    dist_caution: float = 12.0,
) -> int:
    """Convert a distance reading to a threat level (0-5)."""
    if distance_m <= dist_emergency:
        return 5  # EMERGENCY
    elif distance_m <= dist_critical:
        return 4  # CRITICAL
    elif distance_m <= dist_warning:
        return 3  # WARNING
    elif distance_m <= dist_caution:
        return 2  # CAUTION
    else:
        return 1  # MONITOR


# Sector label to bearing mapping (same as obstacle_detector_node)
SECTOR_BEARINGS = {
    'FRONT': 0.0,
    'FRONT_RIGHT': 45.0,
    'RIGHT': 90.0,
    'REAR_RIGHT': 135.0,
    'REAR': 180.0,
    'REAR_LEFT': -135.0,
    'LEFT': -90.0,
    'FRONT_LEFT': -45.0,
    'TOP': 0.0,
    'BOTTOM': 0.0,
}


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class RangeSensorNode(Node):

    def __init__(self):
        super().__init__('range_sensor')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('sensor_sector', 'FRONT')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('max_range_m', 12.0)
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('reading_timeout_s', 0.5)
        self.declare_parameter('distance_emergency_m', 2.0)
        self.declare_parameter('distance_critical_m', 4.0)
        self.declare_parameter('distance_warning_m', 8.0)
        self.declare_parameter('distance_caution_m', 12.0)

        self._port = self.get_parameter('serial_port').value
        self._baud = self.get_parameter('baud_rate').value
        self._sector = self.get_parameter('sensor_sector').value
        self._rate = self.get_parameter('publish_rate_hz').value
        self._max_range = self.get_parameter('max_range_m').value
        self._min_range = self.get_parameter('min_range_m').value
        self._reading_timeout = self.get_parameter('reading_timeout_s').value
        self._dist_emergency = self.get_parameter('distance_emergency_m').value
        self._dist_critical = self.get_parameter('distance_critical_m').value
        self._dist_warning = self.get_parameter('distance_warning_m').value
        self._dist_caution = self.get_parameter('distance_caution_m').value

        # State
        self._serial = None
        self._buffer = b''
        self._last_distance = -1.0
        self._last_strength = 0
        self._last_reading_time_ns = 0  # ROS clock nanoseconds
        self._is_in_air = False
        self._prev_distance = -1.0
        self._prev_time_ns = 0

        # Publishers -- publishes to /range subtopic, merged by obstacle_merger_node
        self._pub_obstacles = self.create_publisher(
            ObstacleArray, '/wardrone/obstacles/range', 10)
        self._pub_range = self.create_publisher(
            Float32, '/wardrone/range_sensor/distance', 10)
        self._pub_safety = self.create_publisher(
            String, '/wardrone/safety/event', 10)

        # Subscribers
        self.create_subscription(
            Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)
        # Accept simulated range input (for SITL without physical sensor)
        self.create_subscription(
            Float32, '/wardrone/range_sensor/simulated', self._on_simulated, 10)

        # Open serial port
        self._open_serial()

        # Timer
        self.create_timer(1.0 / self._rate, self._read_and_publish)

        self.get_logger().info(
            f'Range Sensor ready: port={self._port}, sector={self._sector}, '
            f'range={self._min_range}-{self._max_range}m'
        )

    def _open_serial(self):
        """Try to open the serial port. Non-fatal if it fails (SITL mode)."""
        try:
            import serial
            self._serial = serial.Serial(
                self._port, self._baud, timeout=0.01)
            self.get_logger().info(f'Serial port {self._port} opened')
        except Exception as e:
            self._serial = None
            self.get_logger().warn(
                f'Serial port {self._port} not available: {e}. '
                f'Using simulated input on /wardrone/range_sensor/simulated'
            )

    def _on_telemetry(self, msg: Telemetry):
        self._is_in_air = msg.is_in_air

    def _on_simulated(self, msg: Float32):
        """Accept simulated distance readings (for SITL)."""
        self._last_distance = msg.data
        self._last_strength = 999  # Simulated = always strong
        self._last_reading_time_ns = self.get_clock().now().nanoseconds

    def _read_and_publish(self):
        """Read serial data, parse frames, publish obstacle."""
        now_ns = self.get_clock().now().nanoseconds

        # Read from serial if available
        if self._serial is not None:
            try:
                available = self._serial.in_waiting
                if available > 0:
                    raw = self._serial.read(min(available, 256))
                    self._buffer += raw

                    # Parse all complete frames in buffer
                    while True:
                        frame, self._buffer = find_tfmini_frame(self._buffer)
                        if frame is None:
                            break
                        parsed = parse_tfmini_frame(frame)
                        if parsed['valid']:
                            self._last_distance = parsed['distance_m']
                            self._last_strength = parsed['strength']
                            self._last_reading_time_ns = now_ns
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')

        # Check reading staleness -- invalidate if no fresh data
        if self._last_reading_time_ns > 0:
            age_s = (now_ns - self._last_reading_time_ns) / 1e9
            if age_s > self._reading_timeout:
                self._last_distance = -1.0
                self._prev_distance = -1.0
                return

        # Nothing to publish
        if self._last_distance < 0:
            return

        # Publish raw distance
        range_msg = Float32()
        range_msg.data = self._last_distance
        self._pub_range.publish(range_msg)

        # Only publish obstacles when in air
        if not self._is_in_air:
            return

        # Clamp to valid range
        distance = self._last_distance
        if distance < self._min_range or distance > self._max_range:
            return

        # Estimate approach velocity from consecutive readings (ROS clock)
        approach_vel = 0.0
        ttc = -1.0
        if self._prev_distance > 0 and self._prev_time_ns > 0:
            dt = (now_ns - self._prev_time_ns) / 1e9
            if dt > 0.01:
                # Positive = approaching (distance decreasing)
                approach_vel = (self._prev_distance - distance) / dt
                if approach_vel > 0.3:
                    ttc = distance / approach_vel
                else:
                    approach_vel = max(approach_vel, 0.0)
        self._prev_distance = distance
        self._prev_time_ns = now_ns

        # Build obstacle message
        bearing = SECTOR_BEARINGS.get(self._sector, 0.0)
        threat = distance_to_threat_level(
            distance, self._dist_emergency, self._dist_critical,
            self._dist_warning, self._dist_caution)

        obs = Obstacle()
        obs.header.stamp = self.get_clock().now().to_msg()
        obs.header.frame_id = 'base_link'
        obs.sector = self._sector
        obs.bearing_deg = bearing
        obs.estimated_distance_m = distance
        obs.approach_velocity_m_s = approach_vel
        obs.time_to_collision_s = ttc
        obs.classification = 'unknown'
        obs.classification_confidence = 0.0
        obs.threat_level = threat

        arr = ObstacleArray()
        arr.header.stamp = obs.header.stamp
        arr.header.frame_id = 'base_link'
        arr.obstacles = [obs]
        arr.active_sectors = 1
        arr.max_threat_level = threat
        arr.emergency_detected = (threat >= 5)
        self._pub_obstacles.publish(arr)

        # Safety event for high threats
        if threat >= 4:
            event = String()
            event.data = f'RANGE_OBSTACLE_{"EMERGENCY" if threat >= 5 else "CRITICAL"}'
            self._pub_safety.publish(event)

    def destroy_node(self):
        if self._serial is not None:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RangeSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
