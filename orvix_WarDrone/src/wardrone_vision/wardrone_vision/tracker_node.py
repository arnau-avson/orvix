"""Tracker node.

Subscribes to detections, maintains multi-frame tracking using ObjectTracker,
computes velocity commands for lock-on pursuit, and publishes tracking state.

Lock-on strategy:
- Yaw: PID controller on horizontal offset of target center from image center
- Forward: PID controller on target size ratio vs desired size (distance proxy)
- Altitude: PID controller on vertical offset (optional)
- Search: When target is lost, rotate in place at configurable yaw rate
"""

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import CameraInfo
from wardrone_interfaces.msg import DetectionArray, TrackingTarget
from wardrone_interfaces.srv import SetTrackingTarget

from wardrone_vision.object_tracker import ObjectTracker


class PIDController:
    """Simple PID controller."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = -1.0, output_max: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float = 0.1) -> float:
        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0


class TrackerNode(Node):

    # Tracking states
    STATE_SEARCHING = "SEARCHING"
    STATE_TRACKING = "TRACKING"
    STATE_LOCKED = "LOCKED"
    STATE_LOST = "LOST"

    def __init__(self):
        super().__init__('tracker')

        # Parameters
        self.declare_parameter('tracker_type', 'sort')
        self.declare_parameter('max_lost_frames', 30)
        self.declare_parameter('lock_on_iou_threshold', 0.3)
        self.declare_parameter('pursuit_speed_m_s', 3.0)
        self.declare_parameter('pursuit_altitude_m', 8.0)
        self.declare_parameter('pid_yaw_kp', 0.5)
        self.declare_parameter('pid_yaw_ki', 0.0)
        self.declare_parameter('pid_yaw_kd', 0.1)
        self.declare_parameter('pid_forward_kp', 0.3)
        self.declare_parameter('pid_forward_ki', 0.0)
        self.declare_parameter('pid_forward_kd', 0.05)
        self.declare_parameter('search_yaw_rate_deg_s', 30.0)
        self.declare_parameter('target_size_ratio_min', 0.01)
        self.declare_parameter('target_size_ratio_max', 0.4)

        max_lost = self.get_parameter('max_lost_frames').value
        iou_thresh = self.get_parameter('lock_on_iou_threshold').value
        self._pursuit_speed = self.get_parameter('pursuit_speed_m_s').value
        self._search_yaw_rate = self.get_parameter('search_yaw_rate_deg_s').value
        self._size_ratio_min = self.get_parameter('target_size_ratio_min').value
        self._size_ratio_max = self.get_parameter('target_size_ratio_max').value

        # Object tracker
        self._tracker = ObjectTracker(
            iou_threshold=iou_thresh,
            max_lost_frames=max_lost,
        )

        # PID controllers
        self._pid_yaw = PIDController(
            kp=self.get_parameter('pid_yaw_kp').value,
            ki=self.get_parameter('pid_yaw_ki').value,
            kd=self.get_parameter('pid_yaw_kd').value,
            output_min=-math.radians(self._search_yaw_rate),
            output_max=math.radians(self._search_yaw_rate),
        )
        self._pid_forward = PIDController(
            kp=self.get_parameter('pid_forward_kp').value,
            ki=self.get_parameter('pid_forward_ki').value,
            kd=self.get_parameter('pid_forward_kd').value,
            output_min=-self._pursuit_speed,
            output_max=self._pursuit_speed,
        )

        # State
        self._state = self.STATE_SEARCHING
        self._target_class_id = 2  # Default: car
        self._target_track_id = -1  # -1 = any of class
        self._image_width = 640
        self._image_height = 480
        self._desired_size_ratio = 0.08  # Target should occupy ~8% of image

        # Publishers
        self._pub_target = self.create_publisher(TrackingTarget, '/wardrone/tracking/target', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/wardrone/cmd_velocity', 10)
        self._pub_state = self.create_publisher(String, '/wardrone/tracking/state', 10)

        # Subscribers
        self.create_subscription(DetectionArray, '/wardrone/detections', self._on_detections, 10)
        self.create_subscription(CameraInfo, '/wardrone/camera/camera_info', self._on_camera_info, 10)

        # Service
        self.create_service(SetTrackingTarget, '/wardrone/set_tracking_target', self._handle_set_target)

        # Control loop timer (10 Hz)
        self.create_timer(0.1, self._control_tick)

        self.get_logger().info('Tracker node ready')

    def _on_camera_info(self, msg: CameraInfo):
        self._image_width = msg.width
        self._image_height = msg.height

    def _on_detections(self, msg: DetectionArray):
        """Process new detections and update tracker."""
        self._image_width = msg.image_width if msg.image_width > 0 else self._image_width
        self._image_height = msg.image_height if msg.image_height > 0 else self._image_height

        # Convert to tracker format
        det_list = []
        for d in msg.detections:
            det_list.append({
                'x1': d.x1, 'y1': d.y1, 'x2': d.x2, 'y2': d.y2,
                'class_id': d.class_id, 'class_name': d.class_name,
                'confidence': d.confidence,
            })

        self._tracker.update(det_list)

    def _handle_set_target(self, request, response):
        self._target_class_id = request.target_class_id
        self._target_track_id = request.target_track_id
        self._pid_yaw.reset()
        self._pid_forward.reset()
        response.success = True
        response.message = f'Target set: class={request.target_class_id}, track={request.target_track_id}'
        self.get_logger().info(response.message)
        return response

    def _control_tick(self):
        """Main control loop: find target, compute commands, publish."""
        # Find target track
        target_track = None
        if self._target_track_id >= 0:
            target_track = self._tracker.get_track_by_id(self._target_track_id)
        if target_track is None:
            target_track = self._tracker.get_best_track(class_id=self._target_class_id)

        # Update state
        prev_state = self._state
        if target_track is None:
            if self._state in (self.STATE_TRACKING, self.STATE_LOCKED):
                self._state = self.STATE_LOST
            elif self._state == self.STATE_LOST:
                self._state = self.STATE_SEARCHING
                self._pid_yaw.reset()
                self._pid_forward.reset()
        else:
            if target_track.frames_since_detection == 0:
                if target_track.total_frames > 5:
                    self._state = self.STATE_LOCKED
                else:
                    self._state = self.STATE_TRACKING
            else:
                self._state = self.STATE_TRACKING

        # Publish state change
        if self._state != prev_state:
            state_msg = String()
            state_msg.data = self._state
            self._pub_state.publish(state_msg)
            self.get_logger().info(f'Tracking state: {prev_state} -> {self._state}')

        # Publish tracking target
        target_msg = TrackingTarget()
        target_msg.header.stamp = self.get_clock().now().to_msg()

        if target_track is not None:
            target_msg.is_tracking = True
            target_msg.track_id = target_track.track_id
            target_msg.class_id = target_track.class_id
            target_msg.class_name = target_track.class_name
            target_msg.confidence = target_track.confidence
            target_msg.x1 = target_track.bbox.x1
            target_msg.y1 = target_track.bbox.y1
            target_msg.x2 = target_track.bbox.x2
            target_msg.y2 = target_track.bbox.y2

            # Compute offset from center (normalized -1 to 1)
            cx = target_track.bbox.cx
            cy = target_track.bbox.cy
            target_msg.offset_x = (cx - self._image_width / 2.0) / (self._image_width / 2.0)
            target_msg.offset_y = (cy - self._image_height / 2.0) / (self._image_height / 2.0)

            # Size ratio
            image_area = self._image_width * self._image_height
            target_msg.size_ratio = target_track.bbox.area / image_area if image_area > 0 else 0.0

            target_msg.frames_since_detection = target_track.frames_since_detection
        else:
            target_msg.is_tracking = False
            target_msg.track_id = -1

        self._pub_target.publish(target_msg)

        # Compute velocity command
        cmd = Twist()

        if self._state in (self.STATE_TRACKING, self.STATE_LOCKED) and target_track is not None:
            # Yaw: turn toward target
            yaw_error = -target_msg.offset_x  # Negative because positive offset = turn right
            cmd.angular.z = self._pid_yaw.compute(yaw_error)

            # Forward: adjust distance based on size ratio
            size_error = self._desired_size_ratio - target_msg.size_ratio
            cmd.linear.x = self._pid_forward.compute(size_error)

            # Clamp forward speed
            cmd.linear.x = max(0.0, min(self._pursuit_speed, cmd.linear.x))

        elif self._state == self.STATE_SEARCHING:
            # Rotate in place to search
            cmd.angular.z = math.radians(self._search_yaw_rate)
            cmd.linear.x = 0.0

        self._pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
