"""VIO Bridge Node.

Subscribes to a Visual-Inertial Odometry source (VINS-Fusion, or Gazebo ground truth
in simulation) and republishes the pose estimate for the MAVSDK bridge to relay
to PX4's EKF2 as a vision position estimate.

The node handles frame transformations between the VIO frame and the drone body frame.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from wardrone_interfaces.msg import VioStatus


class VioBridgeNode(Node):

    def __init__(self):
        super().__init__('vio_bridge')

        # Parameters
        self.declare_parameter('vio_source', 'vins_fusion')
        self.declare_parameter('vio_input_topic', '/vins_fusion/odometry')
        self.declare_parameter('enable_sim_ground_truth', False)
        self.declare_parameter('sim_ground_truth_topic', '/gazebo/ground_truth/pose')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('body_to_camera_transform', [0.1, 0.0, -0.05, 0.0, 0.0, 0.0])

        self._vio_source = self.get_parameter('vio_source').value
        self._use_ground_truth = self.get_parameter('enable_sim_ground_truth').value
        self._body_to_cam = self.get_parameter('body_to_camera_transform').value
        publish_rate = self.get_parameter('publish_rate_hz').value

        # State
        self._latest_pose = None
        self._tracking = False
        self._feature_count = 0
        self._pose_count = 0

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self._pub_vio_pose = self.create_publisher(PoseStamped, '/wardrone/vio/pose', 10)
        self._pub_vio_status = self.create_publisher(VioStatus, '/wardrone/vio/status', 10)

        # Subscribers - choose source
        if self._use_ground_truth:
            gt_topic = self.get_parameter('sim_ground_truth_topic').value
            self.create_subscription(
                PoseStamped, gt_topic, self._on_ground_truth, sensor_qos
            )
            self.get_logger().info(f'VIO Bridge using ground truth from: {gt_topic}')
        else:
            vio_topic = self.get_parameter('vio_input_topic').value
            self.create_subscription(
                Odometry, vio_topic, self._on_vio_odometry, sensor_qos
            )
            self.get_logger().info(f'VIO Bridge using VIO from: {vio_topic}')

        # Status timer
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info(f'VIO Bridge ready (source={self._vio_source})')

    def _on_vio_odometry(self, msg: Odometry):
        """Handle VIO odometry from VINS-Fusion or similar."""
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.header.frame_id = 'map'

        # Apply body-to-camera transform compensation
        pose_msg.pose.position.x = msg.pose.pose.position.x - self._body_to_cam[0]
        pose_msg.pose.position.y = msg.pose.pose.position.y - self._body_to_cam[1]
        pose_msg.pose.position.z = msg.pose.pose.position.z - self._body_to_cam[2]
        pose_msg.pose.orientation = msg.pose.pose.orientation

        self._latest_pose = pose_msg
        self._tracking = True
        self._pose_count += 1

        self._pub_vio_pose.publish(pose_msg)

    def _on_ground_truth(self, msg: PoseStamped):
        """Handle ground truth pose from Gazebo (simulation only)."""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose

        self._latest_pose = pose_msg
        self._tracking = True
        self._pose_count += 1

        self._pub_vio_pose.publish(pose_msg)

    def _publish_status(self):
        """Publish VIO health status at 1 Hz."""
        status = VioStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.is_tracking = self._tracking
        status.feature_count = self._feature_count
        status.tracking_confidence = 1.0 if self._tracking else 0.0
        status.drift_estimate_m = 0.0  # Would need evaluator for real estimate
        status.source = 'ground_truth' if self._use_ground_truth else self._vio_source

        self._pub_vio_status.publish(status)

        # Reset tracking flag if no poses received recently
        if self._pose_count == 0:
            self._tracking = False
        self._pose_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = VioBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
