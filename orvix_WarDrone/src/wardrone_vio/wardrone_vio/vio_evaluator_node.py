"""VIO Evaluator Node (simulation only).

Compares the VIO pose estimate against the Gazebo ground truth to measure
drift and accuracy. Publishes drift metrics for logging and evaluation.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from std_msgs.msg import Float64


class VioEvaluatorNode(Node):

    def __init__(self):
        super().__init__('vio_evaluator')

        # Parameters
        self.declare_parameter('evaluation_rate_hz', 1.0)
        self.declare_parameter('ground_truth_topic', '/gazebo/ground_truth/pose')

        eval_rate = self.get_parameter('evaluation_rate_hz').value
        gt_topic = self.get_parameter('ground_truth_topic').value

        # State
        self._vio_pose = None
        self._gt_pose = None
        self._start_time = time.time()
        self._drift_history = []

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self._pub_drift = self.create_publisher(Vector3Stamped, '/wardrone/vio/drift', 10)
        self._pub_drift_rate = self.create_publisher(Float64, '/wardrone/vio/drift_rate', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/wardrone/vio/pose', self._on_vio_pose, 10)
        self.create_subscription(PoseStamped, gt_topic, self._on_ground_truth, sensor_qos)

        # Evaluation timer
        self.create_timer(1.0 / eval_rate, self._evaluate_tick)

        self.get_logger().info(f'VIO Evaluator ready (ground truth: {gt_topic})')

    def _on_vio_pose(self, msg: PoseStamped):
        self._vio_pose = msg

    def _on_ground_truth(self, msg: PoseStamped):
        self._gt_pose = msg

    def _evaluate_tick(self):
        if self._vio_pose is None or self._gt_pose is None:
            return

        # Compute position error
        dx = self._vio_pose.pose.position.x - self._gt_pose.pose.position.x
        dy = self._vio_pose.pose.position.y - self._gt_pose.pose.position.y
        dz = self._vio_pose.pose.position.z - self._gt_pose.pose.position.z
        total_error = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Publish drift vector
        drift_msg = Vector3Stamped()
        drift_msg.header.stamp = self.get_clock().now().to_msg()
        drift_msg.vector.x = dx
        drift_msg.vector.y = dy
        drift_msg.vector.z = dz
        self._pub_drift.publish(drift_msg)

        # Compute drift rate (m/min)
        elapsed_min = (time.time() - self._start_time) / 60.0
        self._drift_history.append(total_error)

        drift_rate = 0.0
        if elapsed_min > 0.0:
            drift_rate = total_error / elapsed_min

        rate_msg = Float64()
        rate_msg.data = drift_rate
        self._pub_drift_rate.publish(rate_msg)

        self.get_logger().info(
            f'VIO drift: [{dx:.3f}, {dy:.3f}, {dz:.3f}]m, '
            f'total={total_error:.3f}m, rate={drift_rate:.3f}m/min'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VioEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
