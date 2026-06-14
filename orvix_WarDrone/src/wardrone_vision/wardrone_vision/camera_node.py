"""Camera node.

Bridges camera sources (Gazebo simulation or real V4L2/CSI) and publishes
raw image frames and camera info for the detection pipeline.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera')

        # Parameters
        self.declare_parameter('source', 'gazebo')
        self.declare_parameter('gazebo_image_topic',
                              '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image')
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        self._source = self.get_parameter('source').value
        self._width = self.get_parameter('width').value
        self._height = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value

        # Publishers
        self._pub_image = self.create_publisher(Image, '/wardrone/camera/image_raw', 10)
        self._pub_info = self.create_publisher(CameraInfo, '/wardrone/camera/camera_info', 10)

        if self._source == 'gazebo':
            gz_topic = self.get_parameter('gazebo_image_topic').value
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
            self.create_subscription(Image, gz_topic, self._on_gazebo_image, sensor_qos)
            self.get_logger().info(f'Camera bridging Gazebo topic: {gz_topic}')
        elif self._source in ('v4l2', 'csi'):
            self._cap = None
            self._start_capture()
        else:
            self.get_logger().error(f'Unknown camera source: {self._source}')

    def _start_capture(self):
        """Start V4L2/CSI camera capture using OpenCV."""
        import cv2
        from cv_bridge import CvBridge

        self._bridge = CvBridge()
        device_id = self.get_parameter('device_id').value
        self._cap = cv2.VideoCapture(device_id)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)

        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device_id}')
            return

        self.get_logger().info(f'Camera opened: device {device_id} ({self._width}x{self._height}@{self._fps}fps)')

        # Capture timer
        self.create_timer(1.0 / self._fps, self._capture_frame)

    def _capture_frame(self):
        """Read a frame from the local camera and publish."""
        if self._cap is None or not self._cap.isOpened():
            return

        ret, frame = self._cap.read()
        if not ret:
            return

        img_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera'
        self._pub_image.publish(img_msg)

        self._publish_camera_info(img_msg.header)

    def _on_gazebo_image(self, msg: Image):
        """Forward Gazebo camera images to wardrone topic."""
        msg.header.frame_id = 'camera'
        self._pub_image.publish(msg)
        self._publish_camera_info(msg.header)

    def _publish_camera_info(self, header):
        """Publish camera intrinsics (simplified pinhole model)."""
        info = CameraInfo()
        info.header = header
        info.width = self._width
        info.height = self._height

        # Approximate focal length for 90 degree FOV
        fx = self._width / 2.0
        fy = self._height / 2.0
        cx = self._width / 2.0
        cy = self._height / 2.0

        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self._pub_info.publish(info)

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
