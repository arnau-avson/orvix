"""Detector node.

Subscribes to camera images, runs YOLO inference using yolo_wrapper,
and publishes detection arrays.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from wardrone_interfaces.msg import Detection, DetectionArray

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

from wardrone_vision.yolo_wrapper import YoloWrapper


class DetectorNode(Node):

    def __init__(self):
        super().__init__('detector')

        # Parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('target_classes', [0, 2, 5, 7])
        self.declare_parameter('inference_size', 640)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('max_detections_per_frame', 20)

        model_path = self.get_parameter('model_path').value
        conf_thresh = self.get_parameter('confidence_threshold').value
        target_classes = list(self.get_parameter('target_classes').value)
        inf_size = self.get_parameter('inference_size').value
        device = self.get_parameter('device').value
        self._publish_debug = self.get_parameter('publish_debug_image').value
        self._max_dets = self.get_parameter('max_detections_per_frame').value

        # YOLO wrapper
        self._yolo = YoloWrapper(
            model_path=model_path,
            confidence_threshold=conf_thresh,
            target_classes=target_classes,
            inference_size=inf_size,
            device=device,
        )

        # CV Bridge
        self._bridge = CvBridge() if CvBridge is not None else None

        # Publishers
        self._pub_detections = self.create_publisher(DetectionArray, '/wardrone/detections', 10)
        if self._publish_debug:
            self._pub_debug = self.create_publisher(Image, '/wardrone/debug/detection_image', 10)

        # Subscriber
        self.create_subscription(Image, '/wardrone/camera/image_raw', self._on_image, 10)

        # Load model
        self.get_logger().info(f'Loading YOLO model: {model_path} on {device}...')
        try:
            self._yolo.load_model()
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')

        self.get_logger().info('Detector node ready')

    def _on_image(self, msg: Image):
        """Process incoming camera image."""
        if self._bridge is None:
            self.get_logger().warn('cv_bridge not available, cannot process images')
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Run detection
        try:
            results = self._yolo.detect(frame)
        except RuntimeError as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        # Limit detections
        results = results[:self._max_dets]

        # Build DetectionArray message
        det_array = DetectionArray()
        det_array.header = msg.header
        det_array.image_width = msg.width
        det_array.image_height = msg.height

        for det in results:
            d = Detection()
            d.header = msg.header
            d.class_id = det.class_id
            d.class_name = det.class_name
            d.confidence = det.confidence
            d.x1 = det.x1
            d.y1 = det.y1
            d.x2 = det.x2
            d.y2 = det.y2
            d.cx = det.cx
            d.cy = det.cy
            d.track_id = -1  # Not yet tracked
            det_array.detections.append(d)

        self._pub_detections.publish(det_array)

        # Debug image
        if self._publish_debug and len(results) > 0:
            annotated = YoloWrapper.draw_detections(frame, results)
            debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            debug_msg.header = msg.header
            self._pub_debug.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
