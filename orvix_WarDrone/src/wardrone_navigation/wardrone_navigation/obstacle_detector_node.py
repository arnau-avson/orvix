"""Obstacle detector node -- multi-camera omnidirectional obstacle detection.

Processes images from up to 8 camera sectors (front, front-right, right, rear-right,
rear, rear-left, left, front-left) to detect obstacles using a combination of:

1. **Motion detection**: Background subtraction + contour analysis on all cameras
   for fast, lightweight obstacle detection.
2. **YOLO classification**: On-demand classification of detected motion regions
   to identify the type of obstacle (bird, drone, vehicle, etc.).
3. **Approach velocity estimation**: Tracks bounding box expansion rate across
   frames to estimate time-to-collision (TTC).

Hardware recommendation (economic):
- 8x OV5647 camera modules (~5 EUR each, 62deg FOV) = ~40 EUR total
- OR 4x IMX219 wide-angle 160deg (~12 EUR each) = ~48 EUR for 4 sectors + diagonals
- Connected via USB webcam adapters or CSI multiplexer (e.g. Arducam multi-camera)
- Raspberry Pi 4 / Jetson Nano for processing

Camera topic convention:
    /wardrone/obstacle_cam/<sector>/image_raw
where <sector> is one of: front, front_right, right, rear_right, rear,
rear_left, left, front_left
"""

import time
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from wardrone_navigation.kalman_tracker import (
    KalmanBoxTracker, compute_iou, associate_detections, x_to_bbox,
)

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from sensor_msgs.msg import Image
from std_msgs.msg import String
from wardrone_interfaces.msg import Obstacle, ObstacleArray, Telemetry

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    import cv2
except ImportError:
    cv2 = None


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SECTORS = [
    'front', 'front_right', 'right', 'rear_right',
    'rear', 'rear_left', 'left', 'front_left',
    'top', 'bottom',
]

# Bearing angle for each sector center (degrees, 0 = front, CW positive)
# For top/bottom sectors, bearing is 0.0 (not meaningful; avoidance uses sector label)
SECTOR_BEARINGS = {
    'front': 0.0,
    'front_right': 45.0,
    'right': 90.0,
    'rear_right': 135.0,
    'rear': 180.0,
    'rear_left': -135.0,
    'left': -90.0,
    'front_left': -45.0,
    'top': 0.0,
    'bottom': 0.0,
}

# Sector name -> Obstacle.msg sector string
SECTOR_LABELS = {
    'front': 'FRONT',
    'front_right': 'FRONT_RIGHT',
    'right': 'RIGHT',
    'rear_right': 'REAR_RIGHT',
    'rear': 'REAR',
    'rear_left': 'REAR_LEFT',
    'left': 'LEFT',
    'front_left': 'FRONT_LEFT',
    'top': 'TOP',
    'bottom': 'BOTTOM',
}

# Threat level thresholds (distance in meters)
THREAT_NONE = 0
THREAT_MONITOR = 1
THREAT_CAUTION = 2
THREAT_WARNING = 3
THREAT_CRITICAL = 4
THREAT_EMERGENCY = 5


# ---------------------------------------------------------------------------
# Pure functions (testable without rclpy)
# ---------------------------------------------------------------------------

def compute_distance_confidence(
    frames_tracked: int,
    classification_conf: float,
    kalman_innovation_area: float,
    kalman_predicted_area: float,
    min_confident_frames: int = 10,
) -> float:
    """Compute confidence in the monocular distance estimate.

    Combines three signals:
    - Track maturity: min(1.0, frames_tracked / min_confident_frames)
    - Classification confidence: direct from YOLO (0-1), floored at 0.1
    - Area stability: 1 - |innovation| / predicted_area, clamped [0, 1]

    Returns: float in [0.0, 1.0].
    """
    maturity = min(1.0, frames_tracked / min_confident_frames)

    if kalman_predicted_area > 1.0:
        stability = 1.0 - min(1.0, abs(kalman_innovation_area) / kalman_predicted_area)
    else:
        stability = 0.0

    conf = maturity * max(classification_conf, 0.1) * stability
    return max(0.0, min(1.0, conf))


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class TrackedContour:
    """A contour tracked across frames for approach velocity estimation."""
    contour_id: int
    sector: str
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    area: float
    center: Tuple[int, int]
    # History of (timestamp, area, center) for velocity estimation
    history: deque = field(default_factory=lambda: deque(maxlen=30))
    classification: str = 'unknown'
    classification_conf: float = 0.0
    last_seen: float = 0.0
    frames_tracked: int = 0
    kalman: Optional[KalmanBoxTracker] = None


@dataclass
class SectorState:
    """Per-sector processing state."""
    prev_frame_gray: Optional[np.ndarray] = None
    bg_subtractor: object = None  # cv2.BackgroundSubtractor
    tracked_contours: Dict[int, TrackedContour] = field(default_factory=dict)
    next_contour_id: int = 0
    last_frame_time: float = 0.0
    active: bool = False


class ObstacleDetectorNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector')

        # --- Parameters ---
        self.declare_parameter('enabled_sectors', list(SECTORS))
        self.declare_parameter('detection_rate_hz', 10.0)
        self.declare_parameter('min_contour_area_px', 500)
        self.declare_parameter('max_contours_per_sector', 10)
        self.declare_parameter('motion_threshold', 25)
        self.declare_parameter('bg_subtractor_history', 100)
        self.declare_parameter('bg_subtractor_var_threshold', 50.0)
        self.declare_parameter('approach_velocity_window_s', 1.0)
        self.declare_parameter('ttc_emergency_s', 2.0)
        self.declare_parameter('ttc_critical_s', 4.0)
        self.declare_parameter('ttc_warning_s', 6.0)
        self.declare_parameter('ttc_caution_s', 10.0)
        self.declare_parameter('distance_emergency_m', 3.0)
        self.declare_parameter('distance_critical_m', 6.0)
        self.declare_parameter('distance_warning_m', 12.0)
        self.declare_parameter('distance_caution_m', 25.0)
        self.declare_parameter('contour_lost_timeout_s', 1.0)
        self.declare_parameter('enable_yolo_classification', True)
        self.declare_parameter('yolo_model_path', 'yolo11n.pt')
        self.declare_parameter('yolo_confidence_threshold', 0.35)
        self.declare_parameter('yolo_device', 'cpu')
        # Camera intrinsics for distance estimation (horizontal FOV in degrees)
        self.declare_parameter('camera_hfov_deg', 62.0)
        self.declare_parameter('camera_resolution_w', 640)
        self.declare_parameter('camera_resolution_h', 480)
        # Known object sizes for monocular distance estimation (meters)
        self.declare_parameter('known_sizes.bird', 0.3)
        self.declare_parameter('known_sizes.drone', 0.5)
        self.declare_parameter('known_sizes.person', 0.5)
        self.declare_parameter('known_sizes.car', 2.0)
        self.declare_parameter('known_sizes.truck', 3.0)
        self.declare_parameter('known_sizes.unknown', 0.5)

        # Read parameters
        self._enabled_sectors = self.get_parameter('enabled_sectors').value
        self._detection_rate = self.get_parameter('detection_rate_hz').value
        self._min_contour_area = self.get_parameter('min_contour_area_px').value
        self._max_contours = self.get_parameter('max_contours_per_sector').value
        self._motion_threshold = self.get_parameter('motion_threshold').value
        self._bg_history = self.get_parameter('bg_subtractor_history').value
        self._bg_var_threshold = self.get_parameter('bg_subtractor_var_threshold').value
        self._velocity_window = self.get_parameter('approach_velocity_window_s').value
        self._ttc_emergency = self.get_parameter('ttc_emergency_s').value
        self._ttc_critical = self.get_parameter('ttc_critical_s').value
        self._ttc_warning = self.get_parameter('ttc_warning_s').value
        self._ttc_caution = self.get_parameter('ttc_caution_s').value
        self._dist_emergency = self.get_parameter('distance_emergency_m').value
        self._dist_critical = self.get_parameter('distance_critical_m').value
        self._dist_warning = self.get_parameter('distance_warning_m').value
        self._dist_caution = self.get_parameter('distance_caution_m').value
        self._contour_lost_timeout = self.get_parameter('contour_lost_timeout_s').value
        self._enable_yolo = self.get_parameter('enable_yolo_classification').value
        self._yolo_model_path = self.get_parameter('yolo_model_path').value
        self._yolo_conf = self.get_parameter('yolo_confidence_threshold').value
        self._yolo_device = self.get_parameter('yolo_device').value
        self._cam_hfov = self.get_parameter('camera_hfov_deg').value
        self._cam_w = self.get_parameter('camera_resolution_w').value
        self._cam_h = self.get_parameter('camera_resolution_h').value

        self._known_sizes = {
            'bird': self.get_parameter('known_sizes.bird').value,
            'drone': self.get_parameter('known_sizes.drone').value,
            'person': self.get_parameter('known_sizes.person').value,
            'car': self.get_parameter('known_sizes.car').value,
            'truck': self.get_parameter('known_sizes.truck').value,
            'unknown': self.get_parameter('known_sizes.unknown').value,
        }

        # Focal length in pixels (from horizontal FOV)
        self._focal_px = (self._cam_w / 2.0) / math.tan(math.radians(self._cam_hfov / 2.0))

        # --- State ---
        self._sector_states: Dict[str, SectorState] = {}
        self._bridge = CvBridge() if CvBridge is not None else None
        self._yolo = None
        self._is_armed = False
        self._is_in_air = False

        cb_group = ReentrantCallbackGroup()

        # --- Publishers ---
        self._pub_obstacles = self.create_publisher(ObstacleArray, '/wardrone/obstacles/vision', 10)
        self._pub_event = self.create_publisher(String, '/wardrone/safety/event', 10)
        self._pub_debug = self.create_publisher(Image, '/wardrone/debug/obstacle_image', 10)

        # --- Subscribers ---
        self.create_subscription(Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)

        # Subscribe to each enabled camera sector
        for sector in self._enabled_sectors:
            if sector not in SECTORS:
                self.get_logger().warn(f'Unknown sector: {sector}, skipping')
                continue
            topic = f'/wardrone/obstacle_cam/{sector}/image_raw'
            self._sector_states[sector] = SectorState()
            if cv2 is not None:
                self._sector_states[sector].bg_subtractor = (
                    cv2.createBackgroundSubtractorMOG2(
                        history=self._bg_history,
                        varThreshold=self._bg_var_threshold,
                        detectShadows=False,
                    )
                )
            self.create_subscription(
                Image, topic,
                lambda msg, s=sector: self._on_sector_image(msg, s),
                10,
                callback_group=cb_group,
            )
            self.get_logger().info(f'Subscribed to obstacle camera: {topic}')

        # --- YOLO classifier (lazy-loaded) ---
        if self._enable_yolo:
            self._load_yolo()

        # --- Processing timer ---
        self._detection_timer = self.create_timer(
            1.0 / self._detection_rate, self._process_tick
        )

        n_sectors = len(self._sector_states)
        self.get_logger().info(
            f'Obstacle Detector ready: {n_sectors} sectors, '
            f'YOLO={"on" if self._enable_yolo else "off"}, '
            f'rate={self._detection_rate}Hz'
        )

    # ------------------------------------------------------------------
    # YOLO setup
    # ------------------------------------------------------------------

    def _load_yolo(self):
        """Load YOLO model for obstacle classification."""
        try:
            from wardrone_vision.yolo_wrapper import YoloWrapper
            # Obstacle-relevant COCO classes:
            # 0=person, 1=bicycle, 2=car, 3=motorcycle, 5=bus, 7=truck,
            # 14=bird, 15=cat, 16=dog, 8=boat, 9=traffic light,
            # 10=fire hydrant, 11=stop sign, 13=bench
            obstacle_classes = [0, 1, 2, 3, 5, 7, 8, 14, 15, 16]
            self._yolo = YoloWrapper(
                model_path=self._yolo_model_path,
                confidence_threshold=self._yolo_conf,
                target_classes=obstacle_classes,
                inference_size=320,  # Smaller for speed on obstacle cams
                device=self._yolo_device,
            )
            self._yolo.load_model()
            self.get_logger().info('YOLO obstacle classifier loaded')
        except Exception as e:
            self.get_logger().warn(f'YOLO classification disabled: {e}')
            self._yolo = None
            self._enable_yolo = False

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_telemetry(self, msg: Telemetry):
        self._is_armed = msg.is_armed
        self._is_in_air = msg.is_in_air

    def _on_sector_image(self, msg: Image, sector: str):
        """Store latest frame for the given sector."""
        if self._bridge is None or cv2 is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'[{sector}] Image conversion failed: {e}')
            return

        state = self._sector_states[sector]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        now = time.time()
        state.last_frame_time = now
        state.active = True

        # Run background subtraction
        fg_mask = state.bg_subtractor.apply(gray)

        # Threshold and morphological cleanup
        _, thresh = cv2.threshold(fg_mask, self._motion_threshold, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter and sort by area
        valid_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area >= self._min_contour_area:
                x, y, w, h = cv2.boundingRect(cnt)
                valid_contours.append((area, x, y, x + w, y + h))

        valid_contours.sort(key=lambda c: c[0], reverse=True)
        valid_contours = valid_contours[:self._max_contours]

        # --- Kalman-based matching ---
        # 1. Predict all existing trackers
        existing_ids = list(state.tracked_contours.keys())
        predicted_bboxes = []
        for cid in existing_ids:
            tc = state.tracked_contours[cid]
            if tc.kalman is not None:
                pred = tc.kalman.predict()
                predicted_bboxes.append(pred)
            else:
                predicted_bboxes.append(tc.bbox)

        # 2. Build detection list
        det_bboxes = [(x1, y1, x2, y2)
                      for (area, x1, y1, x2, y2) in valid_contours]

        # 3. Associate detections with predicted tracker positions
        matches, unmatched_dets, unmatched_trks = associate_detections(
            det_bboxes, predicted_bboxes, iou_threshold=0.3)

        new_tracked = {}

        # 4. Update matched tracks
        for d_idx, t_idx in matches:
            cid = existing_ids[t_idx]
            tc = state.tracked_contours[cid]
            area, x1, y1, x2, y2 = valid_contours[d_idx]
            if tc.kalman is not None:
                tc.kalman.update((x1, y1, x2, y2))
            tc.bbox = (x1, y1, x2, y2)
            tc.area = area
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            tc.center = (cx, cy)
            tc.history.append((now, area, (cx, cy)))
            tc.last_seen = now
            tc.frames_tracked += 1
            new_tracked[cid] = tc

        # 5. Create new tracks for unmatched detections
        for d_idx in unmatched_dets:
            area, x1, y1, x2, y2 = valid_contours[d_idx]
            cid = state.next_contour_id
            state.next_contour_id += 1
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            tc = TrackedContour(
                contour_id=cid,
                sector=sector,
                bbox=(x1, y1, x2, y2),
                area=area,
                center=(cx, cy),
                last_seen=now,
                frames_tracked=1,
                kalman=KalmanBoxTracker((x1, y1, x2, y2)),
            )
            tc.history.append((now, area, (cx, cy)))
            new_tracked[cid] = tc

        # 6. Keep unmatched trackers that haven't timed out (occlusion handling)
        for t_idx in unmatched_trks:
            cid = existing_ids[t_idx]
            tc = state.tracked_contours[cid]
            if now - tc.last_seen < self._contour_lost_timeout:
                # Update bbox from Kalman prediction
                if tc.kalman is not None:
                    pred_bb = x_to_bbox(tc.kalman.x)
                    tc.bbox = (int(pred_bb[0]), int(pred_bb[1]),
                               int(pred_bb[2]), int(pred_bb[3]))
                new_tracked[cid] = tc

        state.tracked_contours = new_tracked

        # Run YOLO classification on significant contours that need it
        if self._enable_yolo and self._yolo is not None:
            for tc in state.tracked_contours.values():
                if tc.classification == 'unknown' and tc.frames_tracked >= 3:
                    self._classify_contour(frame, tc)

        state.prev_frame_gray = gray

    # ------------------------------------------------------------------
    # Processing
    # ------------------------------------------------------------------

    def _process_tick(self):
        """Periodic aggregation of all sector data into ObstacleArray."""
        if not self._is_in_air:
            return

        now = time.time()
        obstacles_msg = ObstacleArray()
        obstacles_msg.header.stamp = self.get_clock().now().to_msg()
        obstacles_msg.header.frame_id = 'base_link'

        active_sectors = 0
        max_threat = THREAT_NONE
        emergency = False

        for sector, state in self._sector_states.items():
            if not state.active:
                continue
            # Consider sector inactive if no frame received recently
            if now - state.last_frame_time > 2.0:
                state.active = False
                continue
            active_sectors += 1

            for tc in state.tracked_contours.values():
                if tc.frames_tracked < 2:
                    continue  # Need at least 2 frames for velocity

                obstacle = self._contour_to_obstacle(tc, sector)
                if obstacle.threat_level > THREAT_NONE:
                    obstacles_msg.obstacles.append(obstacle)
                    if obstacle.threat_level > max_threat:
                        max_threat = obstacle.threat_level
                    if obstacle.threat_level >= THREAT_EMERGENCY:
                        emergency = True

        obstacles_msg.active_sectors = active_sectors
        obstacles_msg.max_threat_level = max_threat
        obstacles_msg.emergency_detected = emergency

        self._pub_obstacles.publish(obstacles_msg)

        # Publish safety event for high threats
        if max_threat >= THREAT_CRITICAL:
            event = String()
            event.data = f'OBSTACLE_{"EMERGENCY" if emergency else "CRITICAL"}'
            self._pub_event.publish(event)

    def _contour_to_obstacle(self, tc: TrackedContour, sector: str) -> Obstacle:
        """Convert a tracked contour into an Obstacle message."""
        obs = Obstacle()
        obs.header.stamp = self.get_clock().now().to_msg()
        obs.header.frame_id = 'base_link'
        obs.sector = SECTOR_LABELS[sector]

        # Bearing: sector center + offset from image center
        bbox_cx = (tc.bbox[0] + tc.bbox[2]) / 2.0
        px_offset = bbox_cx - (self._cam_w / 2.0)
        angle_offset = math.degrees(math.atan2(px_offset, self._focal_px))
        obs.bearing_deg = SECTOR_BEARINGS[sector] + angle_offset

        # Bounding box
        obs.bbox_x1, obs.bbox_y1 = tc.bbox[0], tc.bbox[1]
        obs.bbox_x2, obs.bbox_y2 = tc.bbox[2], tc.bbox[3]

        # Apparent size ratio
        img_area = self._cam_w * self._cam_h
        obs.apparent_size_ratio = tc.area / img_area if img_area > 0 else 0.0

        # Classification
        obs.classification = tc.classification
        obs.classification_confidence = tc.classification_conf

        # Distance estimation -- prefer Kalman-smoothed area when available
        known_size = self._known_sizes.get(tc.classification, self._known_sizes['unknown'])

        if tc.kalman is not None and tc.kalman.get_area() > 1.0:
            smoothed_area = tc.kalman.get_area()
            smoothed_r = tc.kalman.x[3, 0]  # aspect ratio w/h
            if smoothed_r > 0.01:
                smoothed_h = np.sqrt(smoothed_area / smoothed_r)
            else:
                smoothed_h = np.sqrt(smoothed_area)
            if smoothed_h > 1.0:
                obs.estimated_distance_m = (known_size * self._focal_px) / smoothed_h
            else:
                obs.estimated_distance_m = 100.0
        else:
            # Fallback to raw bbox (first frame, no Kalman yet)
            bbox_h_px = tc.bbox[3] - tc.bbox[1]
            if bbox_h_px > 0:
                obs.estimated_distance_m = (known_size * self._focal_px) / bbox_h_px
            else:
                obs.estimated_distance_m = 100.0

        # Conservative distance for unknown objects: assume small (closer)
        if tc.classification == 'unknown' and tc.classification_conf < 0.1:
            conservative_size = 0.3  # smallest known size (bird)
            if tc.kalman is not None and tc.kalman.get_area() > 1.0:
                smoothed_area = tc.kalman.get_area()
                smoothed_r = tc.kalman.x[3, 0]
                sh = np.sqrt(smoothed_area / max(smoothed_r, 0.01))
                if sh > 1.0:
                    conservative_dist = (conservative_size * self._focal_px) / sh
                    obs.estimated_distance_m = min(obs.estimated_distance_m,
                                                   conservative_dist)
            else:
                bbox_h_px = tc.bbox[3] - tc.bbox[1]
                if bbox_h_px > 0:
                    conservative_dist = (conservative_size * self._focal_px) / bbox_h_px
                    obs.estimated_distance_m = min(obs.estimated_distance_m,
                                                   conservative_dist)

        # Approach velocity estimation
        obs.approach_velocity_m_s, obs.time_to_collision_s = (
            self._estimate_approach_velocity(tc, obs.estimated_distance_m)
        )

        # Distance confidence
        if tc.kalman is not None:
            dist_confidence = compute_distance_confidence(
                tc.frames_tracked,
                tc.classification_conf,
                tc.kalman.get_innovation_area(),
                tc.kalman.get_predicted_area(),
            )
        else:
            dist_confidence = 0.0

        # Threat level (with confidence-based dampening)
        obs.threat_level = self._compute_threat_level(
            obs.estimated_distance_m,
            obs.time_to_collision_s,
            obs.approach_velocity_m_s,
            dist_confidence,
        )

        return obs

    def _estimate_approach_velocity(
        self, tc: TrackedContour, current_distance: float
    ) -> Tuple[float, float]:
        """Estimate approach velocity and TTC.

        Primary source: Kalman filter's area velocity state (ds).
        Fallback: linear regression on sqrt(area) vs time (first frames).

        Returns:
            (approach_velocity_m_s, time_to_collision_s)
        """
        # Primary: Kalman-filtered area velocity
        if (tc.kalman is not None and tc.frames_tracked >= 3
                and tc.kalman.get_area() > 1.0):
            ds = tc.kalman.get_area_velocity()
            current_area = tc.kalman.get_area()
            current_sqrt_area = np.sqrt(current_area)
            # ds = d(area)/dt;  d(sqrt_area)/dt = ds / (2 * sqrt_area)
            sqrt_area_rate = ds / (2.0 * current_sqrt_area)
            expansion_rate = sqrt_area_rate / current_sqrt_area

            approach_vel = expansion_rate * current_distance
            if approach_vel > 0.3:
                ttc = current_distance / approach_vel
            else:
                approach_vel = max(approach_vel, 0.0)
                ttc = -1.0
            return approach_vel, ttc

        # Fallback: linear regression on sqrt(area) history
        if len(tc.history) < 3:
            return 0.0, -1.0

        now = time.time()
        recent = [(t, a, c) for (t, a, c) in tc.history
                  if now - t <= self._velocity_window]

        if len(recent) < 2:
            return 0.0, -1.0

        t0 = recent[0][0]
        times = np.array([r[0] - t0 for r in recent])
        sqrt_areas = np.array([math.sqrt(r[1]) for r in recent])

        if times[-1] - times[0] < 0.05:
            return 0.0, -1.0

        n = len(times)
        sum_t = np.sum(times)
        sum_a = np.sum(sqrt_areas)
        sum_ta = np.sum(times * sqrt_areas)
        sum_t2 = np.sum(times * times)

        denom = n * sum_t2 - sum_t * sum_t
        if abs(denom) < 1e-9:
            return 0.0, -1.0

        slope = (n * sum_ta - sum_t * sum_a) / denom

        current_sqrt_area = sqrt_areas[-1]
        if current_sqrt_area < 1.0:
            return 0.0, -1.0

        expansion_rate = slope / current_sqrt_area
        approach_vel = expansion_rate * current_distance

        if approach_vel > 0.3:
            ttc = current_distance / approach_vel
        else:
            approach_vel = max(approach_vel, 0.0)
            ttc = -1.0

        return approach_vel, ttc

    def _compute_threat_level(
        self, distance: float, ttc: float, approach_vel: float,
        confidence: float = 1.0,
    ) -> int:
        """Compute threat level based on distance, TTC, and detection confidence.

        When confidence is low (< 0.3), CRITICAL is dampened to WARNING to
        avoid false-positive emergency manoeuvres.  EMERGENCY is never
        dampened because safety takes priority.
        """
        threat = THREAT_NONE

        # Distance-based threat
        if distance <= self._dist_emergency:
            threat = max(threat, THREAT_EMERGENCY)
        elif distance <= self._dist_critical:
            threat = max(threat, THREAT_CRITICAL)
        elif distance <= self._dist_warning:
            threat = max(threat, THREAT_WARNING)
        elif distance <= self._dist_caution:
            threat = max(threat, THREAT_CAUTION)
        else:
            threat = max(threat, THREAT_MONITOR)

        # TTC-based threat escalation
        if ttc > 0:
            if ttc <= self._ttc_emergency:
                threat = max(threat, THREAT_EMERGENCY)
            elif ttc <= self._ttc_critical:
                threat = max(threat, THREAT_CRITICAL)
            elif ttc <= self._ttc_warning:
                threat = max(threat, THREAT_WARNING)
            elif ttc <= self._ttc_caution:
                threat = max(threat, THREAT_CAUTION)

        # Fast approach velocity is inherently dangerous even at distance
        if approach_vel > 25.0:  # >90 km/h
            threat = max(threat, THREAT_CRITICAL)
        elif approach_vel > 15.0:  # >54 km/h
            threat = max(threat, THREAT_WARNING)

        # Confidence-based dampening: low-confidence CRITICAL → WARNING
        # Never dampen EMERGENCY (safety critical)
        if confidence < 0.3 and threat == THREAT_CRITICAL:
            threat = THREAT_WARNING

        return threat

    # ------------------------------------------------------------------
    # YOLO classification
    # ------------------------------------------------------------------

    # Map COCO class_id -> our obstacle classification labels
    _COCO_TO_OBSTACLE = {
        0: 'person', 1: 'vehicle', 2: 'vehicle', 3: 'vehicle',
        5: 'vehicle', 7: 'vehicle', 8: 'vehicle',
        14: 'bird', 15: 'animal', 16: 'animal',
    }

    def _classify_contour(self, frame: np.ndarray, tc: TrackedContour):
        """Run YOLO on the bounding box region to classify the obstacle."""
        if self._yolo is None:
            return

        x1, y1, x2, y2 = tc.bbox
        h, w = frame.shape[:2]

        # Add padding around the bbox for better classification
        pad = 20
        x1p = max(0, x1 - pad)
        y1p = max(0, y1 - pad)
        x2p = min(w, x2 + pad)
        y2p = min(h, y2 + pad)

        crop = frame[y1p:y2p, x1p:x2p]
        if crop.size == 0:
            return

        try:
            results = self._yolo.detect(crop)
        except RuntimeError:
            return

        if results:
            best = max(results, key=lambda r: r.confidence)
            tc.classification = self._COCO_TO_OBSTACLE.get(
                best.class_id, 'unknown'
            )
            tc.classification_conf = best.confidence
        else:
            # No YOLO match -- could be a static obstacle (tree, building, wire)
            tc.classification = 'unknown'
            tc.classification_conf = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
