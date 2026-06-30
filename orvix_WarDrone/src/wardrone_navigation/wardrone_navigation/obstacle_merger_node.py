"""Obstacle merger node -- fuses vision and range sensor obstacle sources.

Subscribes to:
    /wardrone/obstacles/vision  (from obstacle_detector_node, camera-based)
    /wardrone/obstacles/range   (from range_sensor_node, laser rangefinder)

Publishes:
    /wardrone/obstacles         (unified ObstacleArray for avoidance node)

Fusion strategy:
    Collects the latest message from each source and merges them on a timer.
    Obstacles from the same sector are correlated: if both sources report
    an obstacle in the same sector, the range sensor's distance replaces
    the vision estimate (more accurate), while the vision's classification
    is preserved (range sensor cannot classify).
    Obstacles in non-overlapping sectors pass through unchanged.
"""

import rclpy
from rclpy.node import Node

from wardrone_interfaces.msg import Obstacle, ObstacleArray


# ---------------------------------------------------------------------------
# Pure functions (testable without rclpy)
# ---------------------------------------------------------------------------

def merge_obstacle_arrays(
    vision_obstacles: list,
    range_obstacles: list,
    vision_age_s: float,
    range_age_s: float,
    max_age_s: float = 1.0,
) -> list:
    """Merge vision and range obstacle lists into a single list.

    If both sources report an obstacle in the same sector, the range
    sensor's distance replaces the vision estimate, but the vision's
    classification is kept.

    Sources older than max_age_s are discarded entirely.

    Returns a list of merged Obstacle-like dicts with keys:
        sector, bearing_deg, estimated_distance_m, approach_velocity_m_s,
        time_to_collision_s, classification, classification_confidence,
        threat_level, source
    """
    result = []

    # Filter stale sources
    v_obs = vision_obstacles if vision_age_s <= max_age_s else []
    r_obs = range_obstacles if range_age_s <= max_age_s else []

    # Index range obstacles by sector for quick lookup
    range_by_sector = {}
    for obs in r_obs:
        sector = obs.get('sector', '') if isinstance(obs, dict) else obs.sector
        range_by_sector[sector] = obs

    # Process vision obstacles, fusing with range where available
    fused_sectors = set()
    for v in v_obs:
        sector = v.get('sector', '') if isinstance(v, dict) else v.sector
        r = range_by_sector.get(sector)
        if r is not None:
            # Fuse: range distance + vision classification
            fused = _to_dict(v)
            r_dict = _to_dict(r)
            fused['estimated_distance_m'] = r_dict['estimated_distance_m']
            fused['approach_velocity_m_s'] = r_dict['approach_velocity_m_s']
            fused['time_to_collision_s'] = r_dict['time_to_collision_s']
            # Recalculate threat from the more accurate range distance
            fused['threat_level'] = max(
                r_dict['threat_level'], fused['threat_level'])
            fused['source'] = 'fused'
            result.append(fused)
            fused_sectors.add(sector)
        else:
            d = _to_dict(v)
            d['source'] = 'vision'
            result.append(d)

    # Add range-only obstacles (sectors not covered by vision)
    for sector, r in range_by_sector.items():
        if sector not in fused_sectors:
            d = _to_dict(r)
            d['source'] = 'range'
            result.append(d)

    return result


def _to_dict(obs) -> dict:
    """Convert an Obstacle msg or dict to a plain dict."""
    if isinstance(obs, dict):
        return dict(obs)
    return {
        'sector': obs.sector,
        'bearing_deg': obs.bearing_deg,
        'estimated_distance_m': obs.estimated_distance_m,
        'approach_velocity_m_s': obs.approach_velocity_m_s,
        'time_to_collision_s': obs.time_to_collision_s,
        'classification': obs.classification,
        'classification_confidence': obs.classification_confidence,
        'threat_level': obs.threat_level,
        'bbox_x1': obs.bbox_x1,
        'bbox_y1': obs.bbox_y1,
        'bbox_x2': obs.bbox_x2,
        'bbox_y2': obs.bbox_y2,
        'apparent_size_ratio': obs.apparent_size_ratio,
    }


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class ObstacleMergerNode(Node):

    def __init__(self):
        super().__init__('obstacle_merger')

        self.declare_parameter('merge_rate_hz', 10.0)
        self.declare_parameter('source_max_age_s', 1.0)

        rate = self.get_parameter('merge_rate_hz').value
        self._max_age = self.get_parameter('source_max_age_s').value

        # Latest vision message
        self._vision_msg = None
        self._vision_time_ns = 0

        # Per-sector range buffer (supports multiple range sensors)
        self._range_by_sector = {}        # sector_str -> Obstacle msg
        self._range_time_by_sector = {}   # sector_str -> nanoseconds

        # Publisher -- the unified topic consumed by avoidance
        self._pub = self.create_publisher(
            ObstacleArray, '/wardrone/obstacles', 10)

        # Subscribers
        self.create_subscription(
            ObstacleArray, '/wardrone/obstacles/vision',
            self._on_vision, 10)
        self.create_subscription(
            ObstacleArray, '/wardrone/obstacles/range',
            self._on_range, 10)

        # Merge timer
        self.create_timer(1.0 / rate, self._merge_tick)

        self.get_logger().info(
            f'Obstacle Merger ready: rate={rate}Hz, max_age={self._max_age}s')

    def _on_vision(self, msg: ObstacleArray):
        self._vision_msg = msg
        self._vision_time_ns = self.get_clock().now().nanoseconds

    def _on_range(self, msg: ObstacleArray):
        """Buffer each range obstacle by sector (supports multiple sensors)."""
        now_ns = self.get_clock().now().nanoseconds
        for obs in msg.obstacles:
            self._range_by_sector[obs.sector] = obs
            self._range_time_by_sector[obs.sector] = now_ns

    def _merge_tick(self):
        now_ns = self.get_clock().now().nanoseconds

        # Vision: single source
        vision_age = (now_ns - self._vision_time_ns) / 1e9 if self._vision_time_ns > 0 else 999.0
        v_list = list(self._vision_msg.obstacles) if self._vision_msg else []

        # Range: collect all non-stale per-sector readings
        r_list = []
        stale_sectors = []
        for sector, obs in self._range_by_sector.items():
            t_ns = self._range_time_by_sector.get(sector, 0)
            age = (now_ns - t_ns) / 1e9 if t_ns > 0 else 999.0
            if age <= self._max_age:
                r_list.append(obs)
            else:
                stale_sectors.append(sector)

        # Clean up stale sectors
        for s in stale_sectors:
            del self._range_by_sector[s]
            del self._range_time_by_sector[s]

        # range_age=0.0 since we already filtered stale readings above
        range_age = 0.0 if r_list else 999.0

        merged = merge_obstacle_arrays(
            v_list, r_list, vision_age, range_age, self._max_age)

        if not merged and not v_list and not r_list:
            return  # Nothing from either source

        # Build unified ObstacleArray
        out = ObstacleArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'

        max_threat = 0
        emergency = False
        sectors = set()

        for m in merged:
            obs = Obstacle()
            obs.header.stamp = out.header.stamp
            obs.header.frame_id = 'base_link'
            obs.sector = m['sector']
            obs.bearing_deg = m['bearing_deg']
            obs.estimated_distance_m = m['estimated_distance_m']
            obs.approach_velocity_m_s = m['approach_velocity_m_s']
            obs.time_to_collision_s = m['time_to_collision_s']
            obs.classification = m.get('classification', 'unknown')
            obs.classification_confidence = m.get('classification_confidence', 0.0)
            obs.threat_level = m['threat_level']
            obs.bbox_x1 = m.get('bbox_x1', 0)
            obs.bbox_y1 = m.get('bbox_y1', 0)
            obs.bbox_x2 = m.get('bbox_x2', 0)
            obs.bbox_y2 = m.get('bbox_y2', 0)
            obs.apparent_size_ratio = m.get('apparent_size_ratio', 0.0)

            out.obstacles.append(obs)
            sectors.add(obs.sector)
            if obs.threat_level > max_threat:
                max_threat = obs.threat_level
            if obs.threat_level >= 5:
                emergency = True

        out.active_sectors = len(sectors)
        out.max_threat_level = max_threat
        out.emergency_detected = emergency
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMergerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
