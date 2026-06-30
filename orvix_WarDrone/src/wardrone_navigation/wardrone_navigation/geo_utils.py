"""Geographic utility functions.

Shared by waypoint_navigator, mission_controller, mission_loader,
and other nodes that need GPS distance/bearing calculations.
"""

import math


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate the great-circle distance between two points in meters."""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)

    a = (math.sin(dphi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c


def compute_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Bearing from point 1 to point 2 in degrees (0=North, CW)."""
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    return math.degrees(math.atan2(dlon * math.cos(math.radians(lat1)), dlat))


def normalize_angle(deg: float) -> float:
    """Normalize angle to [-180, 180]."""
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


def offset_position(lat: float, lon: float, bearing_deg: float, distance_m: float):
    """Offset a GPS position by distance in a given bearing. Returns (lat, lon)."""
    R = 6371000.0
    bearing_rad = math.radians(bearing_deg)
    dlat = (distance_m * math.cos(bearing_rad)) / R
    dlon = (distance_m * math.sin(bearing_rad)) / (R * math.cos(math.radians(lat)))
    return lat + math.degrees(dlat), lon + math.degrees(dlon)


def compute_reroute_waypoint(
    current_lat: float, current_lon: float,
    target_lat: float, target_lon: float,
    obstacle_bearing_deg: float,
    offset_distance_m: float = 20.0,
):
    """Compute a temporary waypoint offset perpendicular to the direct path to target.

    Returns (lat, lon) of the offset waypoint, or None if rerouting not needed
    (obstacle is not blocking the path to target).
    """
    path_bearing = compute_bearing(current_lat, current_lon, target_lat, target_lon)

    # Check if obstacle is roughly between us and target (within +/- 60 degrees)
    angle_diff = normalize_angle(obstacle_bearing_deg - path_bearing)
    if abs(angle_diff) > 60.0:
        return None

    # Choose offset direction: perpendicular to path, away from obstacle
    if angle_diff >= 0:
        offset_bearing = path_bearing - 90.0  # Go left of path
    else:
        offset_bearing = path_bearing + 90.0  # Go right of path

    return offset_position(current_lat, current_lon, offset_bearing, offset_distance_m)


def is_point_near_path(
    path_start_lat: float, path_start_lon: float,
    path_end_lat: float, path_end_lon: float,
    point_lat: float, point_lon: float,
    threshold_m: float,
) -> bool:
    """Check if a point is within threshold_m of the line segment from start to end.

    Uses cross-track distance approximation for short distances.
    """
    d_start_end = haversine_distance(path_start_lat, path_start_lon,
                                     path_end_lat, path_end_lon)
    if d_start_end < 1.0:
        return haversine_distance(path_start_lat, path_start_lon,
                                  point_lat, point_lon) < threshold_m

    d_start_point = haversine_distance(path_start_lat, path_start_lon,
                                       point_lat, point_lon)
    d_end_point = haversine_distance(path_end_lat, path_end_lon,
                                     point_lat, point_lon)

    # If point is beyond either end, check distance to nearest endpoint
    if d_start_point > d_start_end + threshold_m:
        return False
    if d_end_point > d_start_end + threshold_m:
        return False

    # Cross-track distance approximation using triangle area
    # Area of triangle = 0.5 * base * height
    # s = semi-perimeter for Heron's formula
    s = (d_start_end + d_start_point + d_end_point) / 2.0
    area_sq = s * (s - d_start_end) * (s - d_start_point) * (s - d_end_point)

    if area_sq <= 0:
        # Zero area means point is collinear with the path (cross-track ≈ 0).
        # Prior checks already excluded points beyond either endpoint.
        return True

    area = math.sqrt(area_sq)
    cross_track = 2.0 * area / d_start_end

    return cross_track < threshold_m


def estimate_obstacle_position(
    drone_lat: float, drone_lon: float, drone_yaw_deg: float,
    obstacle_bearing_deg: float, obstacle_distance_m: float,
) -> tuple:
    """Estimate the GPS position of an obstacle given drone position and bearing.

    obstacle_bearing_deg is relative to the drone's sector (absolute bearing).
    Returns (lat, lon).
    """
    absolute_bearing = obstacle_bearing_deg + drone_yaw_deg
    return offset_position(drone_lat, drone_lon, absolute_bearing, obstacle_distance_m)
