"""Geometric helpers in lat/lon — shared by localization and routing."""
import math
from typing import Tuple

R_EARTH_M = 6_371_000.0


def bearing_deg(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    """Compass bearing from A to B, in degrees (0=N, 90=E)."""
    lat1 = math.radians(a_lat)
    lat2 = math.radians(b_lat)
    dlon = math.radians(b_lon - a_lon)
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def haversine_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    """Great-circle distance between two lat/lon points in meters."""
    lat1, lat2 = math.radians(a_lat), math.radians(b_lat)
    dlat = math.radians(b_lat - a_lat)
    dlon = math.radians(b_lon - a_lon)
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * R_EARTH_M * math.asin(math.sqrt(h))


def project_onto_segment(
    p_lat: float, p_lon: float,
    a_lat: float, a_lon: float,
    b_lat: float, b_lon: float,
) -> Tuple[float, float, float, float]:
    """Project point P onto segment AB.

    Returns (closest_lat, closest_lon, t, perp_dist_m) where t∈[0,1] is the
    parameter along AB. Uses an equirectangular approximation around A,
    accurate to centimeters for segments under ~1 km — well within sidewalk
    edge lengths.
    """
    deg_to_rad = math.pi / 180.0
    cos_lat = math.cos(math.radians(a_lat))
    bx = (b_lon - a_lon) * cos_lat * deg_to_rad * R_EARTH_M
    by = (b_lat - a_lat) * deg_to_rad * R_EARTH_M
    px = (p_lon - a_lon) * cos_lat * deg_to_rad * R_EARTH_M
    py = (p_lat - a_lat) * deg_to_rad * R_EARTH_M

    seg_len_sq = bx * bx + by * by
    if seg_len_sq < 1e-9:
        return a_lat, a_lon, 0.0, math.hypot(px, py)

    t = (px * bx + py * by) / seg_len_sq
    t = max(0.0, min(1.0, t))

    closest_x = t * bx
    closest_y = t * by
    perp_dist = math.hypot(px - closest_x, py - closest_y)

    closest_lon = a_lon + (closest_x / cos_lat) / (deg_to_rad * R_EARTH_M)
    closest_lat = a_lat + closest_y / (deg_to_rad * R_EARTH_M)
    return closest_lat, closest_lon, t, perp_dist
