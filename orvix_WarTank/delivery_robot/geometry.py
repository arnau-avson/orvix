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


def destination_point(
    lat: float, lon: float, bearing_deg_value: float, distance_m: float,
) -> tuple[float, float]:
    """Compute the destination (lat, lon) reached from (lat, lon) by moving
    `distance_m` meters along compass bearing `bearing_deg_value` (0=N, 90=E).

    Standard great-circle formula (Vincenty's direct, spherical Earth). Valid
    over distances up to a few hundred km; for sub-km use it's effectively
    exact at sidewalk-robot scale.
    """
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    brng = math.radians(bearing_deg_value)
    d_R = distance_m / R_EARTH_M

    sin_lat1 = math.sin(lat1)
    cos_lat1 = math.cos(lat1)
    sin_d = math.sin(d_R)
    cos_d = math.cos(d_R)

    lat2 = math.asin(sin_lat1 * cos_d + cos_lat1 * sin_d * math.cos(brng))
    lon2 = lon1 + math.atan2(
        math.sin(brng) * sin_d * cos_lat1,
        cos_d - sin_lat1 * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


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
