"""Mission file loader and validator.

Mission files are YAML with the following structure:

    mission:
      id: "mission_name"
      default_altitude_m: 10.0
      default_speed_m_s: 5.0
      waypoints:
        - latitude_deg: 47.39775
          longitude_deg: 8.54564
          altitude_m: 10.0
        - latitude_deg: 47.39875
          longitude_deg: 8.54664
          speed_m_s: 3.0
          loiter_time_s: 5.0
"""

from dataclasses import dataclass, field
from typing import List, Optional
import yaml


@dataclass
class WaypointData:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float = 10.0
    speed_m_s: float = 0.0
    acceptance_radius_m: float = 0.0
    loiter_time_s: float = 0.0


@dataclass
class MissionData:
    mission_id: str
    waypoints: List[WaypointData]
    default_altitude_m: float = 10.0
    default_speed_m_s: float = 5.0


class MissionLoadError(Exception):
    pass


def load_mission(file_path: str) -> MissionData:
    """Load and validate a mission from a YAML file."""
    try:
        with open(file_path, 'r') as f:
            raw = yaml.safe_load(f)
    except FileNotFoundError:
        raise MissionLoadError(f"Mission file not found: {file_path}")
    except yaml.YAMLError as e:
        raise MissionLoadError(f"Invalid YAML: {e}")

    if not isinstance(raw, dict) or 'mission' not in raw:
        raise MissionLoadError("Missing top-level 'mission' key")

    mission_raw = raw['mission']
    mission_id = mission_raw.get('id', 'unnamed')
    default_alt = float(mission_raw.get('default_altitude_m', 10.0))
    default_speed = float(mission_raw.get('default_speed_m_s', 5.0))

    wp_list = mission_raw.get('waypoints', [])
    if not wp_list:
        raise MissionLoadError("Mission has no waypoints")

    waypoints = []
    for i, wp_raw in enumerate(wp_list):
        if 'latitude_deg' not in wp_raw or 'longitude_deg' not in wp_raw:
            raise MissionLoadError(f"Waypoint {i} missing latitude_deg or longitude_deg")

        lat = float(wp_raw['latitude_deg'])
        lon = float(wp_raw['longitude_deg'])

        if not (-90.0 <= lat <= 90.0):
            raise MissionLoadError(f"Waypoint {i}: latitude {lat} out of range [-90, 90]")
        if not (-180.0 <= lon <= 180.0):
            raise MissionLoadError(f"Waypoint {i}: longitude {lon} out of range [-180, 180]")

        alt = float(wp_raw.get('altitude_m', default_alt))
        speed = float(wp_raw.get('speed_m_s', default_speed))
        radius = float(wp_raw.get('acceptance_radius_m', 0.0))
        loiter = float(wp_raw.get('loiter_time_s', 0.0))

        waypoints.append(WaypointData(
            latitude_deg=lat,
            longitude_deg=lon,
            altitude_m=alt,
            speed_m_s=speed,
            acceptance_radius_m=radius,
            loiter_time_s=loiter,
        ))

    return MissionData(
        mission_id=mission_id,
        waypoints=waypoints,
        default_altitude_m=default_alt,
        default_speed_m_s=default_speed,
    )


def validate_mission(mission: MissionData) -> List[str]:
    """Return a list of warnings (empty = all good)."""
    warnings = []
    if len(mission.waypoints) == 0:
        warnings.append("Mission has no waypoints")
    for i, wp in enumerate(mission.waypoints):
        if wp.altitude_m < 1.0:
            warnings.append(f"Waypoint {i}: altitude {wp.altitude_m}m is very low")
        if wp.altitude_m > 120.0:
            warnings.append(f"Waypoint {i}: altitude {wp.altitude_m}m exceeds 120m (EASA limit)")
    return warnings


def estimate_mission_feasibility(
    mission: MissionData,
    current_lat: float,
    current_lon: float,
    battery_remaining_pct: float,
    battery_reserve_pct: float = 20.0,
    estimated_flight_time_per_pct_s: float = 30.0,
    average_speed_m_s: float = 5.0,
) -> tuple:
    """Estimate whether battery is sufficient for the full mission + RTL.

    Returns:
        (feasible, detail, total_distance_m, estimated_time_s, battery_needed_pct)
    """
    from wardrone_navigation.geo_utils import haversine_distance

    if not mission.waypoints:
        return False, "No waypoints in mission", 0.0, 0.0, 0.0

    # Total mission distance: start -> WP1 -> WP2 -> ... -> WPN
    total_distance = 0.0
    prev_lat, prev_lon = current_lat, current_lon

    for wp in mission.waypoints:
        d = haversine_distance(prev_lat, prev_lon, wp.latitude_deg, wp.longitude_deg)
        total_distance += d
        prev_lat, prev_lon = wp.latitude_deg, wp.longitude_deg

    # RTL distance: last waypoint -> home
    rtl_distance = haversine_distance(prev_lat, prev_lon, current_lat, current_lon)
    total_distance += rtl_distance

    # Loiter time
    loiter_time = sum(wp.loiter_time_s for wp in mission.waypoints)

    # Estimate flight time
    if average_speed_m_s <= 0:
        average_speed_m_s = 5.0
    flight_time_s = (total_distance / average_speed_m_s) + loiter_time

    # Battery needed
    if estimated_flight_time_per_pct_s <= 0:
        estimated_flight_time_per_pct_s = 30.0
    battery_needed_pct = flight_time_s / estimated_flight_time_per_pct_s

    # Available battery after reserve
    available_pct = battery_remaining_pct - battery_reserve_pct

    feasible = battery_needed_pct <= available_pct

    detail = (
        f"Dist: {total_distance:.0f}m "
        f"(mission {total_distance - rtl_distance:.0f}m + RTL {rtl_distance:.0f}m), "
        f"Est. time: {flight_time_s:.0f}s, "
        f"Battery needed: {battery_needed_pct:.1f}%, "
        f"Available: {available_pct:.1f}% "
        f"({battery_remaining_pct:.0f}% - {battery_reserve_pct:.0f}% reserve)"
    )

    return feasible, detail, total_distance, flight_time_s, battery_needed_pct
