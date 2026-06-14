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
