"""Export helpers for inspecting routes outside the program."""
import json
from pathlib import Path
from typing import Iterable, Optional, Union

from .crossings import Crossing
from .models import Route
from .traffic_lights import TrafficLight


def route_to_geojson(
    route: Route,
    lights: Optional[Iterable[TrafficLight]] = None,
    crossings: Optional[Iterable[Crossing]] = None,
) -> dict:
    """Build a GeoJSON FeatureCollection: route LineString + Point features
    for traffic lights and road crossings. Drop the result on
    https://geojson.io to visualize it.

    Color in geojson.io comes from the `marker-color` property; we set it
    per feature kind so signaled crossings appear green, unsignaled red.
    """
    line_coords = [[p.lon, p.lat] for p in route.full_polyline]
    features = [
        {
            "type": "Feature",
            "properties": {
                "kind": "route",
                "distance_m": round(route.total_distance_m, 1),
                "estimated_time_s": round(route.estimated_time_s(), 1),
            },
            "geometry": {"type": "LineString", "coordinates": line_coords},
        },
        {
            "type": "Feature",
            "properties": {"kind": "origin"},
            "geometry": {"type": "Point", "coordinates": [route.origin.lon, route.origin.lat]},
        },
        {
            "type": "Feature",
            "properties": {"kind": "destination"},
            "geometry": {"type": "Point", "coordinates": [route.destination.lon, route.destination.lat]},
        },
    ]

    for light in lights or ():
        features.append({
            "type": "Feature",
            "properties": {
                "kind": "traffic_light",
                "signal_kind": light.kind,
                "crossing_bearing": round(light.crossing_bearing, 1),
                "must_yield": light.must_yield,
                "marker-color": "#FFD700",
                "marker-symbol": "circle",
            },
            "geometry": {"type": "Point", "coordinates": [light.point.lon, light.point.lat]},
        })

    for crossing in crossings or ():
        color = "#22BB22" if crossing.is_signaled else "#CC0000"
        common_props = {
            "kind": "crossing",
            "road_type": crossing.road_type,
            "is_signaled": crossing.is_signaled,
            "road_width_m": round(crossing.road_width_m, 1),
            "crossing_length_m": round(crossing.crossing_length_m, 1),
            "crossing_bearing": round(crossing.crossing_bearing, 1),
            "step_index": crossing.step_index,
        }
        # The crossing as a LineString: entry curb -> exit curb. Stroke
        # color matches signal status. This is the bit the robot "drives
        # across".
        features.append({
            "type": "Feature",
            "properties": {**common_props, "stroke": color, "stroke-width": 4},
            "geometry": {
                "type": "LineString",
                "coordinates": [
                    [crossing.entry_point.lon, crossing.entry_point.lat],
                    [crossing.exit_point.lon, crossing.exit_point.lat],
                ],
            },
        })
        # Endpoints as separate points so they're individually clickable.
        for label, pt in (("entry", crossing.entry_point),
                         ("exit", crossing.exit_point)):
            features.append({
                "type": "Feature",
                "properties": {
                    **common_props,
                    "endpoint": label,
                    "marker-color": color,
                    "marker-symbol": "square" if label == "entry" else "triangle",
                    "marker-size": "small",
                },
                "geometry": {"type": "Point",
                             "coordinates": [pt.lon, pt.lat]},
            })

    return {"type": "FeatureCollection", "features": features}


def save_geojson(
    path: Union[str, Path],
    route: Route,
    lights: Optional[Iterable[TrafficLight]] = None,
    crossings: Optional[Iterable[Crossing]] = None,
) -> Path:
    out = Path(path)
    out.write_text(json.dumps(route_to_geojson(route, lights, crossings), indent=2))
    return out


def route_to_csv(route: Route) -> str:
    """One row per polyline point: index,lat,lon."""
    rows = ["index,lat,lon"]
    for i, p in enumerate(route.full_polyline):
        rows.append(f"{i},{p.lat:.7f},{p.lon:.7f}")
    return "\n".join(rows)
