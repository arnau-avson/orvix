"""Export helpers for inspecting routes outside the program."""
import json
from pathlib import Path
from typing import Iterable, Optional, Union

from .models import Route
from .traffic_lights import TrafficLight


def route_to_geojson(
    route: Route,
    lights: Optional[Iterable[TrafficLight]] = None,
) -> dict:
    """Build a GeoJSON FeatureCollection: the route as a LineString plus one
    Point feature per traffic light. Drop the result on https://geojson.io to
    visualize it.
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
            },
            "geometry": {"type": "Point", "coordinates": [light.point.lon, light.point.lat]},
        })

    return {"type": "FeatureCollection", "features": features}


def save_geojson(
    path: Union[str, Path],
    route: Route,
    lights: Optional[Iterable[TrafficLight]] = None,
) -> Path:
    out = Path(path)
    out.write_text(json.dumps(route_to_geojson(route, lights), indent=2))
    return out


def route_to_csv(route: Route) -> str:
    """One row per polyline point: index,lat,lon."""
    rows = ["index,lat,lon"]
    for i, p in enumerate(route.full_polyline):
        rows.append(f"{i},{p.lat:.7f},{p.lon:.7f}")
    return "\n".join(rows)
