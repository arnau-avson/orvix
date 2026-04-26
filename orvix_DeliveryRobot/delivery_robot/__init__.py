from .models import Point, Step, Route
from .map_loader import (
    load_walk_graph,
    load_walk_graph_from_place,
    load_walk_graph_for_trip,
    load_road_graph,
    load_road_graph_for_trip,
)
from .crossings import Crossing, find_road_crossings
from .geocoder import geocode, reverse_geocode, GeocodingError
from .router import compute_route, RoutingError
from .traffic_lights import (
    TrafficLight,
    TrafficLightSensor,
    AlwaysGoSensor,
    AnnotatedRoute,
    find_traffic_lights,
    plan_with_signals,
    should_proceed,
)
from .export import route_to_geojson, save_geojson, route_to_csv
from .geometry import bearing_deg, haversine_m, project_onto_segment

# Sub-packages — keep heavy deps (torch) lazy by NOT re-exporting perception
# at the top level. Use `from delivery_robot.perception import ...` directly.

__all__ = [
    "Point",
    "Step",
    "Route",
    "load_walk_graph",
    "load_walk_graph_from_place",
    "load_walk_graph_for_trip",
    "load_road_graph",
    "load_road_graph_for_trip",
    "Crossing",
    "find_road_crossings",
    "geocode",
    "reverse_geocode",
    "GeocodingError",
    "compute_route",
    "RoutingError",
    "TrafficLight",
    "TrafficLightSensor",
    "AlwaysGoSensor",
    "AnnotatedRoute",
    "find_traffic_lights",
    "plan_with_signals",
    "should_proceed",
    "route_to_geojson",
    "save_geojson",
    "route_to_csv",
    "bearing_deg",
    "haversine_m",
    "project_onto_segment",
]
