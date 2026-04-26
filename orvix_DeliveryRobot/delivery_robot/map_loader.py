from pathlib import Path
from typing import Optional

import networkx as nx
import osmnx as ox

from .models import Point

_CACHE_DIR = Path(__file__).resolve().parent.parent / ".graph_cache"
_CACHE_DIR.mkdir(exist_ok=True)

# Strict pedestrian filter: only ways genuinely separate from vehicle traffic.
# - footway: sidewalks, footpaths
# - pedestrian: pedestrianized streets (e.g. La Rambla)
# - path: shared paths (parks, etc.)
# - living_street: shared-surface streets where pedestrians have priority
# Excluded:
# - residential/service/tertiary roads (their walkability comes from
#   adjacent sidewalks that OSM rarely models as separate geometry)
# - steps (a wheeled robot cannot climb stairs)
# - indoor=yes/room/corridor/area (inside a building — robot can't enter)
# - tunnel=yes/building_passage (under a building / through a passage that
#   may be gated)
# - access private/no/customers
_STRICT_PEDESTRIAN_FILTER = (
    '["highway"~"footway|pedestrian|path|living_street"]'
    '["foot"!~"no"]'
    '["access"!~"private|no|customers"]'
    '["indoor"!~"yes|room|corridor|area"]'
    '["tunnel"!~"yes|building_passage"]'
)
# Cache version bump — invalidates older cached graphs that used the looser filter.
_STRICT_FILTER_VERSION = "v2"


def _cache_path(key: str) -> Path:
    safe = key.replace("/", "_").replace(" ", "_")
    return _CACHE_DIR / f"{safe}.graphml"


def _build_graph(
    center: Point,
    radius_m: float,
    strict_pedestrian: bool,
) -> nx.MultiDiGraph:
    if strict_pedestrian:
        return ox.graph_from_point(
            center.as_tuple(),
            dist=radius_m,
            custom_filter=_STRICT_PEDESTRIAN_FILTER,
            simplify=True,
            retain_all=False,
        )
    return ox.graph_from_point(
        center.as_tuple(),
        dist=radius_m,
        network_type="walk",
        simplify=True,
    )


def load_walk_graph_from_place(
    place: str,
    use_cache: bool = True,
    strict_pedestrian: bool = False,
) -> nx.MultiDiGraph:
    """Load the pedestrian network for a named place (e.g. 'Barcelona, Spain')."""
    suffix = f"strict_{_STRICT_FILTER_VERSION}" if strict_pedestrian else "walk"
    cache = _cache_path(f"place_{place}_{suffix}")
    if use_cache and cache.exists():
        return ox.load_graphml(cache)

    if strict_pedestrian:
        graph = ox.graph_from_place(
            place, custom_filter=_STRICT_PEDESTRIAN_FILTER, simplify=True
        )
    else:
        graph = ox.graph_from_place(place, network_type="walk", simplify=True)

    ox.save_graphml(graph, cache)
    return graph


def load_walk_graph(
    center: Point,
    radius_m: float = 2000.0,
    use_cache: bool = True,
    strict_pedestrian: bool = False,
) -> nx.MultiDiGraph:
    """Load the pedestrian network around a point.

    Two modes:
    - strict_pedestrian=False (default): network_type='walk' from OSMnx. Includes
      walkable streets whose sidewalks aren't mapped as separate geometry; edge
      lines follow road centerlines.
    - strict_pedestrian=True: only ways tagged as pedestrian-only (footway,
      pedestrian, path, living_street). Geometry matches actual sidewalks/paths
      where OSM has them, but the graph may be disconnected in areas with
      poor sidewalk mapping.
    """
    suffix = f"strict_{_STRICT_FILTER_VERSION}" if strict_pedestrian else "walk"
    cache = _cache_path(
        f"point_{center.lat:.5f}_{center.lon:.5f}_r{int(radius_m)}_{suffix}"
    )
    if use_cache and cache.exists():
        return ox.load_graphml(cache)

    graph = _build_graph(center, radius_m, strict_pedestrian)
    ox.save_graphml(graph, cache)
    return graph


def load_walk_graph_for_trip(
    origin: Point,
    destination: Point,
    margin_m: float = 500.0,
    use_cache: bool = True,
    strict_pedestrian: bool = False,
) -> nx.MultiDiGraph:
    """Load a graph that comfortably covers both endpoints with a safety margin."""
    mid = Point(
        lat=(origin.lat + destination.lat) / 2,
        lon=(origin.lon + destination.lon) / 2,
    )
    haversine_m = ox.distance.great_circle(
        origin.lat, origin.lon, destination.lat, destination.lon
    )
    radius = (haversine_m / 2) + margin_m
    return load_walk_graph(
        mid,
        radius_m=radius,
        use_cache=use_cache,
        strict_pedestrian=strict_pedestrian,
    )


def load_road_graph(
    center: Point,
    radius_m: float = 2000.0,
    use_cache: bool = True,
) -> nx.MultiDiGraph:
    """Load the vehicle road network around a point (no footways/pedestrian).

    Used by `find_road_crossings` to detect where a pedestrian route crosses
    a road. Distinct from the walk graph, which includes footways too.
    """
    cache = _cache_path(
        f"point_{center.lat:.5f}_{center.lon:.5f}_r{int(radius_m)}_drive"
    )
    if use_cache and cache.exists():
        return ox.load_graphml(cache)
    graph = ox.graph_from_point(
        center.as_tuple(),
        dist=radius_m,
        network_type="drive",
        simplify=True,
    )
    ox.save_graphml(graph, cache)
    return graph


def load_road_graph_for_trip(
    origin: Point,
    destination: Point,
    margin_m: float = 500.0,
    use_cache: bool = True,
) -> nx.MultiDiGraph:
    """Vehicle road network covering both trip endpoints + margin."""
    mid = Point(
        lat=(origin.lat + destination.lat) / 2,
        lon=(origin.lon + destination.lon) / 2,
    )
    h = ox.distance.great_circle(
        origin.lat, origin.lon, destination.lat, destination.lon
    )
    radius = (h / 2) + margin_m
    return load_road_graph(mid, radius_m=radius, use_cache=use_cache)
