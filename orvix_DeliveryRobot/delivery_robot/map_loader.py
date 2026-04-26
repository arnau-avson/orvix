from pathlib import Path
from typing import Optional

import networkx as nx
import osmnx as ox

from .models import Point

_CACHE_DIR = Path(__file__).resolve().parent.parent / ".graph_cache"
_CACHE_DIR.mkdir(exist_ok=True)


def _cache_path(key: str) -> Path:
    safe = key.replace("/", "_").replace(" ", "_")
    return _CACHE_DIR / f"{safe}.graphml"


def load_walk_graph_from_place(place: str, use_cache: bool = True) -> nx.MultiDiGraph:
    """Load the pedestrian-only network for a named place (e.g. 'Barcelona, Spain')."""
    cache = _cache_path(f"place_{place}")
    if use_cache and cache.exists():
        return ox.load_graphml(cache)

    graph = ox.graph_from_place(place, network_type="walk", simplify=True)
    ox.save_graphml(graph, cache)
    return graph


def load_walk_graph(
    center: Point,
    radius_m: float = 2000.0,
    use_cache: bool = True,
) -> nx.MultiDiGraph:
    """Load the pedestrian-only network around a point.

    network_type='walk' restricts edges to those legally walkable: footways,
    sidewalks, pedestrian streets, paths, residential streets — and excludes
    motorways, trunks, and other vehicle-only roads.
    """
    cache = _cache_path(f"point_{center.lat:.5f}_{center.lon:.5f}_r{int(radius_m)}")
    if use_cache and cache.exists():
        return ox.load_graphml(cache)

    graph = ox.graph_from_point(
        center.as_tuple(),
        dist=radius_m,
        network_type="walk",
        simplify=True,
    )
    ox.save_graphml(graph, cache)
    return graph


def load_walk_graph_for_trip(
    origin: Point,
    destination: Point,
    margin_m: float = 500.0,
    use_cache: bool = True,
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
    return load_walk_graph(mid, radius_m=radius, use_cache=use_cache)
