from typing import List, Optional

import networkx as nx
import osmnx as ox

from .models import Point, Step, Route


class RoutingError(RuntimeError):
    pass


def _nearest_node(graph: nx.MultiDiGraph, point: Point) -> int:
    return ox.distance.nearest_nodes(graph, X=point.lon, Y=point.lat)


def _heuristic_factory(graph: nx.MultiDiGraph):
    nodes = graph.nodes

    def h(u: int, v: int) -> float:
        return ox.distance.great_circle(
            nodes[u]["y"], nodes[u]["x"],
            nodes[v]["y"], nodes[v]["x"],
        )

    return h


def _edge_length(data: dict) -> float:
    length = data.get("length")
    if length is None:
        raise RoutingError("Edge missing 'length' attribute")
    return float(length)


def _edge_attr(data: dict, key: str) -> Optional[str]:
    value = data.get(key)
    if isinstance(value, list):
        return value[0] if value else None
    return value


def _edge_geometry(data: dict, start: Point, end: Point) -> List[Point]:
    """Return the edge polyline oriented from `start` to `end`.

    OSMnx stores LineString geometries for simplified edges. The geometry's
    direction is arbitrary (depends on OSM way orientation), so we may need
    to reverse it to match the direction of travel.
    """
    geom = data.get("geometry")
    if geom is None:
        return [start, end]

    coords = [Point(lat=lat, lon=lon) for lon, lat in geom.coords]
    if not coords:
        return [start, end]

    d_first = (coords[0].lat - start.lat) ** 2 + (coords[0].lon - start.lon) ** 2
    d_last = (coords[-1].lat - start.lat) ** 2 + (coords[-1].lon - start.lon) ** 2
    if d_last < d_first:
        coords.reverse()
    return coords


def compute_route(
    graph: nx.MultiDiGraph,
    origin: Point,
    destination: Point,
) -> Route:
    """Compute the shortest sidewalk-only route between two points using A*."""
    src = _nearest_node(graph, origin)
    dst = _nearest_node(graph, destination)

    try:
        node_path = nx.astar_path(
            graph,
            src,
            dst,
            heuristic=_heuristic_factory(graph),
            weight="length",
        )
    except nx.NetworkXNoPath as e:
        raise RoutingError(
            f"No pedestrian path found between {origin} and {destination}"
        ) from e

    steps: list[Step] = []
    for u, v in zip(node_path[:-1], node_path[1:]):
        edge_data = min(
            graph.get_edge_data(u, v).values(),
            key=lambda d: float(d.get("length", float("inf"))),
        )
        start = Point(lat=graph.nodes[u]["y"], lon=graph.nodes[u]["x"])
        end = Point(lat=graph.nodes[v]["y"], lon=graph.nodes[v]["x"])
        steps.append(
            Step(
                start=start,
                end=end,
                length_m=_edge_length(edge_data),
                street_name=_edge_attr(edge_data, "name"),
                highway_type=_edge_attr(edge_data, "highway"),
                geometry=_edge_geometry(edge_data, start, end),
            )
        )

    return Route(origin=origin, destination=destination, steps=steps)
