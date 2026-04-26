"""Example: compute a sidewalk-only delivery route A -> B and dump every
coordinate of the path the robot will follow.
"""
from delivery_robot import (
    AlwaysGoSensor,
    geocode,
    compute_route,
    load_walk_graph_for_trip,
    plan_with_signals,
    save_geojson,
    should_proceed,
)


def run(origin_address: str, destination_address: str) -> None:
    origin = geocode(origin_address)
    destination = geocode(destination_address)
    print(f"Origin     : {origin_address}  -> {origin}")
    print(f"Destination: {destination_address}  -> {destination}")

    graph = load_walk_graph_for_trip(
        origin, destination, margin_m=600, strict_pedestrian=True
    )
    route = compute_route(graph, origin, destination)
    annotated = plan_with_signals(route, graph)

    print(f"\nRoute distance : {route.total_distance_m:.0f} m")
    print(f"Estimated time : {route.estimated_time_s() / 60:.1f} min @ 5 km/h")
    print(f"Decision points: {len(route.waypoints)}")
    print(f"Polyline points: {len(route.full_polyline)}  (full sidewalk curve)")
    print(f"Traffic lights : {len(annotated.lights)}")

    print("\n--- Path coordinates (lat, lon) ---")
    for i, p in enumerate(route.full_polyline):
        print(f"  {i:>4}: {p.lat:.7f}, {p.lon:.7f}")

    sensor = AlwaysGoSensor()
    if annotated.lights:
        print("\n--- Traffic lights along the route ---")
        for i, light in enumerate(annotated.lights, 1):
            ok = should_proceed(light, sensor)
            print(
                f"  [{i}] {light.kind:>10} signal at "
                f"({light.point.lat:.5f}, {light.point.lon:.5f})  "
                f"cross bearing {light.crossing_bearing:5.1f}°  "
                f"{'GO' if ok else 'WAIT'}"
            )

    out = save_geojson("route.geojson", route, annotated.lights)
    print(f"\nGeoJSON written to {out.resolve()}")
    print("Drop it on https://geojson.io to visualize the path.")


if __name__ == "__main__":
    run(
        "Plaça de Francesc Macià, Barcelona",
        "L'illa Diagonal, Barcelona",
    )
