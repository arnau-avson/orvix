"""Plan a sidewalk-only delivery route A -> B, detect every road crossing
along the way (geometric: where the pedestrian polyline intersects a vehicle
road), and dump everything to GeoJSON for visualization on geojson.io.
"""
from delivery_robot import (
    AlwaysGoSensor,
    compute_route,
    find_road_crossings,
    geocode,
    load_road_graph_for_trip,
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

    walk_graph = load_walk_graph_for_trip(origin, destination, margin_m=600,
                                          strict_pedestrian=True)
    route = compute_route(walk_graph, origin, destination)
    annotated = plan_with_signals(route, walk_graph)

    # Vehicle network for geometric crossing detection. Loaded separately so
    # the route stays on pure pedestrian ways while we still know where it
    # crosses the road system.
    road_graph = load_road_graph_for_trip(origin, destination, margin_m=600)
    crossings = find_road_crossings(route, road_graph, signal_nodes=annotated.lights)

    print(f"\nRoute distance : {route.total_distance_m:.0f} m")
    print(f"Estimated time : {route.estimated_time_s() / 60:.1f} min @ 5 km/h")
    print(f"Decision points: {len(route.waypoints)}")
    print(f"Polyline points: {len(route.full_polyline)}")
    print(f"Traffic lights : {len(annotated.lights)}")
    print(f"Road crossings : {len(crossings)}")

    if crossings:
        print("\n--- Crossings (entry -> exit, road type, length, signaled) ---")
        for i, c in enumerate(crossings, 1):
            sig = "SIGNALED" if c.is_signaled else "unsignaled"
            print(f"  [{i}] {c.road_type:14}  width={c.road_width_m:5.1f}m  "
                  f"length={c.crossing_length_m:5.1f}m  {sig}")
            print(f"        entry  ({c.entry_point.lat:.6f}, {c.entry_point.lon:.6f})")
            print(f"        midpt  ({c.point.lat:.6f}, {c.point.lon:.6f})")
            print(f"        exit   ({c.exit_point.lat:.6f}, {c.exit_point.lon:.6f})")

    sensor = AlwaysGoSensor()
    if annotated.lights:
        print("\n--- Traffic-light nodes from OSM ---")
        for i, light in enumerate(annotated.lights, 1):
            ok = should_proceed(light, sensor)
            print(f"  [{i}] {light.kind:>10} signal at "
                  f"({light.point.lat:.5f}, {light.point.lon:.5f})  "
                  f"cross bearing {light.crossing_bearing:5.1f}°  "
                  f"{'GO' if ok else 'WAIT'}")

    out = save_geojson("route.geojson", route, annotated.lights, crossings)
    print(f"\nGeoJSON written to {out.resolve()}")
    print("Open it in https://geojson.io — green crosses = signaled crossings,")
    print("red crosses = unsignaled, yellow circles = traffic-light nodes.")


if __name__ == "__main__":
    run(
        "Plaça de Francesc Macià, Barcelona",
        "L'illa Diagonal, Barcelona",
    )
