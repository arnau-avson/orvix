"""Plan a sidewalk-only delivery route A -> B, detect every road crossing
along the way (geometric: where the pedestrian polyline intersects a vehicle
road), and dump everything to GeoJSON for visualization on geojson.io.
"""
from delivery_robot import (
    AlwaysGoSensor,
    compute_route,
    find_road_crossings,
    find_route_inside_buildings,
    geocode,
    load_buildings,
    load_road_graph_for_trip,
    load_walk_graph_for_trip,
    plan_with_signals,
    save_geojson,
    should_proceed,
    snap_outside_buildings,
)
from delivery_robot.models import Point


def run(origin_address: str, destination_address: str) -> None:
    origin = geocode(origin_address)
    destination = geocode(destination_address)
    print(f"Origin     : {origin_address}  -> {origin}")
    print(f"Destination: {destination_address}  -> {destination}")

    # If origin/destination falls inside a building (e.g. a mall, geocoded
    # to its centroid), snap it to the nearest sidewalk-side wall point.
    # The robot delivers at the door, not inside.
    midpoint = Point(
        lat=(origin.lat + destination.lat) / 2,
        lon=(origin.lon + destination.lon) / 2,
    )
    buildings = load_buildings(midpoint, radius_m=1500)
    snapped_origin = snap_outside_buildings(origin, buildings)
    snapped_dest = snap_outside_buildings(destination, buildings)
    if snapped_origin != origin:
        print(f"  origin snapped out of building -> {snapped_origin}")
        origin = snapped_origin
    if snapped_dest != destination:
        print(f"  destination snapped out of building -> {snapped_dest}")
        destination = snapped_dest

    walk_graph = load_walk_graph_for_trip(origin, destination, margin_m=600,
                                          strict_pedestrian=True)
    route = compute_route(walk_graph, origin, destination)
    annotated = plan_with_signals(route, walk_graph)

    # Vehicle network for geometric crossing detection. Loaded separately so
    # the route stays on pure pedestrian ways while we still know where it
    # crosses the road system.
    road_graph = load_road_graph_for_trip(origin, destination, margin_m=600)
    crossings = find_road_crossings(route, road_graph, signal_nodes=annotated.lights)

    # Sanity check: ensure no segment of the route physically passes through
    # a building. Even with the strict pedestrian filter + endpoint snapping
    # this can still happen due to OSM tagging gaps mid-route; flagged
    # segments need manual inspection.
    building_issues = find_route_inside_buildings(route, buildings)

    print(f"\nRoute distance : {route.total_distance_m:.0f} m")
    print(f"Estimated time : {route.estimated_time_s() / 60:.1f} min @ 5 km/h")
    print(f"Decision points: {len(route.waypoints)}")
    print(f"Polyline points: {len(route.full_polyline)}")
    print(f"Traffic lights : {len(annotated.lights)}")
    print(f"Road crossings : {len(crossings)}")
    print(f"Building hits  : {len(building_issues)}")

    if building_issues:
        print("\n!! WARNING — route segments inside buildings (need filter review):")
        for b in building_issues:
            print(f"  segment {b.segment_index}  overlap={b.overlap_length_m:.1f}m  "
                  f"({b.entry_point.lat:.6f}, {b.entry_point.lon:.6f}) -> "
                  f"({b.exit_point.lat:.6f}, {b.exit_point.lon:.6f})")

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

    out = save_geojson("route.geojson", route, annotated.lights,
                       crossings, building_issues)
    print(f"\nGeoJSON written to {out.resolve()}")
    print("Open it in https://geojson.io — green = signaled crossings,")
    print("red = unsignaled crossings, magenta = route inside building (BAD).")


if __name__ == "__main__":
    run(
        "Plaça de Francesc Macià, Barcelona",
        "L'illa Diagonal, Barcelona",
    )
