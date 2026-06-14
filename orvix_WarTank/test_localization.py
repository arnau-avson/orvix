"""Simulate the robot walking the route and watch the tracker emit progress
+ approach events for traffic lights along the way.
"""
from delivery_robot import (
    compute_route,
    geocode,
    load_walk_graph_for_trip,
    plan_with_signals,
)
from delivery_robot.localization import RouteSimulator, RouteTracker


def run(origin_address: str, destination_address: str) -> None:
    origin = geocode(origin_address)
    destination = geocode(destination_address)

    graph = load_walk_graph_for_trip(origin, destination, margin_m=600)
    route = compute_route(graph, origin, destination)
    annotated = plan_with_signals(route, graph)

    print(f"Route: {route.total_distance_m:.0f} m  ({len(annotated.lights)} lights)")

    sim = RouteSimulator(route, speed_mps=1.4, timestep_s=10.0)
    tracker = RouteTracker(route, approach_radius_m=15.0)
    tracker.attach_lights(annotated.lights)

    seen_lights: set = set()
    while (pose := sim.get_pose()) is not None:
        state = tracker.update(pose)
        marker = " OFF-ROUTE" if state.is_off_route else ""
        heading = f"{pose.heading_deg:5.1f}°" if pose.heading_deg is not None else "  ?  "
        print(
            f"t={pose.timestamp_s:5.0f}s  "
            f"progress={state.progress_m:6.0f}m  "
            f"remaining={state.remaining_m:6.0f}m  "
            f"hdg={heading}  "
            f"off_route_d={state.off_route_distance_m:5.2f}m{marker}"
        )
        for light in state.approaching_lights:
            key = (round(light.point.lat, 6), round(light.point.lon, 6))
            if key in seen_lights:
                continue
            seen_lights.add(key)
            print(
                f"   -> APPROACHING {light.kind} signal "
                f"(crossing bearing {light.crossing_bearing:.0f}°) — "
                f"sensor.is_green() should be queried now"
            )

    print(f"\nArrived. Total distance simulated: {tracker.total_m:.0f} m")
    print(f"Lights encountered: {len(seen_lights)} / {len(annotated.lights)}")


if __name__ == "__main__":
    run(
        "Plaça de Francesc Macià, Barcelona",
        "L'illa Diagonal, Barcelona",
    )
