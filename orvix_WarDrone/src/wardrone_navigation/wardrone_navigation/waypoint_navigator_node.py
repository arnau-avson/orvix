"""Waypoint navigator node.

Provides action servers to execute waypoint missions or go to individual waypoints.
Communicates with the MAVSDK bridge via position command topics and telemetry subscriptions.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Bool
from wardrone_interfaces.msg import Telemetry, VehicleState, Waypoint, ObstacleArray
from wardrone_interfaces.srv import LoadMission
from wardrone_interfaces.action import ExecuteMission, GoToWaypoint

from wardrone_navigation.mission_loader import load_mission, MissionLoadError, MissionData
from wardrone_navigation.geo_utils import haversine_distance, compute_reroute_waypoint


class WaypointNavigatorNode(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Parameters
        self.declare_parameter('waypoint_acceptance_radius_m', 2.0)
        self.declare_parameter('default_altitude_m', 10.0)
        self.declare_parameter('default_speed_m_s', 5.0)
        self.declare_parameter('mission_file', '')
        self.declare_parameter('enable_rerouting', False)
        self.declare_parameter('reroute_offset_m', 20.0)

        self._acceptance_radius = self.get_parameter('waypoint_acceptance_radius_m').value
        self._default_alt = self.get_parameter('default_altitude_m').value
        self._default_speed = self.get_parameter('default_speed_m_s').value
        self._enable_rerouting = self.get_parameter('enable_rerouting').value
        self._reroute_offset = self.get_parameter('reroute_offset_m').value

        # State
        self._current_mission: MissionData = None
        self._current_lat = 0.0
        self._current_lon = 0.0
        self._current_alt = 0.0
        self._is_armed = False
        self._mission_active = False

        # Rerouting state (F7)
        self._last_obstacle_bearing = None
        self._avoidance_active = False
        self._avoidance_just_cleared = False

        cb_group = ReentrantCallbackGroup()

        # Publishers
        self._pub_cmd_goto = self.create_publisher(Waypoint, '/wardrone/cmd_goto_global', 10)
        self._pub_current_wp = self.create_publisher(Waypoint, '/wardrone/navigation/current_waypoint', 10)

        # Subscribers
        self.create_subscription(Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)
        self.create_subscription(VehicleState, '/wardrone/state', self._on_state, 10)

        # Rerouting subscribers (F7)
        self.create_subscription(Bool, '/wardrone/obstacle_avoidance/active',
                                 self._on_avoidance_active, 10)
        self.create_subscription(ObstacleArray, '/wardrone/obstacles',
                                 self._on_obstacles_nav, 10)

        # Services
        self.create_service(LoadMission, '/wardrone/load_mission', self._handle_load_mission,
                           callback_group=cb_group)

        # Action servers
        self._execute_mission_action = ActionServer(
            self, ExecuteMission, '/wardrone/execute_mission',
            execute_callback=self._handle_execute_mission,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )
        self._goto_action = ActionServer(
            self, GoToWaypoint, '/wardrone/goto_waypoint',
            execute_callback=self._handle_goto_waypoint,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        # Auto-load mission if parameter set
        mission_file = self.get_parameter('mission_file').value
        if mission_file:
            try:
                self._current_mission = load_mission(mission_file)
                self.get_logger().info(
                    f"Auto-loaded mission '{self._current_mission.mission_id}' "
                    f"with {len(self._current_mission.waypoints)} waypoints"
                )
            except MissionLoadError as e:
                self.get_logger().error(f"Failed to auto-load mission: {e}")

        self.get_logger().info('Waypoint Navigator ready')

    def _on_telemetry(self, msg: Telemetry):
        self._current_lat = msg.latitude_deg
        self._current_lon = msg.longitude_deg
        self._current_alt = msg.relative_altitude_m

    def _on_state(self, msg: VehicleState):
        self._is_armed = msg.is_armed

    def _goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Mission cancel requested')
        return CancelResponse.ACCEPT

    # --- Service: Load Mission ---

    def _handle_load_mission(self, request: LoadMission.Request, response: LoadMission.Response):
        try:
            self._current_mission = load_mission(request.mission_file_path)
            response.success = True
            response.message = f"Loaded mission '{self._current_mission.mission_id}'"
            response.waypoint_count = len(self._current_mission.waypoints)
            self.get_logger().info(response.message)
        except MissionLoadError as e:
            response.success = False
            response.message = str(e)
            response.waypoint_count = 0
            self.get_logger().error(f"Load mission failed: {e}")
        return response

    # --- Action: Execute Mission ---

    async def _handle_execute_mission(self, goal_handle):
        self.get_logger().info('Execute mission started')
        self._mission_active = True
        start_time = time.time()
        total_distance = 0.0

        # Load mission from file if provided, otherwise use current
        mission_file = goal_handle.request.mission_file_path
        if mission_file:
            try:
                self._current_mission = load_mission(mission_file)
            except MissionLoadError as e:
                goal_handle.abort()
                result = ExecuteMission.Result()
                result.success = False
                result.message = str(e)
                self._mission_active = False
                return result

        if self._current_mission is None or len(self._current_mission.waypoints) == 0:
            goal_handle.abort()
            result = ExecuteMission.Result()
            result.success = False
            result.message = 'No mission loaded'
            self._mission_active = False
            return result

        waypoints = self._current_mission.waypoints
        total_wps = len(waypoints)

        for i, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteMission.Result()
                result.success = False
                result.message = 'Mission cancelled'
                result.waypoints_completed = i
                result.waypoints_total = total_wps
                self._mission_active = False
                return result

            # Publish current waypoint
            wp_msg = Waypoint()
            wp_msg.latitude_deg = wp.latitude_deg
            wp_msg.longitude_deg = wp.longitude_deg
            wp_msg.altitude_m = wp.altitude_m
            wp_msg.speed_m_s = wp.speed_m_s
            self._pub_current_wp.publish(wp_msg)

            self.get_logger().info(f'Navigating to waypoint {i+1}/{total_wps}')

            # Navigate to waypoint
            prev_lat, prev_lon = self._current_lat, self._current_lon
            reached = await self._navigate_to_waypoint(wp, goal_handle, i, total_wps)

            if not reached:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result = ExecuteMission.Result()
                    result.success = False
                    result.message = 'Mission cancelled during navigation'
                    result.waypoints_completed = i
                    result.waypoints_total = total_wps
                    self._mission_active = False
                    return result

            total_distance += haversine_distance(
                prev_lat, prev_lon, wp.latitude_deg, wp.longitude_deg
            )

            # Loiter if specified
            if wp.loiter_time_s > 0:
                self.get_logger().info(f'Loitering for {wp.loiter_time_s}s at waypoint {i+1}')
                await self._sleep(wp.loiter_time_s)

        elapsed = time.time() - start_time
        goal_handle.succeed()

        result = ExecuteMission.Result()
        result.success = True
        result.message = 'Mission complete'
        result.waypoints_completed = total_wps
        result.waypoints_total = total_wps
        result.total_distance_m = total_distance
        result.total_time_s = elapsed
        self._mission_active = False

        self.get_logger().info(
            f"Mission complete: {total_wps} waypoints, "
            f"{total_distance:.1f}m, {elapsed:.1f}s"
        )
        return result

    async def _navigate_to_waypoint(self, wp, goal_handle, wp_index: int, total_wps: int) -> bool:
        """Navigate to a single waypoint, publishing feedback. Returns True when reached."""
        acceptance = wp.acceptance_radius_m if wp.acceptance_radius_m > 0 else self._acceptance_radius

        while True:
            if goal_handle.is_cancel_requested:
                return False

            distance = haversine_distance(
                self._current_lat, self._current_lon,
                wp.latitude_deg, wp.longitude_deg
            )

            # Publish feedback
            feedback = ExecuteMission.Feedback()
            feedback.current_waypoint_index = wp_index
            feedback.total_waypoints = total_wps
            feedback.current_latitude_deg = self._current_lat
            feedback.current_longitude_deg = self._current_lon
            feedback.distance_to_next_wp_m = distance
            feedback.mission_progress_pct = (wp_index / total_wps) * 100.0
            goal_handle.publish_feedback(feedback)

            if distance < acceptance:
                self.get_logger().info(f'Waypoint {wp_index+1} reached (dist={distance:.1f}m)')
                return True

            # F7: Check if we need to reroute after obstacle avoidance
            if (self._enable_rerouting and self._avoidance_just_cleared
                    and self._last_obstacle_bearing is not None):
                self._avoidance_just_cleared = False
                reroute = compute_reroute_waypoint(
                    self._current_lat, self._current_lon,
                    wp.latitude_deg, wp.longitude_deg,
                    self._last_obstacle_bearing,
                    self._reroute_offset,
                )
                if reroute:
                    self.get_logger().info(
                        f'Rerouting via ({reroute[0]:.6f}, {reroute[1]:.6f})'
                    )
                    temp_cmd = Waypoint()
                    temp_cmd.latitude_deg = reroute[0]
                    temp_cmd.longitude_deg = reroute[1]
                    temp_cmd.altitude_m = float(wp.altitude_m)
                    temp_cmd.speed_m_s = float(wp.speed_m_s if wp.speed_m_s > 0 else self._default_speed)
                    # Navigate to reroute waypoint first
                    while True:
                        d = haversine_distance(
                            self._current_lat, self._current_lon,
                            reroute[0], reroute[1]
                        )
                        if d < acceptance or goal_handle.is_cancel_requested:
                            break
                        self._pub_cmd_goto.publish(temp_cmd)
                        await self._sleep(0.2)
                self._last_obstacle_bearing = None
            else:
                self._avoidance_just_cleared = False

            # Send GPS goto command to MAVSDK bridge
            cmd = Waypoint()
            cmd.latitude_deg = wp.latitude_deg
            cmd.longitude_deg = wp.longitude_deg
            cmd.altitude_m = float(wp.altitude_m)
            cmd.speed_m_s = float(wp.speed_m_s if wp.speed_m_s > 0 else self._default_speed)
            self._pub_cmd_goto.publish(cmd)

            await self._sleep(0.2)  # 5 Hz update rate

    # --- Action: Go To Waypoint ---

    async def _handle_goto_waypoint(self, goal_handle):
        wp = goal_handle.request.waypoint
        self.get_logger().info(
            f'GoTo waypoint: ({wp.latitude_deg:.6f}, {wp.longitude_deg:.6f}, {wp.altitude_m}m)'
        )

        acceptance = wp.acceptance_radius_m if wp.acceptance_radius_m > 0 else self._acceptance_radius
        start_distance = haversine_distance(
            self._current_lat, self._current_lon,
            wp.latitude_deg, wp.longitude_deg
        )

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = GoToWaypoint.Result()
                result.success = False
                result.message = 'Cancelled'
                result.final_distance_m = haversine_distance(
                    self._current_lat, self._current_lon,
                    wp.latitude_deg, wp.longitude_deg
                )
                return result

            distance = haversine_distance(
                self._current_lat, self._current_lon,
                wp.latitude_deg, wp.longitude_deg
            )

            speed = wp.speed_m_s if wp.speed_m_s > 0 else self._default_speed
            eta = distance / speed if speed > 0 else 0.0

            feedback = GoToWaypoint.Feedback()
            feedback.distance_remaining_m = distance
            feedback.eta_s = eta
            goal_handle.publish_feedback(feedback)

            if distance < acceptance:
                break

            # Send GPS goto command to MAVSDK bridge
            cmd = Waypoint()
            cmd.latitude_deg = wp.latitude_deg
            cmd.longitude_deg = wp.longitude_deg
            cmd.altitude_m = float(wp.altitude_m)
            cmd.speed_m_s = float(wp.speed_m_s if wp.speed_m_s > 0 else self._default_speed)
            self._pub_cmd_goto.publish(cmd)

            await self._sleep(0.2)

        goal_handle.succeed()
        result = GoToWaypoint.Result()
        result.success = True
        result.message = 'Waypoint reached'
        result.final_distance_m = haversine_distance(
            self._current_lat, self._current_lon,
            wp.latitude_deg, wp.longitude_deg
        )
        return result

    # --- F7: Rerouting callbacks ---

    def _on_avoidance_active(self, msg: Bool):
        was_active = self._avoidance_active
        self._avoidance_active = msg.data
        if was_active and not msg.data:
            self._avoidance_just_cleared = True

    def _on_obstacles_nav(self, msg: ObstacleArray):
        if msg.obstacles:
            max_threat = max(msg.obstacles, key=lambda o: o.threat_level)
            if max_threat.threat_level >= 3:
                self._last_obstacle_bearing = max_threat.bearing_deg

    async def _sleep(self, seconds: float):
        """Async sleep using ROS clock."""
        import asyncio
        await asyncio.sleep(seconds)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
