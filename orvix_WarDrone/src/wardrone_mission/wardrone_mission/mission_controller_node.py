"""Mission Controller Node.

Top-level state machine that orchestrates the drone's mission:
- Manages transitions between navigation, search, tracking, and safety modes
- Subscribes to vehicle state, tracking state, and safety events
- Calls action servers for takeoff, land, and mission execution
- Publishes mission state for monitoring
- Enhanced preflight checklist with individual sensor validation
- Battery-aware mission planning
- Intelligent RTL with obstacle avoidance history
"""

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from wardrone_interfaces.msg import (
    VehicleState, TrackingTarget, Telemetry,
    MissionState as MissionStateMsg, PreflightStatus,
    ObstacleArray, Waypoint,
)
from wardrone_interfaces.srv import Arm, SetFlightMode
from wardrone_interfaces.action import Takeoff, Land, ExecuteMission, GoToWaypoint

from wardrone_mission.states import MissionState, MissionEvent, MissionType
from wardrone_mission.state_machine import StateMachine, TransitionRecord


class MissionControllerNode(Node):

    def __init__(self):
        super().__init__('mission_controller')

        # Parameters
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('mission_file', '')
        self.declare_parameter('mission_type', 'navigate_and_track')
        self.declare_parameter('search_pattern', 'expanding_square')
        self.declare_parameter('search_timeout_s', 60.0)
        self.declare_parameter('search_radius_m', 50.0)
        self.declare_parameter('takeoff_altitude_m', 10.0)
        self.declare_parameter('rtl_altitude_m', 15.0)

        # Preflight parameters (F4)
        self.declare_parameter('require_gps_3d_fix', True)
        self.declare_parameter('min_battery_for_takeoff_pct', 25.0)
        self.declare_parameter('min_gps_satellites', 6)

        # Battery mission planning parameters (F3)
        self.declare_parameter('battery_reserve_pct', 20.0)
        self.declare_parameter('estimated_flight_time_per_pct_s', 30.0)

        # Intelligent RTL parameters (F10)
        self.declare_parameter('enable_intelligent_rtl', True)
        self.declare_parameter('rtl_obstacle_clearance_m', 30.0)

        self._mission_file = self.get_parameter('mission_file').value
        self._search_timeout = self.get_parameter('search_timeout_s').value
        self._search_pattern = self.get_parameter('search_pattern').value
        self._takeoff_alt = self.get_parameter('takeoff_altitude_m').value

        mission_type_str = self.get_parameter('mission_type').value
        try:
            self._mission_type = MissionType(mission_type_str)
        except ValueError:
            self._mission_type = MissionType.NAVIGATE_AND_TRACK

        # State machine
        self._sm = self._build_state_machine()
        self._mission_start_time = 0.0
        self._search_start_time = 0.0
        self._current_wp_index = 0
        self._total_waypoints = 0

        # Vehicle state cache
        self._vehicle_healthy = False
        self._vehicle_armed = False
        self._vehicle_in_air = False
        self._tracking_state = "SEARCHING"

        # Extended telemetry cache (F4)
        self._cached_battery_pct = 0.0
        self._cached_gps_sats = 0
        self._cached_gps_fix = 0
        self._cached_gyro_ok = False
        self._cached_accel_ok = False
        self._cached_mag_ok = False
        self._cached_local_pos_ok = False
        self._cached_global_pos_ok = False
        self._cached_home_pos_ok = False
        self._cached_yaw_deg = 0.0

        # Obstacle history for intelligent RTL (F10)
        self._obstacle_positions = []  # list of (lat, lon)

        # Intelligent RTL state (F10)
        self._rtl_waypoints = []
        self._rtl_wp_index = 0
        self._intelligent_rtl_active = False

        # Preflight cooldown (avoid spamming PREFLIGHT_FAIL with auto_arm)
        self._preflight_fail_time = 0.0

        cb_group = ReentrantCallbackGroup()

        # Publishers
        self._pub_mission_state = self.create_publisher(MissionStateMsg, '/wardrone/mission/state', 10)
        self._pub_cmd_vel = self.create_publisher(Twist, '/wardrone/cmd_velocity', 10)
        self._pub_preflight = self.create_publisher(PreflightStatus, '/wardrone/preflight/status', 10)

        # Subscribers
        self.create_subscription(VehicleState, '/wardrone/state', self._on_vehicle_state, 10)
        self.create_subscription(TrackingTarget, '/wardrone/tracking/target', self._on_tracking_target, 10)
        self.create_subscription(String, '/wardrone/tracking/state', self._on_tracking_state, 10)
        self.create_subscription(String, '/wardrone/safety/event', self._on_safety_event, 10)

        # Obstacle subscription for RTL history (F10)
        self.create_subscription(ObstacleArray, '/wardrone/obstacles', self._on_obstacles, 10)

        # Action clients
        self._takeoff_client = ActionClient(self, Takeoff, '/wardrone/takeoff', callback_group=cb_group)
        self._land_client = ActionClient(self, Land, '/wardrone/land', callback_group=cb_group)
        self._mission_client = ActionClient(self, ExecuteMission, '/wardrone/execute_mission', callback_group=cb_group)
        self._goto_client = ActionClient(self, GoToWaypoint, '/wardrone/goto_waypoint', callback_group=cb_group)

        # Service clients
        self._arm_client = self.create_client(Arm, '/wardrone/arm', callback_group=cb_group)
        self._mode_client = self.create_client(SetFlightMode, '/wardrone/set_flight_mode', callback_group=cb_group)

        # Telemetry for position tracking (RTL home detection)
        self._home_lat = None
        self._home_lon = None
        self._current_lat = 0.0
        self._current_lon = 0.0
        self._current_alt = 0.0
        self.create_subscription(Telemetry, '/wardrone/telemetry', self._on_telemetry, 10)

        # Pending action flags (actions run in background, tick monitors results)
        self._takeoff_sent = False
        self._navigate_sent = False
        self._rtl_sent = False
        self._land_sent = False

        # Main tick timer (5 Hz)
        self.create_timer(0.2, self._tick)

        # State publisher timer (2 Hz)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(f'Mission Controller ready (type={self._mission_type.value})')

    def _build_state_machine(self) -> StateMachine:
        sm = StateMachine(initial_state=MissionState.IDLE)
        sm.set_terminal_states(MissionState.DONE)

        # Normal flow
        sm.add_transition(MissionState.IDLE, MissionEvent.CMD_START, MissionState.PREFLIGHT)
        sm.add_transition(MissionState.PREFLIGHT, MissionEvent.PREFLIGHT_OK, MissionState.TAKEOFF)
        sm.add_transition(MissionState.PREFLIGHT, MissionEvent.PREFLIGHT_FAIL, MissionState.IDLE)
        sm.add_transition(MissionState.TAKEOFF, MissionEvent.TAKEOFF_COMPLETE, MissionState.NAVIGATE)

        # Navigation
        sm.add_transition(MissionState.NAVIGATE, MissionEvent.WAYPOINT_REACHED, MissionState.NAVIGATE)
        sm.add_transition(MissionState.NAVIGATE, MissionEvent.TARGET_DETECTED, MissionState.TRACK)

        # Mission complete → depends on mission type
        if self._mission_type == MissionType.NAVIGATE_ONLY:
            sm.add_transition(MissionState.NAVIGATE, MissionEvent.MISSION_COMPLETE, MissionState.RTL)
        else:
            sm.add_transition(MissionState.NAVIGATE, MissionEvent.MISSION_COMPLETE, MissionState.SEARCH)

        # Search
        sm.add_transition(MissionState.SEARCH, MissionEvent.TARGET_DETECTED, MissionState.TRACK)
        sm.add_transition(MissionState.SEARCH, MissionEvent.SEARCH_TIMEOUT, MissionState.RTL)

        # Track
        sm.add_transition(MissionState.TRACK, MissionEvent.TARGET_LOST, MissionState.SEARCH)
        sm.add_transition(MissionState.TRACK, MissionEvent.CMD_RTL, MissionState.RTL)

        # RTL and Land
        sm.add_transition(MissionState.RTL, MissionEvent.HOME_REACHED, MissionState.LAND)
        sm.add_transition(MissionState.LAND, MissionEvent.LANDED, MissionState.DONE)

        # Emergency
        sm.add_transition(MissionState.EMERGENCY, MissionEvent.LANDED, MissionState.DONE)

        # Global safety transitions
        sm.add_global_transition(
            MissionEvent.SAFETY_CRITICAL, MissionState.EMERGENCY,
            exclude_states={MissionState.DONE, MissionState.EMERGENCY}
        )
        sm.add_global_transition(
            MissionEvent.SAFETY_WARNING, MissionState.RTL,
            exclude_states={MissionState.DONE, MissionState.EMERGENCY, MissionState.RTL, MissionState.LAND}
        )
        sm.add_global_transition(
            MissionEvent.CMD_ABORT, MissionState.LAND,
            exclude_states={MissionState.DONE, MissionState.LAND}
        )

        # Track-only mode: takeoff goes directly to search
        if self._mission_type == MissionType.TRACK_ONLY:
            sm.add_transition(MissionState.TAKEOFF, MissionEvent.TAKEOFF_COMPLETE, MissionState.SEARCH)

        # Logging
        sm.on_any_transition(self._on_state_transition)

        return sm

    def _on_state_transition(self, record: TransitionRecord):
        self.get_logger().info(
            f'State: {record.from_state.value} --{record.event.value}--> {record.to_state.value}'
            + (f' ({record.reason})' if record.reason else '')
        )

    # --- Subscribers ---

    def _on_vehicle_state(self, msg: VehicleState):
        self._vehicle_healthy = msg.is_healthy
        self._vehicle_armed = msg.is_armed
        self._vehicle_in_air = msg.is_in_air

        # Detect landing
        if self._sm.state in (MissionState.LAND, MissionState.EMERGENCY):
            if not msg.is_in_air and self._vehicle_armed:
                self._sm.handle_event(MissionEvent.LANDED, "Vehicle landed")

    def _on_tracking_target(self, msg: TrackingTarget):
        if msg.is_tracking:
            if self._sm.state in (MissionState.SEARCH, MissionState.NAVIGATE):
                self._sm.handle_event(MissionEvent.TARGET_DETECTED, "Target acquired")
        else:
            if self._sm.state == MissionState.TRACK:
                self._sm.handle_event(MissionEvent.TARGET_LOST, "Target lost")

    def _on_tracking_state(self, msg: String):
        self._tracking_state = msg.data

    def _on_safety_event(self, msg: String):
        event_type = msg.data
        if event_type in ('CRITICAL_BATTERY', 'LINK_LOST'):
            self._sm.handle_event(MissionEvent.SAFETY_CRITICAL, event_type)
        elif event_type in ('LOW_BATTERY', 'GPS_DEGRADED', 'WIND_WARNING'):
            self._sm.handle_event(MissionEvent.SAFETY_WARNING, event_type)

    def _on_telemetry(self, msg: Telemetry):
        """Cache telemetry for preflight checks, position tracking, etc."""
        self._current_lat = msg.latitude_deg
        self._current_lon = msg.longitude_deg
        self._current_alt = msg.relative_altitude_m

        # Extended cache (F4)
        self._cached_battery_pct = msg.battery_remaining_pct
        self._cached_gps_sats = msg.gps_num_satellites
        self._cached_gps_fix = msg.gps_fix_type
        self._cached_gyro_ok = msg.is_gyrometer_calibration_ok
        self._cached_accel_ok = msg.is_accelerometer_calibration_ok
        self._cached_mag_ok = msg.is_magnetometer_calibration_ok
        self._cached_local_pos_ok = msg.is_local_position_ok
        self._cached_global_pos_ok = msg.is_global_position_ok
        self._cached_home_pos_ok = msg.is_home_position_ok
        self._cached_yaw_deg = msg.yaw_deg

        # Record home position on first telemetry (before takeoff)
        if self._home_lat is None and msg.is_home_position_ok:
            self._home_lat = msg.latitude_deg
            self._home_lon = msg.longitude_deg
            self.get_logger().info(
                f'Home position set: ({self._home_lat:.6f}, {self._home_lon:.6f})'
            )

    def _on_obstacles(self, msg: ObstacleArray):
        """Record obstacle positions for intelligent RTL (F10)."""
        if self._sm.state != MissionState.NAVIGATE:
            return
        for obs in msg.obstacles:
            if obs.threat_level >= 3 and obs.estimated_distance_m < 50.0:
                from wardrone_navigation.geo_utils import estimate_obstacle_position
                obs_lat, obs_lon = estimate_obstacle_position(
                    self._current_lat, self._current_lon, self._cached_yaw_deg,
                    obs.bearing_deg, obs.estimated_distance_m,
                )
                self._obstacle_positions.append((obs_lat, obs_lon))

    @staticmethod
    def _haversine(lat1, lon1, lat2, lon2):
        R = 6371000.0
        p1, p2 = math.radians(lat1), math.radians(lat2)
        dp = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
        return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

    # --- Main tick ---

    def _tick(self):
        """Main control loop, runs at 5 Hz."""
        state = self._sm.state

        if state == MissionState.IDLE:
            # Auto-start if configured (with cooldown after preflight fail)
            if self.get_parameter('auto_arm').value and self._mission_file:
                if time.time() - self._preflight_fail_time > 5.0:
                    self._sm.handle_event(MissionEvent.CMD_START, "Auto-start")

        elif state == MissionState.PREFLIGHT:
            self._do_preflight()

        elif state == MissionState.TAKEOFF:
            self._do_takeoff()

        elif state == MissionState.NAVIGATE:
            self._do_navigate()

        elif state == MissionState.SEARCH:
            self._do_search()

        elif state == MissionState.RTL:
            self._do_rtl()

        elif state == MissionState.LAND:
            self._do_land()

    # --- F4: Enhanced Preflight ---

    def _do_preflight(self):
        """Enhanced preflight checklist with individual sensor validation."""
        status = PreflightStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        all_ok = True
        failures = []

        # GPS check
        require_3d = self.get_parameter('require_gps_3d_fix').value
        min_sats = self.get_parameter('min_gps_satellites').value
        gps_ok = True
        if require_3d and self._cached_gps_fix < 3:
            gps_ok = False
            failures.append(f"GPS fix {self._cached_gps_fix} < 3")
        if self._cached_gps_sats < min_sats:
            gps_ok = False
            failures.append(f"GPS sats {self._cached_gps_sats} < {min_sats}")
        status.gps_ok = gps_ok
        status.gps_detail = f"Fix={self._cached_gps_fix}, Sats={self._cached_gps_sats}"
        if not gps_ok:
            all_ok = False

        # Battery check
        min_batt = self.get_parameter('min_battery_for_takeoff_pct').value
        batt_ok = self._cached_battery_pct > min_batt
        status.battery_ok = batt_ok
        status.battery_detail = f"{self._cached_battery_pct:.0f}% (min {min_batt:.0f}%)"
        if not batt_ok:
            all_ok = False
            failures.append(f"Battery {self._cached_battery_pct:.0f}% <= {min_batt:.0f}%")

        # Individual health checks
        status.gyro_ok = self._cached_gyro_ok
        status.accel_ok = self._cached_accel_ok
        status.mag_ok = self._cached_mag_ok
        status.local_position_ok = self._cached_local_pos_ok
        status.global_position_ok = self._cached_global_pos_ok
        status.home_position_ok = self._cached_home_pos_ok

        for name, val in [("Gyro", status.gyro_ok), ("Accel", status.accel_ok),
                          ("Mag", status.mag_ok), ("LocalPos", status.local_position_ok),
                          ("GlobalPos", status.global_position_ok),
                          ("HomePos", status.home_position_ok)]:
            if not val:
                all_ok = False
                failures.append(f"{name} NOT OK")

        # Mission loaded check
        has_mission = bool(self._mission_file)
        status.mission_loaded = has_mission
        status.mission_detail = f"File: {self._mission_file}" if has_mission else "No mission file"
        if self._mission_type != MissionType.TRACK_ONLY and not has_mission:
            all_ok = False
            failures.append("No mission file for navigation")

        # F3: Battery-aware mission feasibility check
        if all_ok and has_mission and self._mission_type != MissionType.TRACK_ONLY:
            try:
                from wardrone_navigation.mission_loader import load_mission, estimate_mission_feasibility
                mission_data = load_mission(self._mission_file)
                feasible, detail, _, _, _ = estimate_mission_feasibility(
                    mission_data,
                    self._current_lat, self._current_lon,
                    self._cached_battery_pct,
                    battery_reserve_pct=self.get_parameter('battery_reserve_pct').value,
                    estimated_flight_time_per_pct_s=self.get_parameter('estimated_flight_time_per_pct_s').value,
                )
                status.battery_detail += f" | {detail}"
                if not feasible:
                    all_ok = False
                    failures.append(f"Insufficient battery: {detail}")
            except Exception as e:
                all_ok = False
                failures.append(f"Mission validation error: {e}")

        status.passed = all_ok
        status.failure_reason = "; ".join(failures) if failures else ""
        self._pub_preflight.publish(status)

        if all_ok:
            self._mission_start_time = time.time()
            self._sm.handle_event(MissionEvent.PREFLIGHT_OK, "All preflight checks passed")
        else:
            self._preflight_fail_time = time.time()
            self._sm.handle_event(MissionEvent.PREFLIGHT_FAIL, status.failure_reason)
            self.get_logger().error(f"PREFLIGHT FAILED: {status.failure_reason}")

    def _do_takeoff(self):
        """Send takeoff action (once) and wait for completion."""
        if self._takeoff_sent:
            return

        self._takeoff_sent = True
        self.get_logger().info(f'Sending takeoff to {self._takeoff_alt}m')

        if not self._takeoff_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Takeoff action server not available yet')
            self._takeoff_sent = False
            return

        goal = Takeoff.Goal()
        goal.target_altitude_m = self._takeoff_alt
        future = self._takeoff_client.send_goal_async(goal)
        future.add_done_callback(self._on_takeoff_goal_response)

    def _on_takeoff_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Takeoff goal rejected')
            self._takeoff_sent = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_takeoff_result)

    def _on_takeoff_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f'Takeoff complete at {result.final_altitude_m:.1f}m')
            self._sm.handle_event(MissionEvent.TAKEOFF_COMPLETE, "Takeoff done")
        else:
            self.get_logger().error(f'Takeoff failed: {result.message}')
            self._takeoff_sent = False

    def _do_navigate(self):
        """Send execute_mission action (once) and wait for completion."""
        if self._navigate_sent:
            return

        self._navigate_sent = True
        self.get_logger().info('Sending execute mission')

        if not self._mission_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('ExecuteMission action server not available yet')
            self._navigate_sent = False
            return

        goal = ExecuteMission.Goal()
        goal.mission_file_path = self._mission_file
        future = self._mission_client.send_goal_async(
            goal, feedback_callback=self._on_navigate_feedback
        )
        future.add_done_callback(self._on_navigate_goal_response)

    def _on_navigate_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._current_wp_index = fb.current_waypoint_index
        self._total_waypoints = fb.total_waypoints

    def _on_navigate_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('ExecuteMission goal rejected')
            self._navigate_sent = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_navigate_result)

    def _on_navigate_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(
                f'Mission navigation complete: {result.waypoints_completed} waypoints, '
                f'{result.total_distance_m:.0f}m, {result.total_time_s:.0f}s'
            )
            self._sm.handle_event(MissionEvent.MISSION_COMPLETE, "All waypoints reached")
        else:
            self.get_logger().warn(f'Mission navigation ended: {result.message}')
            self._sm.handle_event(MissionEvent.MISSION_COMPLETE, result.message)

    # --- F10: Intelligent RTL ---

    def _do_rtl(self):
        """RTL with obstacle avoidance or PX4 native RTL as fallback."""
        if not self._rtl_sent:
            self._rtl_sent = True

            enable_intelligent = self.get_parameter('enable_intelligent_rtl').value
            if (enable_intelligent and self._obstacle_positions
                    and self._home_lat is not None):
                self._do_intelligent_rtl()
            else:
                self._do_px4_rtl()
            return

        # Monitor for intelligent RTL waypoint progress
        if self._intelligent_rtl_active:
            return  # GoToWaypoint callbacks handle progress

        # Monitor distance to home (for PX4 native RTL)
        if self._home_lat is not None:
            dist = self._haversine(
                self._current_lat, self._current_lon,
                self._home_lat, self._home_lon
            )
            if dist < 5.0 and self._current_alt < 3.0:
                self._sm.handle_event(MissionEvent.HOME_REACHED, f"Near home ({dist:.1f}m)")

    def _do_px4_rtl(self):
        """Delegate RTL to PX4 native mode."""
        self.get_logger().info('Commanding PX4 native RTL')
        if self._mode_client.wait_for_service(timeout_sec=0.0):
            req = SetFlightMode.Request()
            req.mode = 'RTL'
            future = self._mode_client.call_async(req)
            future.add_done_callback(self._on_rtl_mode_response)
        else:
            self.get_logger().warn('SetFlightMode service not available')
            self._rtl_sent = False

    def _do_intelligent_rtl(self):
        """Compute return path avoiding known obstacles."""
        from wardrone_navigation.geo_utils import is_point_near_path, compute_reroute_waypoint

        rtl_alt = self.get_parameter('rtl_altitude_m').value
        clearance = self.get_parameter('rtl_obstacle_clearance_m').value

        self.get_logger().info(
            f'Intelligent RTL: {len(self._obstacle_positions)} obstacles in history'
        )

        # Build waypoint list
        rtl_waypoints = []

        # Step 1: Current position at RTL altitude
        rtl_waypoints.append((self._current_lat, self._current_lon, rtl_alt))

        # Step 2: For each obstacle near the direct path home, add offset waypoint
        for obs_lat, obs_lon in self._obstacle_positions:
            if is_point_near_path(
                self._current_lat, self._current_lon,
                self._home_lat, self._home_lon,
                obs_lat, obs_lon, clearance,
            ):
                result = compute_reroute_waypoint(
                    self._current_lat, self._current_lon,
                    self._home_lat, self._home_lon,
                    obstacle_bearing_deg=0.0,
                    offset_distance_m=clearance,
                )
                if result:
                    rtl_waypoints.append((result[0], result[1], rtl_alt))

        # Step 3: Home position
        rtl_waypoints.append((self._home_lat, self._home_lon, rtl_alt))

        self._rtl_waypoints = rtl_waypoints
        self._rtl_wp_index = 0
        self._intelligent_rtl_active = True
        self._send_next_rtl_waypoint()

    def _send_next_rtl_waypoint(self):
        """Send the next RTL waypoint via GoToWaypoint action."""
        if self._rtl_wp_index >= len(self._rtl_waypoints):
            # All RTL waypoints reached
            self._intelligent_rtl_active = False
            self._sm.handle_event(MissionEvent.HOME_REACHED, "Intelligent RTL complete")
            return

        lat, lon, alt = self._rtl_waypoints[self._rtl_wp_index]
        self.get_logger().info(
            f'RTL waypoint {self._rtl_wp_index + 1}/{len(self._rtl_waypoints)}: '
            f'({lat:.6f}, {lon:.6f}, {alt:.1f}m)'
        )

        if not self._goto_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('GoToWaypoint not available, falling back to PX4 RTL')
            self._intelligent_rtl_active = False
            self._do_px4_rtl()
            return

        goal = GoToWaypoint.Goal()
        goal.waypoint = Waypoint()
        goal.waypoint.latitude_deg = lat
        goal.waypoint.longitude_deg = lon
        goal.waypoint.altitude_m = alt
        goal.waypoint.speed_m_s = 5.0

        future = self._goto_client.send_goal_async(goal)
        future.add_done_callback(self._on_rtl_wp_goal_response)

    def _on_rtl_wp_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('RTL waypoint rejected, falling back to PX4 RTL')
            self._intelligent_rtl_active = False
            self._do_px4_rtl()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rtl_wp_result)

    def _on_rtl_wp_result(self, future):
        result = future.result().result
        if result.success:
            self._rtl_wp_index += 1
            self._send_next_rtl_waypoint()
        else:
            self.get_logger().warn(f'RTL waypoint failed: {result.message}, falling back to PX4 RTL')
            self._intelligent_rtl_active = False
            self._do_px4_rtl()

    def _on_rtl_mode_response(self, future):
        result = future.result()
        if not result.success:
            self.get_logger().error(f'RTL mode failed: {result.message}')
            self._rtl_sent = False

    def _do_land(self):
        """Send land action (once) and let vehicle state detect landing."""
        if self._land_sent:
            return

        self._land_sent = True
        self.get_logger().info('Sending land command')

        if not self._land_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Land action server not available yet')
            self._land_sent = False
            return

        goal = Land.Goal()
        goal.land_in_place = True
        future = self._land_client.send_goal_async(goal)
        future.add_done_callback(self._on_land_goal_response)

    def _on_land_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Land goal rejected')
            self._land_sent = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_land_result)

    def _on_land_result(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Landing complete')
        else:
            self.get_logger().error(f'Land action failed: {result.message}')
            self._land_sent = False

    def _do_search(self):
        """Check search timeout."""
        if self._search_start_time == 0.0:
            self._search_start_time = time.time()

        elapsed = time.time() - self._search_start_time
        if elapsed > self._search_timeout:
            self._sm.handle_event(MissionEvent.SEARCH_TIMEOUT, f"Search timeout ({elapsed:.0f}s)")
            self._search_start_time = 0.0

    # --- State publisher ---

    def _publish_state(self):
        msg = MissionStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state = self._sm.state.value

        if len(self._sm.history) > 0:
            msg.previous_state = self._sm.history[-1].from_state.value
            msg.transition_reason = self._sm.history[-1].reason
        else:
            msg.previous_state = ""
            msg.transition_reason = ""

        msg.mission_elapsed_s = (
            time.time() - self._mission_start_time
            if self._mission_start_time > 0 else 0.0
        )
        msg.current_waypoint_index = self._current_wp_index
        msg.total_waypoints = self._total_waypoints

        self._pub_mission_state.publish(msg)

    # --- Public interface ---

    def start_mission(self):
        """External trigger to start a mission."""
        self._sm.handle_event(MissionEvent.CMD_START, "Manual start")

    def abort_mission(self):
        """External trigger to abort."""
        self._sm.handle_event(MissionEvent.CMD_ABORT, "Manual abort")

    def request_rtl(self):
        """External trigger for return to launch."""
        self._sm.handle_event(MissionEvent.CMD_RTL, "Manual RTL")


def main(args=None):
    rclpy.init(args=args)
    node = MissionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
