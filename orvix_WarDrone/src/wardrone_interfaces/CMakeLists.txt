cmake_minimum_required(VERSION 3.8)
project(wardrone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/Telemetry.msg"
  "msg/VehicleState.msg"
  "msg/Waypoint.msg"
  "msg/WaypointList.msg"
  "msg/Detection.msg"
  "msg/DetectionArray.msg"
  "msg/TrackingTarget.msg"
  "msg/MissionState.msg"
  "msg/VioStatus.msg"
  "msg/Obstacle.msg"
  "msg/ObstacleArray.msg"
  "msg/PreflightStatus.msg"
  "msg/WindEstimate.msg"
  # Services
  "srv/Arm.srv"
  "srv/SetFlightMode.srv"
  "srv/LoadMission.srv"
  "srv/SetTrackingTarget.srv"
  # Actions
  "action/Takeoff.action"
  "action/Land.action"
  "action/ExecuteMission.action"
  "action/GoToWaypoint.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

ament_package()
