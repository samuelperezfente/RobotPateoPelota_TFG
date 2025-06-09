cmake_minimum_required(VERSION 3.5)
project(ball_kicking_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DetectBallAndGoal.action"
  DEPENDENCIES builtin_interfaces action_msgs geometry_msgs
)

ament_package()
