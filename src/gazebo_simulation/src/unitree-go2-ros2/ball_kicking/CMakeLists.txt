cmake_minimum_required(VERSION 3.5)
project(ball_kicking)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)

install(
  DIRECTORY
    scripts
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/ball_searching.py
  scripts/move_to_target.py
  scripts/ball_goal_searching.py
  scripts/ball_goal_searching_action_server.py
  scripts/calculate_and_move_to_ball.py
  scripts/calculate_and_move_to_ball_action_server.py
  scripts/low_level_pid_controller.py
  scripts/orchestrator.py
  launch/ball_kicking.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

