cmake_minimum_required(VERSION 3.5)
project(ball_kicking)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(
  DIRECTORY
    scripts
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/points_cloud_filter.py
  scripts/transform_pointcloud.py
  scripts/front_object_distance.py
  scripts/detections_distance.py
  scripts/start_kick.py
  launch/ball_kicking.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
