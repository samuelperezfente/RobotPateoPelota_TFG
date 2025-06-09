include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "velodyne",          -- frame del LIDAR (verifica tu TF)
  published_frame = "base_link",        -- frame base del robot
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  publish_frame_projected_to_2d = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                  -- Usamos 1 nube de puntos
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- Parámetros para el LIDAR Velodyne
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 100.
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05

-- Ajustes del scan matcher 3D
TRAJECTORY_BUILDER_3D.ceres_scan_matcher = {
  occupied_space_weight = 20.,
  translation_weight = 10.,
  rotation_weight = 40.,
}

-- Parámetros del submap 3D
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 90
TRAJECTORY_BUILDER_3D.submaps.grid_options_3d.resolution = 0.1

return options
