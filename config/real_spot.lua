include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "velodyne",    -- Spot IMU frame name from TF
  published_frame = "body",       -- Spot's body frame
  odom_frame = "odom",
  provide_odom_frame = false,          -- Spot already publishes odom
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,                 -- Spot odometry is very reliable
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                 -- VLP-16 outputs PointCloud2
  lookup_transform_timeout_sec = 0.5,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- Velodyne VLP-16 tuning
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2   -- aggregate 2 sweeps
TRAJECTORY_BUILDER_2D.min_range = 0.5                  -- VLP-16 spec
TRAJECTORY_BUILDER_2D.max_range = 100.0                -- VLP-16 max range
TRAJECTORY_BUILDER_2D.min_z = -0.3                     -- filter ground hits
TRAJECTORY_BUILDER_2D.max_z = 1.0                      -- filter high returns
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90       -- bigger submaps for less noise
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Pose graph tuning for Spot
POSE_GRAPH.optimize_every_n_nodes = 90                  -- less frequent optimization
POSE_GRAPH.constraint_builder.min_score = 0.7           -- stricter loop closures

-- Comment out pure localization mode (we are mapping)
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

return options
