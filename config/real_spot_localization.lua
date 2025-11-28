include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Frames (same as your mapping setup that works with Spot)
  map_frame = "map",
  tracking_frame = "body",
  published_frame = "odom",
  odom_frame = "body",
  provide_odom_frame = false,             -- Cartographer publishes map->odom; Spot publishes odom->body
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  -- Inputs (localization benefits from odom)
  use_odometry = true,                    -- enable Spot odometry for a stronger motion prior
  use_nav_sat = false,
  use_landmarks = false,

  -- One point cloud input
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,

  -- Timings (same as mapping)
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

-- VLP-16 @ ~10 Hz (same as mapping where it worked well)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 100.0
TRAJECTORY_BUILDER_2D.min_z = -0.3
TRAJECTORY_BUILDER_2D.max_z = 1.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Keep more nodes during turns (same tweak that helped mapping)
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Preserve detail for matching (same as mapping)
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- Real-time correlative scan matcher (same windows that behaved well)
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window  = 0.15

-- Submaps (keep as in mapping; Cartographer will localize to these)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- Pose graph: disable global loop closures/optimization for pure localization
POSE_GRAPH.optimize_every_n_nodes = 0        -- no global optimization passes
POSE_GRAPH.global_sampling_ratio = 1.0       -- don’t even sample for loop closures
POSE_GRAPH.constraint_builder.min_score = 0.7

-- Keep only a small sliding window of submaps for localization
-- TRAJECTORY_BUILDER.pure_localization = true
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

return options
