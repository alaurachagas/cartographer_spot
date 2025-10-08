include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",

  tracking_frame = "body",              -- Spot IMU/body in TF
  published_frame = "odom",             -- publish the robot body pose (NOT "odom")
  odom_frame = "odom",
  provide_odom_frame = false,           -- Spot already publishes odom->body

  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,

  use_odometry = false,                 -- keep false unless you wire Spot odom into Cartographer
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                 -- VLP-16 as PointCloud2

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

-- Velodyne VLP-16 tuning (10 Hz)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1     -- one cloud per node (was 2)
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 100.0
TRAJECTORY_BUILDER_2D.min_z = -0.45
TRAJECTORY_BUILDER_2D.max_z = 1.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- Keep more nodes when rotating fast (reduce drop threshold)
-- Default angle threshold is ~0.1 rad (~5.7 deg); 0.5 deg keeps more nodes during turns.
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)

-- Slightly denser input to preserve features for scan matching
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025          -- was 0.05

-- Real-time correlative scan matcher: widen a bit for quick yaw
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(15.0)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window  = 0.15

-- Submaps & pose graph
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90        -- OK for 10 Hz; ~9 s per submap
POSE_GRAPH.optimize_every_n_nodes = 90                   -- optimize each submap
POSE_GRAPH.constraint_builder.min_score = 0.7            -- stricter loop closures

-- Mapping (not pure localization)
-- TRAJECTORY_BUILDER.pure_localization_trimmer = { max_submaps_to_keep = 3, }

return options
