-- ms616_cartographer.lua
-- Cartographer config for ms616: 2D LiDAR SLAM with RAW wheel odom + IMU.
-- Clean baseline: Cartographer receives /diff_drive_controller/odom directly;
-- robot_localization EKF is intentionally not in this SLAM loop.
--
-- PLACE THIS IN: <ms616_pkg>/config/ms616_cartographer.lua
--
-- Two distinct sensor-fusion layers are in play, deliberately:
--   1. robot_localization's EKF fuses odom+IMU BEFORE this file even
--      sees data — that's what makes /odometry/filtered cleaner than
--      raw wheel odom.
--   2. Cartographer ALSO takes raw IMU directly (use_imu_data = true
--      below) for its own internal scan-matching correction — this is
--      independent of #1 and helps specifically with per-scan
--      rotational drift during fast turns, which is exactly the kind
--      of local-minimum/skew problem visible in your screenshot.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",   -- CHECK: must match the IMU link name
                                   -- from the URDF snippet. Cartographer
                                   -- wants the tracking frame co-located
                                   -- with the IMU when IMU data is used.
  published_frame = "odom",       -- Cartographer publishes map -> odom;
                                   -- odom -> base_link comes from the EKF
  odom_frame = "odom",
  provide_odom_frame = false,     -- false because robot_localization's
                                   -- EKF is already providing odom ->
                                   -- base_link; Cartographer should only
                                   -- publish map -> odom, not compete
                                   -- over the same frame
  publish_frame_projected_to_2d = true,
  use_odometry = true,            -- consume /odometry/filtered
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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

MAP_BUILDER.use_trajectory_builder_2d = true

-- ============================================================
-- TRAJECTORY BUILDER — this is where most of your drift fixes live
-- ============================================================

TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 8.0     -- CHECK: match your lidar's real max range
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- Ceres scan matcher weights — raising rotation_weight relative to
-- translation_weight specifically targets the yaw-drift-driven skew
-- visible in your screenshot (the whole map rotated as one rigid block).
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20

-- Real-time correlative scan matcher: helps recover from a bad local
-- minimum by doing a brute-force search around the current estimate
-- before ceres refines it. Turn this ON given you're already seeing
-- local-minimum failures — the cost is CPU, which is worth it here.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0

-- Submap size: smaller submaps = more frequent local consistency
-- checks, which helps in a maze with lots of tight turns and
-- repetitive-looking corridors (a classic scan-matching confusion
-- case — narrow corridors look alike, encouraging wrong local minima)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Motion filter: drop near-duplicate scans so the pose graph isn't
-- doing redundant work — but don't set the thresholds so loose that
-- fast rotation gets under-sampled (another yaw-drift contributor).
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- ============================================================
-- POSE GRAPH — global optimization / loop closure
-- ============================================================

-- Optimize more frequently than the 90-node default. In a small
-- maze this is cheap and gives you tighter correction of accumulated
-- drift before it compounds into the kind of skew in your screenshot.
POSE_GRAPH.optimize_every_n_nodes = 25

POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- Loop closure search radius — keep tight for a small maze;
-- loosen (and accept slower loop closure) if the environment is
-- larger in reality than in sim.
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options