-- UNVALIDATED IN THE FIELD (added 2026-08-28): 3D live-SLAM mirror of
-- jackal2_2d_liveslam_toronto.lua for a brief A/B try-out. The options block
-- mirrors the CURRENT 2D contract (extrapolator off, /tracked_pose = raw
-- scan-match pose in odom, EKF in robot_tracker_cpp owns odom->base_link);
-- the TRAJECTORY_BUILDER_3D internals come from the April 2026
-- jackal2_3d_liveslam_outdoor.lua attempt, refreshed where the 2D file has
-- since moved (threads, insertion probabilities, pose-graph cadence).
-- Select via use_3d_carto:=true on stack.launch / cartographer_mapping_3d.launch.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "os_imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true, -- consumers expect strictly planar poses

  -- Mirrors the 2D file (field 2026-08-21): extrapolator OFF, poses published
  -- only at scan-match times; robot_tracker_cpp's EKF owns odom->base_link.
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  publish_tracked_pose = true,
  publish_tracked_pose_in_odom = true, -- fork option, same contract as 2D
  publish_to_tf = true,
  publish_odom_to_published_frame = false, -- fork option: map->odom only

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 1,
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

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 6

-- POSE_GRAPH: cadence and weights mirror the current 2D file.
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- TORONTO WEIGHTS START (2D-mirrored; local-slam weights from the old 3D file)
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0
POSE_GRAPH.optimization_problem.fix_z_in_3d = true
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 0.
POSE_GRAPH.optimization_problem.rotation_weight = 0.
-- TORONTO WEIGHTS END
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 4
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.0
POSE_GRAPH.global_sampling_ratio = 0.0002
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(4.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8

-- TRAJECTORY_BUILDER_3D: from the old outdoor 3D tune.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 200.0
TRAJECTORY_BUILDER_3D.use_intensities = false
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 3. -- high res
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6. -- low res

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 200.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 200.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 30
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
-- Insertion probabilities mirror the 2D file's current values.
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.48

-- Extrapolator: CONSTANT VELOCITY, mirroring the validated 2D setup (upstream
-- 3D default is also CV). The imu_based extrapolator carried over from the
-- April 2026 attempt came from a commit that never worked ("failed to get 3d
-- to work") and produced badly wrong orientation in the 2026-08-28 try-out;
-- it also weighted planar wheel odom (translation weight 1.0) against full-3D
-- IMU orientation, which fight each other on bumps.
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_3D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.005

-- overlapping_submaps_trimmer_2d is 2D-only; no 3D equivalent, so only the
-- localization trimmer carries over.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER.collate_fixed_frame = false

return options
