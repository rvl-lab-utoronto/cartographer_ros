-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "os_imu",
  -- tracking_frame = "imu_link",
  published_frame = "base_link",-- The ROS frame ID to use as the child frame for publishing poses.
                                -- For example “odom” if an “odom” frame is supplied by a different part of the system.
  odom_frame = "odom",
  provide_odom_frame = true, -- publishing a tf between odom_frame and published_frame
                             -- setting this to true when published_frame is also odom gives errors.
  publish_frame_projected_to_2d = true, -- the published pose data is strictly in 2D (x, y, yaw) if this is set to true.
                                         -- This prevents potentially unwanted out-of-plane poses in 2D mode that can occur due to the pose extrapolation step
  use_pose_extrapolator = true,
  use_odometry = true, -- provide odometry topic in launch file
  use_nav_sat = false,
  use_landmarks = false,

  publish_tracked_pose=true, -- publish the pose of the robot in the odom frame
  publish_to_tf = true, -- publish a tf between map_frame and published_frame

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10, -- For multi echo laser scan
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 1.0,
  pose_publish_period_sec = 20e-3,
  trajectory_publish_period_sec = 100e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 32

-- TO GO LIVE:
-- Reverted to frequent optimization for live run stability
POSE_GRAPH.optimize_every_n_nodes = 1
-- POSE_GRAPH.global_sampling_ratio = 0.0002 -- default 0.003
-- POSE_GRAPH.global_constraint_search_after_n_seconds = .0

-- POSE_GRAPH.optimization_problem
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
-- TORONTO WEIGHTS START
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 0. -- Default is typically higher
POSE_GRAPH.optimization_problem.rotation_weight = 0.    -- Default is typically higher
-- TORONTO WEIGHTS END
-- POSE_GRAPH.optimization_problem.log_solver_summary = true

-- POSE_GRAPH.constraint_builder
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 4e3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(4.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 7 -- CPU OPTIMIZATION: Dropped from 8 to speed up loop closure search
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 4

-- TRAJECTORY_BUILDER_3D.pose_extrapolator: params for combining the CVMM with IMU (available at 100Hz) and odom (100Hz) to extrapolate poses before matching
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.num_threads = 2
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.gravity_constant = 9.81
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_rotation_weight = 0.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.pose_translation_weight = 0.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_rotation_weight = 0.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_translation_weight = 1.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 10.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 0.5

-- in order to run real-time? change these from the offline (default) settings
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.25
TRAJECTORY_BUILDER_3D.max_range = 50.0
TRAJECTORY_BUILDER_3D.use_intensities = false
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1 -- Increased from 0.15 (CPU OPTIMIZATION)
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.

-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 80.0
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 20.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 5 -- CPU OPTIMIZATION: Dropped from 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 5. -- High res
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6. -- Low res
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false

-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 50
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 5.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 1.0

-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 50
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 50.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 5.0



-- TRAJECTORY_BUILDER_3D.submaps
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 50
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.2 -- Increased from 0.10 (CPU OPTIMIZATION: Coarser grids for large map area)
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.50 -- Increased from 0.45 (CPU OPTIMIZATION: Coarser grids for large map area)

TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.51
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2

-- END TO GO LIVE

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER.collate_fixed_frame = false

return options