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

  publish_tracked_pose = true, -- publish the pose of the robot in the odom frame
  publish_to_tf = true, -- publish a tf between map_frame and published_frame

  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10, -- For multi echo laser scan
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

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 6


-- TO GO LIVE:
-- Every-node optimization minimizes drift quickly but jitters map/odom as constraints
-- get re-solved. Try 2–5 if idle pose looks too noisy; increase if global drift worsens.
POSE_GRAPH.optimize_every_n_nodes = 1
-- POSE_GRAPH.global_constraint_search_after_n_seconds = .0

-- POSE_GRAPH.optimization_problem
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- TORONTO WEIGHTS START
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0
POSE_GRAPH.optimization_problem.fix_z_in_3d = true
-- TORONTO WEIGHTS END
-- POSE_GRAPH.optimization_problem.log_solver_summary = true

-- POSE_GRAPH.constraint_builder
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 4
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.0



-- in order to run real-time? change these from the offline (default) settings
-- 2–3 : slightly smoother local matches, +CPU/latency; keep 1 if the platform is marginal.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.use_imu_data = true -- false to disable the use of IMU data
TRAJECTORY_BUILDER_2D.min_z = -100.
TRAJECTORY_BUILDER_2D.max_z = 100.
TRAJECTORY_BUILDER_2D.max_range = 200.0
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.1

-- TRAJECTORY_BUILDER_2D.pose_extrapolator: params for combining the CVMM with IMU and odom to extrapolate poses before matching
-- NOTE: use_imu_based = true is NOT supported in 2D — local_trajectory_builder_2d.cc has a hard CHECK(!use_imu_based). IMU is still used via use_imu_data = true.
-- TRAJECTORY_BUILDER_2D.pose_extrapolator.use_imu_based = false -- NB imu_based is NOT USED IN 2D
TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.imu_gravity_time_constant = 10.
TRAJECTORY_BUILDER_2D.pose_extrapolator.constant_velocity.pose_queue_duration = 0.005

-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 8.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 4
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.

-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter
-- calculated from measurements from rosbag in Munich.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 200.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 1.0

-- TRAJECTORY_BUILDER_2D.submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.48

-- END TO GO LIVE

POSE_GRAPH.overlapping_submaps_trimmer_2d = {
  fresh_submaps_count = 2,
  min_covered_area = 10,
  min_added_submaps_count = 3,
}

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER.collate_fixed_frame = false

return options


-- --   CONFIG FROM MUNICH IS BELOW
-- -- Copyright 2016 The Cartographer Authors
-- --
-- -- Licensed under the Apache License, Version 2.0 (the "License");
-- -- you may not use this file except in compliance with the License.
-- -- You may obtain a copy of the License at
-- --
-- --      http://www.apache.org/licenses/LICENSE-2.0
-- --
-- -- Unless required by applicable law or agreed to in writing, software
-- -- distributed under the License is distributed on an "AS IS" BASIS,
-- -- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- -- See the License for the specific language governing permissions and
-- -- limitations under the License.

-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "os_imu",
--   -- tracking_frame = "imu_link",
--   published_frame = "base_link",-- The ROS frame ID to use as the child frame for publishing poses.
--                                 -- For example “odom” if an “odom” frame is supplied by a different part of the system.
--   odom_frame = "odom",
--   provide_odom_frame = true, -- publishing a tf between odom_frame and published_frame
--                              -- setting this to true when published_frame is also odom gives errors.
--   publish_frame_projected_to_2d = true, -- the published pose data is strictly in 2D (x, y, yaw) if this is set to true.
--                                          -- This prevents potentially unwanted out-of-plane poses in 2D mode that can occur due to the pose extrapolation step
--   use_pose_extrapolator = true,
--   use_odometry = true, -- provide odometry topic in launch file
--   use_nav_sat = false,
--   use_landmarks = false,

--   publish_tracked_pose=true, -- publish the pose of the robot in the odom frame
--   publish_to_tf = true, -- publish a tf between map_frame and published_frame

--   num_laser_scans = 0,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 10, -- For multi echo laser scan
--   num_point_clouds = 1,
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_2d = true
-- MAP_BUILDER.num_background_threads = 6
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 6
-- TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.num_threads = 6
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 6
-- POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 6
-- POSE_GRAPH.optimize_every_n_nodes = 0
-- TRAJECTORY_BUILDER_2D.use_imu_data = true -- false to disable the use of IMU data
-- -- TRAJECTORY_BUILDER_2D.min_z = -0.15
-- TRAJECTORY_BUILDER_2D.min_z = -100.
-- TRAJECTORY_BUILDER_2D.max_z = 100.
-- -- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- -- TO GO LIVE:
-- POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.optimize_every_n_nodes = 1
-- POSE_GRAPH.global_sampling_ratio = 0.0002 -- default 0.003
-- -- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 4e3
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(4.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 15
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 15.0

-- MAP_BUILDER.num_background_threads = 6

-- -- in order to run real-time? change these from the offline (default) settings
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 6


-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e2

-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.25
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 30
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 15.0
-- TRAJECTORY_BUILDER_2D.max_range = 100.0
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 30
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.25

-- -- END TO GO LIVE


-- -- POSE_GRAPH.optimization_problem.log_solver_summary = true
-- POSE_GRAPH.overlapping_submaps_trimmer_2d = {
--   fresh_submaps_count = 2,
--   min_covered_area = 10,
--   min_added_submaps_count = 3,
-- }

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 5,
-- }

-- return options