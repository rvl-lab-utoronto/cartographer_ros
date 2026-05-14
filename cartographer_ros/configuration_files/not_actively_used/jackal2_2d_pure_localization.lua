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

include "jackal2_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

options.provide_odom_frame = true

options.pose_publish_period_sec = 1./50.
options.trajectory_publish_period_sec = 1./25.

options.odometry_sampling_ratio = 1.
options.num_point_clouds = 1



POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 12
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.global_sampling_ratio = 0.0002 -- default 0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 4e3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(3.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 15
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 12
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.global_constraint_search_after_n_seconds = 5.0


MAP_BUILDER.num_background_threads = 6

-- in order to run real-time? change these from the offline (default) settings
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 6
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.num_threads = 6
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 50
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 20
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.max_range = 60.0
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1

TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.25





-- options.tracking_frame = "base_link"
-- TRAJECTORY_BUILDER_2D.min_z = 0.25
-- options.tracking_frame = "os_imu"
-- options.tracking_frame = "imu_link",
-- options.imu_sampling_ratio = 1.
-- TRAJECTORY_BUILDER_2D.use_imu_data = false -- false to disable the use of IMU data


-- No effect on localization accuracy, but increases computational load.
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.20 -- default 0.15
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)

-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.1 -- default 0.5
-- TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.01 -- default 0.1
-- TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004 -- default 0.004

-- TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false -- default false
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false -- default false

-- Reducing Global Latency
-- POSE_GRAPH.optimize_every_n_nodes = 5 -- default 100
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false -- default false -- for more computationally intensive localization
-- -- TRAJECTORY_BUILDER.pure_localization = true
-- POSE_GRAPH.global_sampling_ratio = 0.0003 -- default 0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 -- default 0.3
-- MAP_BUILDER.num_background_threads = 8 -- default 4 -- This should be the same as the number of cores which I am not sure about
-- POSE_GRAPH.constraint_builder.min_score = 0.75 -- default 0.55
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2 -- default 5.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 0.5 -- default 1.
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(5.) -- default math.rad(15.)
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- default 10

-- -- Tuning Local SLAM for Lower Latency
-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.3 -- default 0.15
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.25 -- default 0.1
-- TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.6 -- default 0.45
-- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 50 -- default 160

-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100 -- default 150
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 10. -- default 15.
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 3. -- default 2.

-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 125 -- default 200
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 30. -- default 60.
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 5. -- default 4.

-- POSE_GRAPH.optimization_problem.log_solver_summary = true


return options
















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

-- include "jackal2_2d.lua"

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

-- options.provide_odom_frame = true

-- options.pose_publish_period_sec = 1./50.
-- options.trajectory_publish_period_sec = 1./25.

-- options.odometry_sampling_ratio = 1.
-- options.num_point_clouds = 1



-- POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.optimize_every_n_nodes = 1
-- POSE_GRAPH.global_sampling_ratio = 0.0003 -- default 0.003
-- -- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
-- -- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 4e3
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 1.0
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(3.)
-- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 15
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 12
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- POSE_GRAPH.global_constraint_search_after_n_seconds = 1.0


-- MAP_BUILDER.num_background_threads = 6

-- -- in order to run real-time? change these from the offline (default) settings
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 6
-- TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.num_threads = 6
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.50

-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 10 -- was 4 and it seemed to be fine, but then it really started to fly to other parts of the map when it. NB it's 50 on the legion. But i the issue that might've actually been num_range_data below being too low at 20 instead of 40.
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 75.0
-- TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 20.0
-- TRAJECTORY_BUILDER_2D.max_range = 40.0
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1

-- TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.25





-- -- options.tracking_frame = "base_link"
-- -- TRAJECTORY_BUILDER_2D.min_z = 0.25
-- -- options.tracking_frame = "os_imu"
-- -- options.tracking_frame = "imu_link",
-- -- options.imu_sampling_ratio = 1.
-- -- TRAJECTORY_BUILDER_2D.use_imu_data = false -- false to disable the use of IMU data


-- -- No effect on localization accuracy, but increases computational load.
-- -- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.20 -- default 0.15
-- -- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)

-- -- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.1 -- default 0.5
-- -- TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.01 -- default 0.1
-- -- TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004 -- default 0.004

-- -- TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false -- default false
-- -- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false -- default false

-- -- Reducing Global Latency
-- -- POSE_GRAPH.optimize_every_n_nodes = 5 -- default 100
-- -- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false -- default false -- for more computationally intensive localization
-- -- -- TRAJECTORY_BUILDER.pure_localization = true
-- -- POSE_GRAPH.global_sampling_ratio = 0.0003 -- default 0.003
-- -- POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 -- default 0.3
-- -- MAP_BUILDER.num_background_threads = 8 -- default 4 -- This should be the same as the number of cores which I am not sure about
-- -- POSE_GRAPH.constraint_builder.min_score = 0.75 -- default 0.55
-- -- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2 -- default 5.
-- -- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 0.5 -- default 1.
-- -- POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(5.) -- default math.rad(15.)
-- -- POSE_GRAPH.global_constraint_search_after_n_seconds = 30 -- default 10

-- -- -- Tuning Local SLAM for Lower Latency
-- -- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.3 -- default 0.15
-- -- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.25 -- default 0.1
-- -- TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.6 -- default 0.45
-- -- TRAJECTORY_BUILDER_3D.submaps.num_range_data = 50 -- default 160

-- -- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100 -- default 150
-- -- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 10. -- default 15.
-- -- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 3. -- default 2.

-- -- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 125 -- default 200
-- -- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 30. -- default 60.
-- -- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 5. -- default 4.

-- -- POSE_GRAPH.optimization_problem.log_solver_summary = true


-- return options
