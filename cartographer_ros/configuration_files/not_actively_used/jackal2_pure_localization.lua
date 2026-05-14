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

include "jackal2.lua"



TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- POSE_GRAPH.overlapping_submaps_trimmer_2d = {
--   fresh_submaps_count = 2,
--   min_covered_area = 9,
--   min_added_submaps_count = 40,
-- }

-- No effect on localization accuracy, but increases computational load.
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.20 -- default 0.15
-- TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)

-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.1 -- default 0.5
-- TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.01 -- default 0.1
-- TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004 -- default 0.004

-- TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false -- default false
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false -- default false

-- Reducing Global Latency
POSE_GRAPH.optimize_every_n_nodes = 1 -- default 100
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true -- default false -- for more computationally intensive localization
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

-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true -- default false
-- POSE_GRAPH.optimization_problem.fix_z_in_3d = true -- default false

-- TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true -- try later

return options
