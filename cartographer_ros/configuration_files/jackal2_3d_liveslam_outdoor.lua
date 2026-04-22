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


POSE_GRAPH.optimize_every_n_nodes = 1
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- TO GO LIVE:
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 12
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.global_sampling_ratio = 0.0002 -- default 0.003
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 4e3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 0.5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(4.)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 12
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 12
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.0
POSE_GRAPH.optimization_problem.fix_z_in_3d = true
-- TORONTO WEIGHTS START
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.0
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.0
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.acceleration_weight = 0. -- Default is typically higher
POSE_GRAPH.optimization_problem.rotation_weight = 0.    -- Default is typically higher
-- TORONTO WEIGHTS END



-- in order to run real-time? change these from the offline (default) settings

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 200.0
TRAJECTORY_BUILDER_3D.use_intensities = false

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 12
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 3. -- High res
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6. -- Low res


-- calculated from measurements from rosbag in Munich.

TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 200.0
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 200.0
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 30
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.1

TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.solver_options.num_threads = 6
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.gravity_constant = 9.81
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_rotation_weight = 0.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.odometry_translation_weight = 1.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 5.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 1.0

TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45

-- END TO GO LIVE


-- POSE_GRAPH.optimization_problem.log_solver_summary = true

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}

return options
