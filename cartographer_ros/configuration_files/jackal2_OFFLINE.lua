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
  tracking_frame = "imu_link",
  published_frame = "base_link", -- The ROS frame ID to use as the child frame for publishing poses.
                            -- For example “odom” if an “odom” frame is supplied by a different part of the system.
  odom_frame = "odom",
  provide_odom_frame = true, -- publishing a tf between odom_frame and published_frame
                             -- setting this to true when published_frame is also odom gives errors.
  publish_frame_projected_to_2d = true, -- the published pose data is strictly in 2D (x, y, yaw) if this is set to true.
                                        -- This prevents potentially unwanted out-of-plane poses in 2D mode that can occur due to the pose extrapolation step
                                        -- Doesn't change the values in /tracked_pose but the published transform between odom and base_link only has x, y, yaw
  use_pose_extrapolator = true, -- Used to specify whether Cartographer should utilize a pose extrapolator to estimate the robot's pose between scan matches
                                -- The idea behind the pose extrapolator is to use sensor data of other sensors besides the range finder to predict where the next scan should be inserted into the submap
                                -- Uses odometry and IMU

  use_odometry = false, -- provide odometry topic in launch file
  use_nav_sat = false, -- sep oct 12, for outdoor bag
  use_landmarks = false,
  publish_tracked_pose=true,
  publish_to_tf = true, -- publish a tf between map_frame and published_frame
  num_laser_scans = 0, -- provide laser scan topic in launch file
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2, -- provide pointcloud topic in launch file
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.,
}

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 5
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true

MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 30
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimize_every_n_nodes = 1
POSE_GRAPH.constraint_builder.sampling_ratio = 0.25
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 1000
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.log_residual_histograms = false

POSE_GRAPH.optimization_problem.fix_z_in_3d = true
-- POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true

-- TRAJECTORY_BUILDER_3D.use_intensities = true
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.intensity_cost_function_options_0.weight = 0.2

-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05 -- default 0.10
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20. -- default 20 -- try higher
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true
-- POSE_GRAPH.optimization_problem.huber_scale = 5e-1



return options
