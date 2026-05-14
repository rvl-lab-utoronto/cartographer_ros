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
  -- tracking_frame = "base_link",
  -- tracking_frame = "base_link",
  -- tracking_frame = "imu_link",
  tracking_frame = "os_imu",
  published_frame = "base_link",-- The ROS frame ID to use as the child frame for publishing poses.
                                -- For example “odom” if an “odom” frame is supplied by a different part of the system.
  odom_frame = "odom",
  provide_odom_frame = false, -- publishing a tf between odom_frame and published_frame
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
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1./5.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1./2.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 30
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 30
TRAJECTORY_BUILDER_2D.pose_extrapolator.imu_based.solver_options.num_threads = 30
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 30
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 30
POSE_GRAPH.optimize_every_n_nodes = 1
TRAJECTORY_BUILDER_2D.use_imu_data = true -- false to disable the use of IMU data
TRAJECTORY_BUILDER_2D.min_z = -0.50
-- TRAJECTORY_BUILDER_2D.min_z = 0.405

-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.overlapping_submaps_trimmer_2d = {
  fresh_submaps_count = 2,
  min_covered_area = 9,
  min_added_submaps_count = 40,
}

return options
