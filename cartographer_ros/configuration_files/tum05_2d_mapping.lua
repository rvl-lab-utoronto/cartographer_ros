-- Munich tum05 2D offline mapping for the Stage-1 incremental chain (chain2d, 2026-10-26).
-- = the Toronto campus 2D mapping lua (jackal2_2d_mapping_uoft_campus.lua: local SLAM 0.5/1/40-scan, max_range 200 m, odometry on)
-- + the settings the Toronto chain finally ran with (campus-map-weights-and-overlap notes, 2026-09-16/17), which the committed
--   campus lua no longer carries:
--     pose graph  local_slam_pose_translation_weight 1e4, huber 5e3, 100 iterations
--     closure search  window 60 s, global sampling 0.03, constraint sampling SAMPLING (0.5; Toronto 1.0 then 0.15)
--     front end   50-scan submaps, max_range 60 (adaptive + loop closure filters), adaptive 0.5 m / 200 pts
-- tum05 frames and topics are the Toronto ones (tracking os_imu, published base_link, odom on /jackal_velocity_controller/odom,
-- imu /ouster_imu_debiased, points2 /filtered_point_cloud2 from the carto_input bags): nothing frame specific to override.
include "jackal2_2d_mapping_uoft_campus.lua"

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e4
POSE_GRAPH.optimization_problem.huber_scale = 5e3
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.global_constraint_search_after_n_seconds = 60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.global_sampling_ratio = 0.03
TRAJECTORY_BUILDER_2D.max_range = 60.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 60.
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 60.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50
return options
