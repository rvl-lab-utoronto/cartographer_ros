-- Live 2D SLAM, Munich (TUM Jackal, Ouster OS1-128 at 2048x10).
--
-- The Toronto live lua plus Munich overrides (2026-10-26), same pattern as
-- jackal2_2d_pure_localization_uoft_campus.lua. Munich therefore runs the Toronto
-- structure: pose extrapolator OFF, /tracked_pose published in odom at scan time,
-- map->odom only, and robot_tracker_cpp's EKF owning odom->base_link
-- (robot_tracker_cpp_toronto.yaml + robot_tracker_cpp_munich.yaml, loaded together
-- in cartographer_mapping_2d.launch). The three fork options and the EKF's
-- publish_tf only work as a pair: never flip one side alone.
--
-- Add an override here only for a real Munich difference. The two below are the
-- values chosen on the Munich robot (cartographer_ros b5debc6, 2026-06-15) that
-- belong to its hardware; the other Munich-trip edits (slower submap / pose /
-- trajectory publish periods, pose_queue_duration) were tied to the old
-- extrapolator-owns-TF setup and now take Toronto's values.
-- UNTUNED (2026-10-26): both overrides and everything inherited from Toronto are
-- untuned for this structure on Munich bags.
include "jackal2_2d_liveslam_toronto.lua"

-- The OS1-128 2048x10 returns 4x the points per scan of Toronto's OS-1-64 1024x10;
-- the Munich run used a coarser voxel (0.2 vs Toronto 0.1) to keep the scan matcher
-- in budget. UNTUNED.
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.2

-- Munich trusted the odometry rotation in the pose graph (Toronto 0.0). UNTUNED.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1.0

return options
