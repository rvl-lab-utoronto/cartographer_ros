-- tum05 2D mapping for a SEEDED bag (-initial_trajectory_poses): no whole-map search, lock-on bootstrap on. Same blocks as
-- jackal2_2d_seeded_uoft_campus.lua (Toronto), on top of tum05_2d_mapping.lua.
include "tum05_2d_mapping.lua"
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.constraint_builder.initial_pose_num_nodes = 100000
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 15.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.55
return options
