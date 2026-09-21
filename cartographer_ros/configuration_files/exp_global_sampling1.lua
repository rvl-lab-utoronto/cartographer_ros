-- GLOBAL tuning variant 'sampling1' (tune_global_bag12.sh, 2026-09-17). Seeded bag on an unfrozen base.
include "jackal2_2d_mapping_uoft_campus.lua"
POSE_GRAPH.constraint_builder.initial_pose_num_nodes = 100000
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 15.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.55
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
return options
