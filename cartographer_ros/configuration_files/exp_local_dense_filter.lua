-- LOCAL tuning variant 'dense_filter' (tune_local_parallel.sh, 2026-09-17). Global SLAM off.
include "jackal2_2d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0   -- every revisit gets a match attempt: the metric needs them all
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
return options
