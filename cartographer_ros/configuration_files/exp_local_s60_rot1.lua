-- LOCAL tuning variant 's60_rot1' (tune_local_parallel.sh, 2026-09-17). Global SLAM off.
include "jackal2_2d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0   -- every revisit gets a match attempt: the metric needs them all
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
return options
