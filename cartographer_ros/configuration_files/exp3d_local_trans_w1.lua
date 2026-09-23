-- 3D LOCAL tuning variant 'trans_w1' (tune_local_parallel_3d.sh, 2026-09-21). Global SLAM off.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0   -- every revisit gets a match attempt: the metric needs them all
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1.
return options
