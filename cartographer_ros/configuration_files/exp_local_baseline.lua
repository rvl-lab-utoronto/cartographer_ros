-- LOCAL tuning variant 'baseline' (tune_local_parallel.sh, 2026-09-17). Global SLAM off.
include "jackal2_2d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0   -- every revisit gets a match attempt: the metric needs them all
return options
