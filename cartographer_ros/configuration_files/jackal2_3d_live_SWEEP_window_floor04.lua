-- SWEEP candidate (legion replays, 2026-10-07): live lua + the bounded window + the constraint score floor at 0.4
-- (the bootstrap floor initial_pose_min_score stays 0.55: the wide bootstrap window must keep its stricter gate).
include "jackal2_3d_live_SWEEP_window.lua"
POSE_GRAPH.constraint_builder.min_score = 0.4
return options
