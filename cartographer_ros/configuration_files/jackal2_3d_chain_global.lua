-- Chain stage for a bag with NO untrimmed start pose anywhere (its start was trimmed as overlap
-- in every 2D file). The full bag runs unseeded and cartographer's GLOBAL constraint search
-- localizes it against the frozen map, the same mechanism pure localization uses. Search is
-- sampled (global_sampling_ratio), so cost stays bounded. An unconnected result is set aside
-- by chain3d.sh, never carried forward.
include "jackal2_3d_chain.lua"
POSE_GRAPH.global_sampling_ratio = 0.01                       -- upstream 0.003; one bag at a time, spend a bit more
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.     -- upstream default (the chain lua had 1e9 = never)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
return options
