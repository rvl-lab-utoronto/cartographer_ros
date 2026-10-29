-- exp3d_extend.lua (frozen-map extension, seeded) with the cross-session score floors lowered for pure localization on a map recorded on other days
-- (memory carto-localization-tuning: floor 0.55 finds nothing on a seasonally changed map, 0.3 gave zero wrong matches on 4 bags; cartographer defaults here are min_score 0.62,
-- rotational 0.77, low resolution 0.55). The seed is a 2D-map placement good to ~1 m, so the local window cannot reach a wrong place; every constraint is gated by its residual afterwards.
include "exp3d_extend.lua"
POSE_GRAPH.constraint_builder.min_score = 0.40
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.60
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.40
return options
