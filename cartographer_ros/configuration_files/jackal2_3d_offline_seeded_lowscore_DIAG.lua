-- DIAGNOSTIC ONLY (2026-10-07): the offline seeded lua with the match score floors dropped to 0.2, to see the score
-- distribution of the constraints the live floors (0.55) reject. Never for production runs.
include "jackal2_3d_offline_seeded_localization_uoft_campus.lua"
POSE_GRAPH.constraint_builder.min_score = 0.2
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.2
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.2
return options
