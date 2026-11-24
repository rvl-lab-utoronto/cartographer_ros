-- DIAGNOSTIC (2026-10-07): offline fast lua + full_resolution_depth 1 for the accuracy/match-count check on Trillium.
include "jackal2_3d_offline_seeded_fast_localization_uoft_campus.lua"
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 1
return options
