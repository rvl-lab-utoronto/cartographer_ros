-- SWEEP candidate (legion replays, 2026-10-07): live lua + bounded window + full_resolution_depth 1 (3 live): the finest
-- levels of each submap grid stack at full resolution; fewer = cheaper first-touch build and less memory, coarser bounds.
include "jackal2_3d_live_SWEEP_window.lua"
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 1
return options
