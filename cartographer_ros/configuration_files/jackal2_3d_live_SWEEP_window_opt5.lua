-- SWEEP candidate for the robot's live lua (legion replays, 2026-10-07): the live lua plus the lines below only.
include "jackal2_3d_live_localization_uoft_campus.lua"
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(10.)
POSE_GRAPH.optimize_every_n_nodes = 5
return options
