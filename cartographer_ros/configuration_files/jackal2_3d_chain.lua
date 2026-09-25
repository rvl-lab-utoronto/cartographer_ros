-- Incremental 3D campus chain: one new bag per stage on top of the FROZEN previous stage.
-- Inputs are the v3 carto_inputs (ground-in filtered_point_cloud3, accelerometer-bias-corrected
-- IMU), which put the measured z error at a 700 m return under 1 m (TUNING_README_3D.md 12),
-- so the search windows come down from the groundless crutches.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.   -- seeds from the 2D map land up to ~7 m off (bag 3 measured); 6 missed it
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 5.
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.
POSE_GRAPH.optimize_every_n_nodes = 300   -- 90 was the measured campus bottleneck (478 solves in 15 min)
return options
