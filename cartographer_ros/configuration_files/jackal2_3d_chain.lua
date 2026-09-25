-- Incremental 3D campus chain: one new bag per stage on top of the FROZEN previous stage.
-- Inputs are the v3 carto_inputs (ground-in filtered_point_cloud3, accelerometer-bias-corrected
-- IMU), which put the measured z error at a 700 m return under 1 m (TUNING_README_3D.md 12),
-- so the search windows come down from the groundless crutches.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.   -- seeds from the 2D map land up to ~7 m off (bag 3 measured); 6 missed it
-- z window sized to the worst offender actually measured post-fix, not the typical case: a
-- bag whose route mostly does not overlap anything yet (08-24 18:04, in the pre-reorder chain)
-- drifted +7.2..+7.9 m in z before finding any cross constraint, across every bias value
-- tested (0.133 / 0.20 / its own 0.315 stop estimate) -- that drift is a COVERAGE gap, not a
-- bias error, so the fix here is margin, not a smaller number. 15 m covers 7.9 m worst-case
-- with ~2x headroom; max_constraint_distance (a 3D radius) raised to 25 so it does not clip
-- the corner case of a near-max xy AND near-max z offset at once (sqrt(10^2+15^2) = 18).
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 15.
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.
POSE_GRAPH.optimize_every_n_nodes = 300   -- 90 was the measured campus bottleneck (478 solves in 15 min)
return options
