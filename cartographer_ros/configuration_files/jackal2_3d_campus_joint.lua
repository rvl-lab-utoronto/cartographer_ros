-- FULL CAMPUS 3D map, joint offline run over the seeded carto_input bags (2026-09-21).
--
-- Inherits the tuned single-bag config, which closes the 534 m King's College Circle loop at
-- 0.28 m against the tuned 2D map's 0.40 m. Two things a multi-trajectory run must change:
--
-- 1. GLOBAL branch-and-bound search OFF. 2D measured it producing 26k to 53k work-item queues
--    with no result in ten minutes, and worse, placing bags confidently and WRONGLY: a bag scored
--    hundreds of self-consistent constraints while sitting 14 to 44 deg out of true. Every bag
--    here is instead SEEDED from the untrimmed 2D map via -initial_trajectory_poses. 3D makes
--    that mandatory rather than merely preferable: the fork's initial_pose_* bootstrap exists
--    only in constraint_builder_2d, so an unseeded 3D bag has nothing to fall back on.
--
-- 2. CONSTRAINT SEARCH SIZED TO THE SEEDED ERROR, not to single-bag drift. The single-bag tuning
--    needed a 10 m xy / 20 m z window and a 40 m constraint distance because local SLAM arrived
--    back metres out after 534 m with nothing correcting it. That reasoning does NOT carry over:
--    a seeded bag starts sub-metre from truth. Measured cost of getting this wrong, on the first
--    attempt with the single-bag windows: ~5 constraints/s at 620 % CPU, only 8 of 23
--    trajectories searched in 25 minutes, extrapolating to 3-4 hours before the final
--    optimization. Search cost grows with window volume AND with candidate count, which goes as
--    max_constraint_distance squared, so these values are roughly an order of magnitude cheaper.
--    z stays the loosest axis because the seed flag carries only x, y and yaw: every bag starts
--    at z = 0 and then drifts within itself on a cloud that is 0.5 % horizontal surface.
include "jackal2_3d_mapping_uoft_campus.lua"

POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 6.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 10.
POSE_GRAPH.constraint_builder.max_constraint_distance = 20.
return options
