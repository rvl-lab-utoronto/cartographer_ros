-- 3D LOCAL variant 'loopclose_w1' on the LOOP bag (2026-09-21). Refinement of 'loopclose', which was the
-- first configuration to close the 534 m loop (big-loop error 0.51 m, constraint gap 474 s).
-- Attribution from the finished runs, big-loop xy / z / did it close:
--     baseline            4.52 / 1.18  no      z window 20 alone   4.55 / 1.18  no
--     translation_w 2     1.39 / 13.46 no      z window 20 + free z 6.18 / 9.77 no
--     translation_w 1     1.10 / 7.72  no      loopclose (w2+win+free) 0.51/0.19 YES
--     translation_w 0.5   3.08 / 3.44  no
-- Widening the z window alone never closed it; good local xy was the necessary ingredient, and
-- translation_weight 1 is the best local xy measured, better than the 2 that loopclose used.
-- Lower weight also means less z drift (37.2 / 15.3 / 10.2 m at 2 / 1 / 0.5) because the matcher
-- is pulled less by the tilted odometry prior, but xy degrades below 1: this is the isotropic
-- single-weight tension of section 3.7, and 1 is the knee.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 20.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.
POSE_GRAPH.constraint_builder.max_constraint_distance = 40.
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1.
return options
