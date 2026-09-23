-- 3D LOCAL variant 'loopclose' on the LOOP bag (2026-09-21): the first configuration that should
-- be able to CLOSE the 534 m loop, built from measured gate-by-gate failures rather than guesses.
--
-- Big-loop error measured against the 2D original map, which says which two moments are the same
-- place (0.13 m apart, 529 m of driving between):
--     baseline          xy 4.52 m   z  1.18 m   -> z misses the 1 m z window
--     translation_w 2   xy 1.39 m   z 13.46 m   -> xy fine, z misses by a mile
--     zwin20 + free z   xy 6.18 m   z  9.77 m   -> z fine now, xy misses the 5 m xy window
-- So no single change gets both ends inside their windows at once. This one takes the local
-- setting that made xy good and opens the two search windows far enough to cover what is left.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0

-- local SLAM: the knob that took big-loop xy from 4.52 m to 1.39 m (upstream is 5.)
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 2.

-- search windows, sized to the measured residual error and not to taste
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 20.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.
POSE_GRAPH.constraint_builder.max_constraint_distance = 40.   -- 3D distance: must admit the z error too

-- and let the constraint, once found, actually move z; this is also what switches on the pose
-- graph's IMU residuals and use_online_imu_extrinsics_in_3d (optimization_problem_3d.cc:354)
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
return options
