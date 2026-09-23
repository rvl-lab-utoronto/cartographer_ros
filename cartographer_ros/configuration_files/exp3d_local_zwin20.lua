-- 3D LOCAL variant 'zwin20' on the LOOP bag (2026-09-21): can the big loop be FOUND at all?
-- PRECONDITION, not a tuning knob: fast_correlative_scan_matcher_3d searches only
-- linear_z_search_window = 1 m in z, while local SLAM has drifted 6 to 7 m vertically by the time
-- the robot returns to the start. Measured on the baseline loop-bag run: all 128 revisit
-- constraints span 66 to 103 s, i.e. neighbouring submaps, and ZERO span the 534 m loop, which
-- needs a gap near 450 s. The big loop is never even attempted. Until the z window covers the
-- drift, no z experiment on this bag can mean anything, because nothing asserts "same place,
-- different height". max_constraint_distance is a 3D distance, so it has to admit the z error too.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 20.
POSE_GRAPH.constraint_builder.max_constraint_distance = 40.
return options
