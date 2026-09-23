-- 3D LOCAL sweep variant 'free_z' on the LOOP bag (2026-09-21). The one variant that is not a
-- local knob at all: it frees z in the pose graph, which is also what switches ON the IMU
-- residual blocks and use_online_imu_extrinsics_in_3d (optimization_problem_3d.cc:354 vs the
-- fix_z branch at 279/456). Run on the loop bag because this is the ONLY bag that returns to a
-- place it left 534 m earlier: on an out-and-back bag no constraint ever asserts "same place,
-- different height", so freeing z changes nothing (measured on 2026-08-28-18-22-45: 10.00 m of
-- z drift free vs 10.25 m fixed). Global SLAM stays OFF as the local protocol requires; the final
-- optimization is where the loop constraint gets to act.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
return options
