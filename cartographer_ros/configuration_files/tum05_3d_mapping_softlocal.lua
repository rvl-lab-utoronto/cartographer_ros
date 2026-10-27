-- Munich tum05 3D mapping with the Toronto "soft local SLAM" pose-graph weights (2026-10-26): for batches whose straight, feature-poor streets
-- come out shortened / rotated (along-track slip of local SLAM). Same as tum05_3d_mapping.lua except
--   local_slam_pose_translation_weight 1e5 -> 1e4, huber_scale 1e1 -> 5e3, ceres max_num_iterations -> 100
-- (the measured fix of the 2026-09-16 Toronto campus diagnosis, memory campus-map-weights-and-overlap; local 1e3 is too free).
include "tum05_3d_mapping.lua"
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e4
POSE_GRAPH.optimization_problem.huber_scale = 5e3
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100
return options
