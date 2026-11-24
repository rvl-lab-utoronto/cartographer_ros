-- DIAGNOSTIC ONLY (2026-10-07): the offline seeded lua with every Cartographer background and optimization thread count at
-- 192 (a whole Trillium node), to measure how far more threads speed up the background work. Not for production runs.
include "jackal2_3d_offline_seeded_localization_uoft_campus.lua"
MAP_BUILDER.num_background_threads = 192
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 192
return options
