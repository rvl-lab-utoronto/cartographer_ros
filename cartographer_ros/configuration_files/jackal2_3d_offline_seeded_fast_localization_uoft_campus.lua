-- OFFLINE dataset extraction, fast mode (cartographer_harness --wait lagged): the offline seeded lua plus more background and
-- optimization threads (speed only, same algorithm). The robot's correction delay is reproduced by the harness, not by
-- the thread count: measured once with --wait realtime on the offline seeded lua (the robot's 4 threads).
include "jackal2_3d_offline_seeded_localization_uoft_campus.lua"
MAP_BUILDER.num_background_threads = 16
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 16
return options
