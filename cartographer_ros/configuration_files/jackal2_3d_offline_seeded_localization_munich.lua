-- OFFLINE dataset extraction only (traj_dataset_carto/cartographer_harness), never on the robot. Munich and Vienna twin of
-- jackal2_3d_offline_seeded_localization_uoft_campus.lua: the robot's Munich live lua (jackal2_3d_live_localization_munich.lua,
-- stack.launch city:=Munich|Vienna) with exactly the same offline-only changes the Toronto offline lua applies on top of the
-- Toronto live lua: the bootstrap search narrowed to the exact seed, the 2 m / 1 m / 10 deg constraint window, and the
-- low offline score floors (0.3 on the steady-state and the initial pose; the Munich live floors 0.45 / 0.40 are above them,
-- as Toronto's live 0.55 is above its offline 0.3; the rotational floor and everything else stay the live values).
-- Seed = the bag's first node in the map (or its gated localization on the frozen map for a bag outside the map), see run_loop.sbatch.
include "jackal2_3d_live_localization_munich.lua"
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 1.
POSE_GRAPH.constraint_builder.initial_pose_linear_z_search_window = 0.5
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(5.)
POSE_GRAPH.constraint_builder.min_score = 0.3
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(10.)
return options
