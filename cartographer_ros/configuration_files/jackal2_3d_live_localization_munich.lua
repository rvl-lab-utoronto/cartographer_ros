-- LIVE 3D pure localization in MUNICH (stack.launch city:=Munich localization:=localization3d) against the final Munich map
-- (~/maps/munich_tum_final_v4/munich_tum_final_v4_loctrim_ultralight.pbstream, slam_jackal/maps/munich_2026/FINAL_MAP.md).
-- = the Toronto live lua (unchanged, trimmer and all) + the settings below, the winner of the 2026-11-02 sweep
-- (slam_jackal/maps/munich_2026/final/sweep/: 26 settings x 15 Munich bags x {bag start, late cold start}, scored on the pose the
-- robot publishes, through traj_dataset_carto's lockstep harness in Toronto's emergent-latency mode). Pick rule: pass every bag from
-- its start, then the fewest cold-start failures, then false jumps; not the best median. None of it is a hardware difference: the
-- 2025 and 2026 Munich bags have Toronto's IMU rate, frames and tf_static.

include "jackal2_3d_live_localization_uoft_campus.lua"

-- 1. Score floors 0.45 (Toronto 0.55; cartographer default low-resolution 0.55, rotational 0.77). Correct matches on the Munich map
--    score 0.40-0.78, median ~0.50 (the map frame sits ~3 deg off gravity, Toronto's ~0.9 deg, so every gravity-aligned live scan
--    matches worse): Toronto's live lua (0.55) failed 6 of 15 bags from their start. Sweep 0.35-0.50: 0.45 and 0.50 pass every bag from its start;
--    0.35 adds false jumps. The rotational floor is not binding (correct matches score ~0.95).
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_low_resolution_score = 0.45
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.60

-- 2. Start-up (bootstrap) floor 0.40, below the steady-state floor: the first match after the rviz click is the hardest (seed error up
--    to ~0.5 m / 3 deg). Sweep 0.40 / 0.50 / 0.60: 0.40 best; 0.50 and 0.60 failed 1-3 bags from their start or raised the worst p90 to 1.2 m.
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.40

-- 3. Pose-graph IMU acceleration term back on, at 1e3 (Toronto's live lua 0, upstream 1.1e2). With both IMU terms off (Toronto's fix for
--    its odom_2d gyro-bias problem) nothing holds gravity between the sparser Munich constraints: z drifted 3 m and
--    analyze_wild_2025-10-14-16-04-48 lost the map after ~165 s. Bisection: either term alone fixes that; the acceleration term keeps the
--    rotation term off (Toronto's fix). Sweep 0 / 1e1 / 1.1e2 / 1e3: 1e3 had the fewest cold-start failures (10 of 15 vs 12), same
--    bag-start results; rotation_weight 1.6e4 (Toronto's eval value) was worse (worst p90 3.1-4.9 m).
POSE_GRAPH.optimization_problem.acceleration_weight = 1e3

-- 4. Optimize every 20 nodes (Toronto's live lua 10). Sweep 5 / 10 / 20: 20 cut the worst p90 from the bag start from 1.5 m to 0.16 m
--    (the anchor bag's drought at 680-760 s) at the same CPU. global_sampling_ratio stays 1e-4 (1e-5 and 1e-3: no gain, one more bag failed).
POSE_GRAPH.optimize_every_n_nodes = 20
return options
