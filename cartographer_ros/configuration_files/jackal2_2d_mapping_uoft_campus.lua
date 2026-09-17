-- Campus scale multi bag OFFLINE mapping (2026-09-09). UNVALIDATED: written for
-- the first joint cartographer_offline_node run over the carto_input bags
-- listed in slam_jackal/maps/uoft_campus_2026-09/bag_order.txt (one trajectory
-- per bag, none frozen, one final optimization).
--
-- Everything sensor and matcher related is INHERITED from the live outdoor lua
-- (jackal2_2d_liveslam_toronto.lua: max_range 200 m, 0.1 m submaps, voxel 0.1,
-- adaptive filter 100 pts / 1.0 m, hit 0.55 / miss 0.48, global search 15 s
-- after a trajectory is unconnected at the default 0.003 sampling), so the
-- map is built from exactly what live SLAM sees on campus. Only what a joint
-- offline run needs differs:
--  * both trimmers OFF: the live lua keeps 5 submaps for CPU; a pbstream
--    written with trimmers is not a map.
--  * optimize_every_n_nodes 90 (live: 1): the offline node runs a final
--    optimization anyway, and per node optimization over a ~50 trajectory
--    graph would dominate the wall time. ~90 nodes is one submap, enough for
--    constraint proposals along the way. Global search stays ON throughout:
--    every new bag must find its anchor in the earlier bags, and cross day
--    loop closures are the point of the joint run.
include "jackal2_2d_liveslam_toronto.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = nil
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil
POSE_GRAPH.optimize_every_n_nodes = 90

-- Threads (2026-09-11): offline only, sized to sep-ws (12 logical CPUs). The
-- live lua keeps 6/4/4/4 for the robot.
--  * num_background_threads: the constraint-search pool, the main win here.
--  * optimization_problem stays at 4: measured on the c01 trim pass
--    (2026-09-11), 12 Ceres threads cost +3.7 GB peak RSS (2.0 -> 5.8 GB)
--    for no speedup (32 s vs 36 s). Memory is the binding constraint here.
--  * constraint_builder ceres_scan_matcher stays at 1: each refinement runs
--    INSIDE a background thread, so 12 x 12 would oversubscribe to 144.
--  * local ceres_scan_matcher runs alone on the main thread: safe at 12.
MAP_BUILDER.num_background_threads = 12
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 12
-- Pose-graph optimization weights (2026-09-16). Measured on the 3-bag expand-chain state
-- (v2_e05: bags 08-28 14:35, 08-28 18:09, 08-25 18:27, 1115 loop closures): with the cartographer
-- defaults the closures were found but NOT applied. Local SLAM slips 0.7-0.9 m ALONG-TRACK on
-- straight, feature-poor stretches (same spot, same amount, on two different days: the wheel
-- odometry scale error feeding the unobservable direction), the closures there said so
-- ("differs by translation 0.8 m", score 57-62 %), and after optimization their residual was
-- still 0.75-0.92 m: local_slam_pose weight 1e5 vs loop closure 1.1e4, and huber_scale 1e1 in
-- WEIGHTED units caps every closure with > 1 mm error to a constant pull. Result: double walls.
-- Re-optimizing the same graph (trim pass, trimmer off, nothing re-mapped):
--   defaults            residual median 0.093 m  p90 0.374  max 0.92
--   100 iterations only               0.093       0.374      0.92   (only rigid flex)
--   local 1e4                         0.043       0.231      0.85
--   local 1e4 + huber 5e3             0.010       0.033      0.15   <- chosen
--   local 1e3 + huber 5e3             0.001       0.004      0.06   (too free: nothing resists a false match)
-- All 1115 closures of three bags satisfied to 15 cm at once is not what false matches do; the
-- map reshaped smoothly (2.2 m at the far end of bag 1, growing monotonically from the origin).
-- Rotation weight stays 1e5 (IMU heading is good, the slips were pure translation). Guard
-- (pbstream_displacement_guard.py) remains the defense against a bag pulled by a false closure.
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e4    -- cartographer default 1e5
POSE_GRAPH.optimization_problem.huber_scale = 5e3                           -- default 1e1; quadratic up to ~0.45 m at weight 1.1e4
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 100  -- live lua sets 10; offline can afford convergence
-- Loop-closure search (2026-09-16). Same 3-bag state: bag 2 (08-28 18:09, same route as bag 1)
-- had found only 18 cross-bag closures, in tight pairs five minutes apart. Cross-trajectory LOCAL
-- search only runs while the pair was connected within global_constraint_search_after_n_seconds
-- by this trajectory's own nodes (pose_graph_2d.cc ComputeConstraint); after that only the global
-- sampler can re-find the map. Re-mapping that bag onto bag 1 (frozen), cross-bag closures found:
--   15 s window, sampling 0.3, global 0.003 (defaults)     18   15 min
--   1e9 window, sampling 1.0, global 0.01                    1   the trajectory left the 7 m local window
--                                                              at node 125 and, never re-globalized, was lost
--   60 s window, sampling 1.0, global 0.03                4908   54 min, 8.6 GB peak, residual median 0.035 m  <- chosen
-- The expiry is the re-localization safety net: keep it, long enough to hold a lock across short
-- match-free stretches, and re-globalize often when lost. sampling_ratio 0.5 roughly halves the
-- wall time if ~5000 closures per bag is more than needed. chunks/seed_from_cache.py can start a
-- bag in place (-initial_trajectory_poses) instead of relying on the global sampler.
POSE_GRAPH.global_constraint_search_after_n_seconds = 60   -- live lua sets 15
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0         -- cartographer default 0.3
POSE_GRAPH.global_sampling_ratio = 0.03                    -- cartographer default 0.003
-- Local SLAM / sensor settings for OFFLINE mapping (2026-09-17). The live lua's values are tuned
-- for real-time on the robot; offline we can afford denser matching and more submaps. Measured by
-- mapping bag 1 (08-28 14:35) alone under each variant and reading the PRE-optimization drift at
-- revisit straight out of the log ("differs by translation X" = how far the scan match had to move
-- local SLAM's estimate). That is not circular: no optimizer has touched it yet. Median / p90 / max
-- drift in m and median match score, all at 94 % of the bag:
--   live settings (200 m, voxel 0.1, adaptive 1.0/100, 100-scan submaps)  0.200 0.590 0.96  59.5 %
--   + range 60 m, adaptive 0.5/200                                        0.180 0.700 1.29  62.3 %
--   + range 60 m, voxel 0.05, adaptive 0.25/500                           0.180 0.673 1.63  63.4 %
--   + 50-scan submaps, adaptive 0.5/200                                   0.140 0.420 1.10  65.2 %   <- chosen
--   + 50-scan submaps, voxel 0.05, adaptive 0.25/500                      0.150 0.420 1.13  67.2 %
--   + 50-scan submaps, no wheel odometry                                  0.050 0.150 2.10  66.6 %
-- Submap size is the lever: a 100-scan submap spans ~10 s of driving, and local SLAM's own drift is
-- baked into it as blur before the pose graph ever sees it. The aggressive voxel/adaptive values
-- bought nothing over cartographer's defaults, so the defaults are used. Dropping wheel odometry
-- cuts median drift 3x but the tail (p99 1.0-2.2 m, max 2.1-2.9 m) is the featureless-straight
-- failure mode this map already suffers from, so odometry stays on.
-- NOT validated: max_range 60 came along as part of every tested variant; the 200 m ablation
-- (exp_ls_dense_s50no_corr_r200.lua) was never run. 60 m is far past the ~15 m median and ~30 m p90
-- at which occupied cells actually sit, and it removes the long grazing returns that draw the
-- starbursts. Also untested: online correlative scan matching, 30-scan submaps.
TRAJECTORY_BUILDER_2D.max_range = 60.                                 -- live 200 m (cartographer default 30)
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 60.           -- live 200
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5          -- live 1.0, cartographer default 0.5
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200      -- live 100, cartographer default 200
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 60.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.5   -- cartographer default 0.9
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 50                     -- live 100; the main win
-- 50-scan submaps double the submap count, so the constraint sampler is halved to keep the search
-- cost per bag roughly where it was measured (bag 2: 4091 cross-bag closures, 55 min, at 1.0).
POSE_GRAPH.constraint_builder.sampling_ratio = 0.15   -- 2026-09-17 01:10: cut from 0.5 for wall-clock. Bag 2 found 4091 cross-bag closures at the old effective rate and the guard needs 3; ~0.3x the pairs still leaves >1000 per bag. Constraint search dominates the stage time and grows with the loaded map (v2_f: 55 min at 31 submaps loaded, 86 min at 65).
return options
