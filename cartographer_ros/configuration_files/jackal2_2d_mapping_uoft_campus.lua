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

-- LOCAL SLAM, tuned 2026-09-17 the way the cartographer tuning guide says: global SLAM OFF
-- (optimize_every_n_nodes = 0), one bag alone, one knob at a time, judged by slippage = the
-- PRE-optimization "differs by translation" at REVISITS only (node-to-submap time gap > 60 s;
-- matches against the submap being built are not revisits). Bag 12 (08-21 16:41) is the only one
-- of the 13 with a real loop: 562 m of driving between submap and revisit. Bag 1 has NO loops.
-- Big-loop slippage on bag 12, everything else at the original-map values (200 m, voxel 0.1):
--   translation_weight 8 (live)   9.4 m   = 1.7 % of distance; the matcher is pulled toward the
--                                          wheel-odometry prediction, whose scale is off
--   translation_weight 4 / 2 / 1  3.6 / 1.8 / 1.5 m
--   translation_weight 0.5        1.17 m  (0.4 -> 1.17, 0.3 -> 1.32, 0.6 -> 1.38: flat bottom)
--   odometry OFF                  2.0 m, but short (60-200 s) revisits 0.15 -> 3.3 m: keep it
--   + rotation_weight 1           0.92 m  (40 -> 1.8 m; IMU yaw + scan beats a stiff prior)
--   + submaps 40 / 50             0.59 / 0.54 m  (30 -> 3.25, 60 -> 1.22, 70 -> 1.26, 100 -> 1.17;
--                                          a cliff above 50 with no mechanism found: stay in 40-50)
--   + submaps 40 + rotation 1     0.40 m  = 0.07 % of distance          <- chosen
-- Short revisits stayed 0.17-0.22 m in every variant. No gain from: online correlative matching
-- (1.21 m, 2x time), dense voxel/adaptive filters (2.0 m), range 60 (1.18 = 200 m), tighter
-- motion filter (2.3 m). Confirmed on bag 5 (the only other bag with revisits, 60-200 s):
-- baseline 0.08 m median / 0.22 max, 134 matches, score 61.6 -> 0.02 / 0.05, 345 matches, 65.1.
-- Sweep tooling and every run: chunks/tune_local_parallel.sh, chunks/tune_local_bag12*/.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5   -- live 8
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.       -- live 5
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40                    -- live 100

-- Ceres per-iteration progress, POSE GRAPH ONLY (2026-09-18, user). The joint optimization runs
-- silent for ten to twenty minutes on a graph this size; this prints cost, gradient and step per
-- iteration so it can be watched. Requires the local cartographer patch that exposes the field
-- (submodule src/cartographer: ceres_solver_options.proto plus .cc, fields 4 and 5).
-- Deliberately NOT set on TRAJECTORY_BUILDER_2D.ceres_scan_matcher: that solves once per
-- accumulated scan, so it would emit hundreds of thousands of lines per bag and slow the run on
-- stdout alone. The pose graph optimizer fires only on the periodic and final optimizations.
POSE_GRAPH.optimization_problem.ceres_solver_options.minimizer_progress_to_stdout = true

return options
