-- 3D pure localization against the merged campus pbstream (write_merged_pbstream.py), loaded with
-- load_frozen_state. Mapping-side settings (local SLAM, grids, matcher options) come from the map's
-- own batch config so live scans are built exactly like the map's submaps; the localization-only
-- choices below mirror jackal2_2d_pure_localization_uoft_campus.lua, translated to 3D.
-- VERIFIED 2026-09-30 (user) on a full replay of wild_uoft_2026-09-22-10-30-01 (655 s, climbs to z 2.2 m); first used 2026-09-28 by the held-out-bag offline test.
include "exp3d_batch.lua"

-- Live trajectory trimmer. keep_uncovered and its radii exactly as jackal2_2d_pure_localization_uoft_campus.lua
-- (see there): on the stored map the live trajectory stays capped, off it the live submaps survive and the run
-- maps like live SLAM. The coverage test is xy only (pose_graph_trimmer.cc), so it is 3D-safe. 3 kept, not 2D's
-- 5: a 3D submap here is 160 scans against 2D's 40. Marked VERIFIED 2026-09-30 (user)
-- with the replay above; that bag stays on the stored map, so the off-map retention path itself did not run.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
  keep_uncovered = true,
  coverage_resolution = 1.,
  coverage_radius = 12.,
  keep_radius = 3.,
}

-- Bootstrap after the rviz click, ported to 3D 2026-09-30 (cartographer fork, PoseGraph3D / ConstraintBuilder3D,
-- same mechanism as 2D fork 60088547; values as the 2D loc lua, see its rationale). Until the first constraint
-- to the frozen map: every frozen submap within max_constraint_distance on every node, no sampling, this wider
-- window, optimize after every node, so map->odom snaps. Then the cheap steady state below applies. Without it
-- the snap depended on the steady-state sampling, which was cut for CPU.
-- z window 2 m: the start z comes from the map's own nodes at the click (start_pose_uoft_campus_3d.yaml).
POSE_GRAPH.constraint_builder.initial_pose_num_nodes = 150
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 7.
POSE_GRAPH.constraint_builder.initial_pose_linear_z_search_window = 2.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(45.)
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.55
-- constraints to the frozen map: local search only, never a global search once localized
-- Cadence = jackal2_2d_pure_localization_uoft_campus.lua (10 nodes, 0.05; see its rationale: map->odom lands
-- every 10 nodes, each submap in range tried every 20th node). 2026-09-30: 30 / 0.03 was tried for CPU and
-- REVERTED the same day. It was judged offline on the FINAL optimized trajectory (xy 2.8 cm), which hides the
-- real-time lag; live it gave 1-2 constraints per optimization (~1 per 3 s) and the live pose drifted off the
-- map by up to 0.19 m / 1.9 deg between corrections. The CPU problem was the inherited offline thread counts
-- (fixed below), not the cadence.
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e11
POSE_GRAPH.global_sampling_ratio = 0.0001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 4.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.)
-- 0.55 = the bootstrap floor (initial_pose_min_score). 2026-09-30 live replay: at 0.6, from node 1 to 240 (~55 s,
-- ~30 m driven up the slope north of the start) ~150 attempts gave 1 constraint and the pose drifted 1.0 m; the
-- bootstrap's matches in the same area scored 0.57-0.60 and were confirmed correct (the next match agreed to
-- 0.11 m), and the first matches after the gap scored 0.62-0.63. VERIFIED 2026-09-30 (user):
-- the same stretch at 0.55 gave 21 constraints, worst offset 0.15 m; whole bag 847 constraints, no droughts.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
-- Threads and logging, 2026-09-30. exp3d_batch.lua -> jackal2_3d_mapping_uoft_campus.lua sizes these for an
-- OFFLINE run that owns the machine: 8 background threads and 8 Ceres threads on EVERY front-end scan match,
-- plus per-iteration Ceres output. Live, the node shares 8 cores with yolo, rviz and the planner. Same choices
-- and rationale as jackal2_2d_pure_localization_uoft_campus.lua: 4 background threads cap the constraint
-- search (the queue absorbs bursts), the front-end match is a small problem where worker threads cost more in
-- spin and sync than they save (upstream default 1), the pose graph solve keeps 4 (one at a time).
MAP_BUILDER.num_background_threads = 4
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 1
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.optimization_problem.ceres_solver_options.minimizer_progress_to_stdout = false
POSE_GRAPH.optimization_problem.log_solver_summary = false
return options
