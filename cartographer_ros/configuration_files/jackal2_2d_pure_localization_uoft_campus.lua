-- Pure localization against the U of T campus map (v2_FINAL2: 35 frozen
-- trajectories, 382 submaps, ~40k nodes, 0.1 m, built at max_range 200).
--
-- Tuning ported 2026-09-16 from geodex_jackal_ws
-- slam_jackal/config/jackal2_2d_localize_toronto.lua, which is field measured
-- (Toronto office, 2026-09-07/08/13). Before that port this file was the live
-- SLAM lua with four lines changed, so every CPU relevant knob sat at its SLAM
-- default: measured here 2026-09-16, cartographer_node pinned 6 background
-- threads at ~600% and the pose graph work queue grew to 11k items while a bag
-- played at 1x, draining to 0 only when the bag was paused.
--
-- The single most useful measurement from that other project, because it stops
-- you chasing the wrong thing: turning the GLOBAL (full submap) search off did
-- not remove the CPU load (470% while driving with it off). The cost is the
-- per node LOCAL search against the frozen submaps in range. Everything below
-- that touches sampling, window size or thread counts is aimed at that.
--
-- Inherits the live outdoor lua deliberately: the campus map was built on it
-- (jackal2_2d_mapping_uoft_campus.lua line 20), so frames, topics, submap
-- resolution and the front end all match what produced the submaps. Do NOT
-- inherit from jackal2_2d_mapping_uoft_campus.lua, which is offline only.
include "jackal2_2d_liveslam_toronto.lua"

-- Live trajectory only. The loaded map is frozen and untouched by this.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}

-- ===================================================================
-- The wedge fix. Keep this first, it is not a tuning knob.
-- ===================================================================
-- MUST be nil, and inheriting it is not harmless. OverlappingSubmapsTrimmer2D
-- is registered on the whole POSE GRAPH (pose_graph_2d.cc:60), not per
-- trajectory, and TrimmingHandle::GetOptimizedSubmapData hands it every
-- kFinished submap with a global pose, all 382 loaded ones included, with no
-- frozen check.
--
-- The immediate problem is not that it trims the map, it is that it WEDGES THE
-- NODE. HandleWorkQueue runs every trimmer while holding the pose graph mutex
-- (pose_graph_2d.cc:503-513), and AddSubmapsToSubmapCoverageGrid2D walks every
-- cell of every submap doing two Rigid3d compositions and a std::map insert per
-- cell. Measured on this map: 382 submaps x ~2.35M cells median = ~900M
-- iterations and a red-black tree of order 10 GB, all under the lock.
--
-- The same thing was independently measured on the office map (2026-09-08): the
-- work queue grew ~8000 items/min to 130k in every run whatever else the lua
-- said. It is also the signature of the 2026-09-16 campus replay that scored
-- 148 m: 39k backlog, zero constraints, map->odom stuck at its seed.
--
-- Freshness comes from INTRA_SUBMAP constraints, which frozen loads used to
-- drop; cartographer aa15e4f1 restores them, which is what makes the frozen
-- submaps visible to this trimmer on THIS build. It can trim nothing useful
-- here anyway: frozen submaps are never trimmed and pure_localization_trimmer
-- already caps the live trajectory.
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil

-- ===================================================================
-- Bootstrap: wide search right after the rviz click, then narrow.
-- ===================================================================
-- Fork cartographer 60088547. Armed only by InitializeGlobalSubmapPoses when a
-- trajectory has an initial pose, i.e. exactly the rviz click path. While
-- armed, MaybeAddConstraint skips the per submap sampler and matches EVERY
-- frozen submap within max_constraint_distance on EVERY node using the window
-- below, and the pose graph optimizes after every node so map->odom snaps as
-- soon as the first match lands. Disarms on the first constraint to a frozen
-- trajectory; initial_pose_num_nodes is only the give up cap, after which
-- rosout warns "bootstrap search ended without a constraint" and you re-click.
--
-- 150, NOT the office lua's 10. Measured here 2026-09-16: with a good click
-- (nearest submap origin 1.3 m, 14 submaps inside max_constraint_distance)
-- bootstrap ran its 10 nodes over 13 s and landed ZERO constraints, then the
-- ordinary sampled search immediately scored 0.79..0.88. The budget was not
-- spent matching, it was spent WAITING: the first time any submap is touched
-- its multi-resolution precomputation grid must be built, the match task
-- depends on that build (constraint_builder_2d.cc, AddDependency on
-- creation_task_handle), and these are campus submaps of ~2.35M cells, 14 of
-- them, on 4 threads. The budget has to outlast that one-off build.
--
-- The budget is counted in NODES, and node rate is not fixed: with
-- motion_filter.max_time_seconds = 1 a near stationary robot makes ~1 node/s,
-- so 10 nodes was 13 s, while a robot driving makes 10/s and 10 nodes would be
-- ~1 s. 150 covers both: ~15 s driving, longer parked, and it disarms the
-- instant one constraint to the map lands, so a good click still costs nothing.
-- Re-clicking after the grids are cached latches much faster.
POSE_GRAPH.constraint_builder.initial_pose_num_nodes = 150
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 7.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(45.)
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.55

-- ===================================================================
-- Steady state search cost. This is where the 600% went.
-- ===================================================================
-- 0.3 (SLAM default) -> 0.05. A localized robot needs a fresh constraint every
-- few nodes, not every third one. Each submap in range is now tried every 20th
-- node, a constraint every ~2 s of driving.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.05

-- 7 m / 30 deg -> 4 m / 15 deg. Branch and bound cost scales with window area
-- times angular range, so this is ~1/6 of stock. STEADY STATE ONLY: the click
-- is matched with the wider bootstrap window above, this window only has to
-- cover drift between two sampled constraints.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 4.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)

-- 0.55 -> 0.65. On the office map 92% of local matches are misses (the scan
-- does not overlap that submap) and a miss costs as much as a hit; branch and
-- bound prunes every candidate below min_score, so a higher floor ends misses
-- earlier. A true match below 0.65 is dropped, which delays the next
-- constraint, it does not corrupt the pose.
POSE_GRAPH.constraint_builder.min_score = 0.65

-- The fast correlative matcher derives its angular step from the FARTHEST point
-- of the node cloud (step ~ resolution / r_max), so capping the constraint
-- search cloud bounds the number of angular slices per match. Stock is 50 m: at
-- this map's 0.1 m that is ~131 slices per 15 deg window, against ~31 at 20 m.
-- OUTDOOR DEVIATION, UNVALIDATED: the office lua uses 12 m. Kept at 20 here
-- because campus squares and wide streets need more geometry than a corridor to
-- match uniquely. If matching is unreliable in open areas, RAISE this first.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 20.

-- DO NOT lower max_constraint_distance. It is the node to SUBMAP ORIGIN
-- distance. Measured on this map over 382 origins and 40,246 nodes: nearest
-- origin 2.54 m at p50, 6.89 m at p90, 10.83 m at p99, 18.83 m max, and origins
-- are one num_range_data (100 scans, ~15 m of driving) apart along a single
-- pass. It was tried at 10 here on 2026-09-16 to cut CPU and reverted: the CPU
-- is per match cost and thread oversubscription, not candidate count, so it
-- bought nothing. On the office map 6 m left the robot with NO candidate
-- submaps, no constraints, and rviz showing offset copies of the walls.
-- It also bounds how wrong the rviz click may be, since it tests the submap
-- origin against the ESTIMATED pose.
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- ===================================================================
-- Threads. 8 logical cores here, shared with yolo, rviz, move_base.
-- ===================================================================
-- 6 -> 4. Caps the constraint search whatever the load, so the queue absorbs
-- bursts instead of the front end and the planner losing cores. This is
-- literally the 600%: 600 = 6 threads saturated.
MAP_BUILDER.num_background_threads = 4

-- The live lua gives every ceres solve 4 threads. Per constraint refinement and
-- the front end scan match are tiny problems (a few hundred residuals) where
-- worker threads cost more in spin and sync than they save, and 6 background
-- threads each spawning 4 solver threads is 24 runnable threads on 8 cores.
-- Upstream default for both is 1. The pose graph optimization keeps its 4:
-- bigger problem, and only one runs at a time.
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- ===================================================================
-- Front end insertion.
-- ===================================================================
-- No free space insertion in the LIVE submaps. Nothing consumes it in
-- localization: the front end ceres match and the constraint search only read
-- hit cells (unknown and free score the same). Stock true makes CastRays walk
-- origin->hit for EVERY return into BOTH active submaps, cost ~ sum(ray length)
-- / resolution. Outdoors at max_range 200 on a 0.1 m grid that is the dominant
-- per scan cost, and when a scan exceeds the 100 ms budget the single threaded
-- cartographer_ros spinner holds the node mutex, so /tracked_pose, map->odom
-- and the publish timer all fall behind, robot_tracker mutes and the MPC gate
-- closes. Insertion becomes 2 x N hit updates.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = false

-- A parked robot makes a node only every max_time_seconds, and the snap after a
-- click needs nodes (constraint attempts) plus optimize_every_n_nodes of them
-- for the optimization that applies it. At stock 5 s that is ~50 s parked. At
-- 1 s it snaps within ~10 s. Costs nothing while driving, where the distance
-- and angle limits fire first.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 1.

-- ===================================================================
-- Pose graph cadence and the global search.
-- ===================================================================
-- Not about solve cost (frozen constraints are dropped by Ceres). With 1 the
-- work queue stops after EVERY node to wait for that node's matches; with 10 it
-- keeps draining sensor items while matches run in the pool. map->odom then
-- lands every 10 nodes (~1 s of driving).
POSE_GRAPH.optimize_every_n_nodes = 10

-- Global (full submap) search effectively off. The click connects the live
-- trajectory to all frozen ones (fork ConnectAlsoToAllFrozen) and the local
-- search keeps it connected, so this only fires when localization is already
-- lost. Then it pulses global_sampling_ratio per (node, submap) pair over ALL
-- 382 submaps with no distance gate, each a MatchFullSubmap: measured ~11
-- whole submap searches per second at 10 Hz, each also building a ~16 MB
-- precomputation grid that is never freed.
-- LOST: automatic re-localization after a kidnap, and recovery after driving
-- off the map. Re-click instead. Re-enable moderately (60 s, 0.001) once the
-- CPU is confirmed low.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e6
POSE_GRAPH.global_sampling_ratio = 0.0001

return options
