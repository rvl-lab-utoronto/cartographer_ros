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
--
-- keep_uncovered (fork 2026-09-21) is what makes this both things at once. The
-- stock trimmer drops every live submap except the last max_submaps_to_keep,
-- wherever it is, so driving off the stored map builds territory and then
-- throws it away. With this set, a live submap is spared when the nodes
-- inserted into it reach ground the FROZEN trajectories never drove, so:
--   on the stored map  -> nothing is spared, the live trajectory stays capped
--                         at 5 submaps and the CPU profile below is unchanged;
--   off the stored map -> the submaps survive and the run maps like it did
--                         before there was a map.
-- Coverage is the frozen nodes' positions dilated by coverage_radius, built
-- ONCE on the first trim (one pass over the loaded nodes), then a lookup.
-- Spared live submaps are folded into it, so a second pass over the same new
-- ground is trimmed rather than stacking another copy of it.
--
-- coverage_radius 12 m: how far off the old path still counts as mapped. It is
-- NOT the sensor range. Too large and a genuine excursion is mistaken for
-- covered ground and discarded; too small and driving a parallel sidewalk
-- re-maps ground that is already good. UNVALIDATED, 12 is a first guess from
-- the campus corridors being roughly 20 m wide; measure before trusting it.
--
-- coverage_resolution 1 m: the question is "has the robot been here", not
-- where a wall is, so the bitmap is coarse on purpose. At 1 m the campus map
-- is ~400 kB.
--
-- The cost of NOT trimming off-map submaps is that the live graph grows while
-- off the map, exactly as it did in the pre-map days. That is the intent, but
-- it is unbounded: a long excursion eventually costs what live SLAM costs.
-- keep_uncovered is OFF pending a rewrite, 2026-09-21. As first written,
-- PureLocalizationTrimmer::IsRedundant runs inside Trim, which cartographer
-- calls WHILE HOLDING THE POSE GRAPH MUTEX, and it is far too expensive to be
-- there: the first call stamps a disc of ~450 cells for each of ~80k frozen
-- nodes (~36M operations), and every later call iterates the whole 307k-entry
-- constraint list TWICE per candidate submap. That is the same shape as the
-- wedge this file already documents for OverlappingSubmapsTrimmer2D, and it
-- would starve the constraint builder exactly when the bootstrap needs it.
-- Not yet proven to be the cause of anything observed; turned off so that the
-- map change (v2_FINAL2 -> v2_k22_trim) can be tested on its own. Turn it back
-- on only after the membership lookup is precomputed and the coverage build is
-- off the critical path.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
  keep_uncovered = false,
  coverage_resolution = 1.,
  coverage_radius = 12.,
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
-- Local SLAM: the offline-tuned front end the map was built with.
-- ===================================================================
-- These three are the result of the tuning study in
-- chunks/TUNING_README.md and are set by jackal2_2d_mapping_uoft_campus.lua
-- (lines 60-62), which is offline-only and therefore must not be inherited
-- here. Without repeating them, localization silently ran the PRE-tuning live
-- values (8 / 5 / 100), because this file includes the live lua directly.
-- Added 2026-09-21 after that was noticed. OFFLINE VERIFIED, ONLINE
-- UNVALIDATED: the numbers below are the offline bag-12 measurement, never
-- run live.
--
-- Measured on bag 12 (08-21 16:41, the only bag with a 562 m loop), drift at
-- revisit before any optimization:
--   translation_weight 8 (live)      9.4 m   = 1.7 % of distance driven
--   translation_weight 0.5           1.17 m  (flat bottom: 0.4 -> 1.17, 0.6 -> 1.38)
--   + rotation_weight 1              0.92 m
--   + submaps.num_range_data 40      0.40 m  = 0.07 %
-- The mechanism is this robot's ~14 % wheel-odometry scale error: the stock
-- translation_weight pulls the scan matcher toward that bad prediction. In
-- MAPPING that showed up as loop error. In LOCALIZATION the same pull fights
-- the frozen map between constraints, so the pose drifts further than it needs
-- to and each correction arrives as a larger visible jump.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5   -- live 8
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight    = 1.    -- live 5

-- 40, matching the map. The frozen submaps in this map ARE 40-scan submaps, so
-- this also makes the live submaps the same size as the ones they are matched
-- against. Note it invalidates the "one num_range_data (100 scans, ~15 m)"
-- figure quoted in the max_constraint_distance comment below: at 40 the origins
-- of the FROZEN submaps are ~6 m of driving apart, which is consistent with the
-- measured p50 nearest-origin distance of 2.54 m rather than at odds with it.
-- It also means live submaps finish sooner, so the trimmer fires more often.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40                   -- live 100

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
-- 2026-09-21: WAS 1e6, and that is why the initial pose was never corrected.
-- This value does NOT disable the global search. Read pose_graph_2d.cc
-- ComputeConstraint: the LOCAL search is used when
--     same trajectory  OR  node_time < last_connection_time + THIS
-- and only otherwise does it fall through to the global sampler. For a freshly
-- started live trajectory, last_connection_time against every frozen
-- trajectory is a default-constructed common::Time (year 1), because
-- TrajectoryConnectivityState::LastConnectionTime is a std::map lookup on a
-- missing key. Node time is ~6.4e17 ticks (2026); 1e6 s is 1e13 ticks. The
-- comparison is false for every frozen submap, forever.
--
-- So 1e6 made the LOCAL branch unreachable, not the global one. The bootstrap
-- above only bypasses the per-submap sampler INSIDE that local branch, so it
-- could never fire either. The sole remaining path was the global sampler at
-- global_sampling_ratio 0.0001, i.e. effectively nothing, so no constraint was
-- ever formed, ConnectAlsoToAllFrozen was never reached, and the map->odom
-- stayed at whatever the rviz click said while the live submaps drifted off by
-- exactly that error. Measured 2026-09-21: 199 live nodes, 8 frozen submaps
-- within max_constraint_distance of the robot, and "0 computations resulted in
-- 0 additional constraints" for the whole run.
--
-- 1e11 seconds is ~1e18 ticks: larger than any node time, so the local branch
-- is always taken, which is what this file wanted all along ("the local search
-- keeps it connected"). Still comfortably inside int64. The global search is
-- disabled by global_sampling_ratio below, which is the knob that actually
-- does that.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e11
POSE_GRAPH.global_sampling_ratio = 0.0001

return options
