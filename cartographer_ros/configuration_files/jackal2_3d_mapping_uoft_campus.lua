-- 3D OFFLINE mapping of the U of T campus from the SAME carto_input bags as the 2D map
-- (2026-09-21, plans/carto-3d/plan.md). UNVALIDATED: step 0 of the plan, nothing tuned yet.
--
-- Input is the masker's 3D cloud (/filtered_point_cloud2: verticals plus coherent low structure
-- under 0.6 m, humans removed, NO ground), IMU at 100 Hz in os_imu, wheel odom at 50 Hz.
--
-- TRAJECTORY_BUILDER_3D starts from the UPSTREAM defaults (trajectory_builder_3d.lua restored to
-- cartographer 877157a0 on 2026-09-21; the TUM edits to that file are gone). Only what the sensor
-- dictates is set here. Everything else is a knob for chunks/tune_local_parallel_3d.sh, tuned the
-- way the cartographer guide says: global SLAM off, one bag with a real loop, one knob at a time.
--
-- POSE_GRAPH: upstream defaults plus the Toronto pose-graph choices that carry over on principle
-- (odometry weights 0: the 14 percent wheel-odom scale error belongs in the local
-- prior, not in the graph). Trimmers off, optimize_every_n_nodes 90: same reasons as the 2D
-- campus lua.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "os_imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,   -- 3D: keep the full pose in what the node publishes
  use_pose_extrapolator = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  publish_tracked_pose = true,
  publish_tracked_pose_in_odom = true,
  publish_to_tf = true,
  publish_odom_to_published_frame = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true

TRAJECTORY_BUILDER.pure_localization_trimmer = nil
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil
TRAJECTORY_BUILDER.collate_fixed_frame = false
POSE_GRAPH.optimize_every_n_nodes = 90

-- Threads, sized to sepehr-legion (8 logical CPUs). Same split rationale as the 2D campus lua:
-- constraint search pool gets the cores, each constraint refinement runs INSIDE one of those
-- threads so stays at 1, the local matcher runs alone on the main thread, pose-graph Ceres at 4
-- (more threads cost RSS for no speedup, measured 2026-09-11).
MAP_BUILDER.num_background_threads = 8
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 1
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 8

-- Sensor-dictated (not tuning): the Ouster's usable range and the masker's near cut.
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 200.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 200.
-- The cloud's intensity is the Ouster 16-bit signal; the upstream intensity cost and its
-- threshold of 40 were written for a different scale. Geometry only.
TRAJECTORY_BUILDER_3D.use_intensities = false
-- Constant-velocity extrapolator (upstream default). The imu_based one produced badly wrong
-- orientation in the 2026-08-28 live try-out and weights planar odom against 3D IMU; not swept.
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = false

-- Toronto pose-graph choice carried over from the 2D lua on principle: the 14 % wheel-odometry
-- scale error belongs in the local prior, not in the graph.
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.

-- ============================================================================
-- TUNED 2026-09-21 on wild_uoft_2026-08-21-16-41-36 (King's College Circle), the only bag on
-- campus that revisits a place after a long drive: the ORIGINAL 2D map puts two of its moments
-- 0.129 m apart with 528.8 m of driving between them. Metric = the distance a 3D run puts between
-- the nodes at those two timestamps, which is the same definition the 2D tuning used, so these
-- numbers compare directly with the 2D README's 9.4 m baseline and 0.40 m tuned.
-- Full narrative and every run: maps/uoft_campus_2026-09/chunks/TUNING_README_3D.md
--
--   upstream defaults                    4.52 m   loop NOT closed
--   translation_weight 2                 1.39 m   loop NOT closed
--   translation_weight 1                 1.10 m   loop NOT closed
--   translation_weight 0.5               3.08 m   loop NOT closed
--   z window 20 m alone                  4.55 m   loop NOT closed
--   z window 20 m + fix_z false          6.18 m   loop NOT closed
--   + translation_weight 2 + xy window   0.51 m   LOOP CLOSED (gap 474 s)
--   + translation_weight 1               0.35 m   LOOP CLOSED (gap 475 s)   <- chosen
--   + translation_weight 0.5             3.51 m   loop NOT closed
--   + translation_weight 1, rotation 4   7.00 m   loop NOT closed
-- ============================================================================

-- LOCAL. The 3D translation_weight is a SINGLE ISOTROPIC scalar over x, y and z
-- (translation_delta_cost_functor_3d.h), so it cannot be loosened for x and y while being
-- tightened for z the way 2D loosened it to 0.5. Lower weight monotonically reduces z drift
-- (43.1 / 37.2 / 15.3 / 10.2 m at 5 / 2 / 1 / 0.5) because the matcher is pulled less by the
-- tilted odometry prior, but big-loop xy has a clear minimum at 1 and degrades on both sides.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1.   -- upstream 5.
-- rotation_weight stays at the upstream 4e2. Loosening it to 4 gave the best SHORT-revisit
-- slippage on a short bag (0.10 m) and was therefore tempting, but on the loop bag it cost
-- 7.00 m of big-loop error and lost loop closure entirely. The short-bag result did not transfer.

-- LOOP CLOSURE PRECONDITIONS. Not taste: sized to measured residual error. With the upstream
-- windows the 534 m loop was NEVER found in any run; every constraint topped out near a 100 s
-- node-to-submap gap, i.e. neighbouring submaps only, while the loop needs about 440 s. Local
-- SLAM arrives back metres out in BOTH xy and z (z because the cloud is 0.5 % horizontal surface
-- and cannot observe it), and the default 5 m xy / 1 m z windows are smaller than that error.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 10.  -- upstream 5.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 20.   -- upstream 1.
POSE_GRAPH.constraint_builder.max_constraint_distance = 40.   -- upstream 15.; a 3D distance, so it must absorb the z error too

-- fix_z_in_3d FALSE, and this is a correction of an inherited setting rather than a tuning choice.
-- true was carried over from jackal2_2d_liveslam_toronto.lua, where it is right because that stack
-- publishes planar poses. In a real 3D map it does three harmful things at once
-- (optimization_problem_3d.cc): a SubsetParameterization freezes z at line 279 so the solver
-- cannot repair accumulated z error; the IMU acceleration and rotation residuals are added ONLY
-- in the !fix_z branch at line 354, so nothing anchors gravity globally; and
-- use_online_imu_extrinsics_in_3d, the library's own remedy for the measured ~1.2 deg IMU to
-- LiDAR pitch disagreement, is bypassed with them. Unable to move z, the solver expressed the
-- needed correction as ROTATION: optimized poses swung 35.6 deg of roll on the loop bag while
-- cartographer's own stored per-node gravity_alignment for the same run spanned only 8.2 deg.
-- With it false that swing is 6.1 deg and z drift is 4.69 m instead of 43.1 m.
POSE_GRAPH.optimization_problem.fix_z_in_3d = false

-- Ceres per-iteration progress, pose graph only (local cartographer patch, same as the 2D lua).
POSE_GRAPH.optimization_problem.ceres_solver_options.minimizer_progress_to_stdout = true

return options
