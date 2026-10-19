-- OFFLINE dataset extraction only (traj_dataset_carto/cartographer_harness), never on the robot.
-- The robot's live localization lua, unchanged, except the bootstrap search around the initial pose: offline the trajectory
-- is seeded with the bag's own pose from the full map's optimized trajectory (exact to centimetres), so the live 7 m / 45 deg
-- window (sized for an rviz click) only lets wrong-heading matches against the frozen map rotate the new trajectory
-- (2026-10-07 dev bag: 21-26 deg while parked) and costs ~180 ms per scan for the first 150 scans.
-- Also offline-only (see below): the 0.3 score floors and the 2 m / 10 deg constraint window. The pose-graph IMU fixes of
-- 2026-10-13 (online IMU extrinsics off, IMU rotation/acceleration residuals off) live in the included live lua since
-- 2026-10-19 (user decision), so offline and live share them.
include "jackal2_3d_live_localization_uoft_campus.lua"
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 1.
POSE_GRAPH.constraint_builder.initial_pose_linear_z_search_window = 0.5
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(5.)
-- Score floors: the live 0.55 rejected every match on the 2026-10-07 dev bag (1896 attempts, 0 constraints; trajectory
-- unanchored, heading drifted 23 deg); at a 0.2 floor all 104 matches implied the right pose (scores 0.25-0.72, mean 0.55)
-- and localization was 3.9 cm / 0.08 deg median vs the map trajectory. 0.3 = just above the lowest correct match seen; the
-- exact seed and the narrow bootstrap window above are what keep wrong matches out.
POSE_GRAPH.constraint_builder.min_score = 0.3
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.3
-- Constraint search window 2 m / 1 m / 10 deg (robot lua: 4 / 2 / 15): once localized the drift between corrections is
-- centimetres. Measured on legion 2026-10-07 (live stack, 09-23 bag): searches 5x cheaper, matches found 45 vs 39, anchor
-- time unchanged; on the dev bag with the 0.3 floor the slowest search went from 3.0 s to 0.8 s. Rotational floor left at
-- 0.77: 0.8 delayed the bootstrap anchor from 1 s to 15 s.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 2.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(10.)
return options
