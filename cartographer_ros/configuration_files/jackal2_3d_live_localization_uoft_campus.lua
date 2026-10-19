-- LIVE 3D pure localization against the trimmed 3D campus map (stack.launch localization:=localization3d).
-- Everything that shapes the scan matching comes from jackal2_3d_pure_localization_uoft_campus.lua (which
-- takes the mapping side from exp3d_batch.lua, so live scans are built exactly like the map's submaps:
-- /filtered_point_cloud3, debiased IMU, no wheel odometry). The one live-only choice is below.
-- VERIFIED 2026-09-30 (user) on a full replay of wild_uoft_2026-09-22-10-30-01 (655 s, climbs to z 2.2 m): 847 constraints to the map, match offsets
-- median 6 cm / p90 12 cm, published pose within 2.3 cm median of the pose graph on the climbs.
include "jackal2_3d_pure_localization_uoft_campus.lua"

-- Pose-graph IMU terms for the live trajectory (2026-10-13, traj_dataset_creation/docs/reports/localization_divergence.md;
-- moved here from the offline lua 2026-10-19, user decision). UNVALIDATED (live): verified only in the offline lockstep loop
-- (Trillium, 28 bags: the 7 diverging bags 0.52-17.2 m -> 0.06-0.15 m xy p95, 18 controls unchanged within a few cm); not yet
-- run live on the robot nor in a legion bag-play replay.
-- 1. Online IMU extrinsics OFF. The tracking frame is os_imu itself, so the live trajectory's IMU calibration is the identity
--    by construction, but the inherited mapping setting (use_online_imu_extrinsics_in_3d = true) left it free in a problem of
--    a few hundred nodes: on the first solves (robot parked, gyro deltas ~0) Ceres sent it 110-180 deg away and the gravity
--    constant to its 0 bound, a local minimum it never left; with the IMU frame upside down the rotation residuals invert every
--    turn (08-19-14-52-02: optimized node turned +28 deg while 7 map matches said -33 deg; 21 m / 44 deg off, 17.2 m p95 ->
--    0.08 m with this line; 09-23-16-20-16 the same, 36 m).
-- 2. IMU rotation / acceleration residuals OFF. The gyro in /ouster_imu_debiased carries odom_2d's bias estimate, and that
--    estimate goes 2-10 deg/s wrong for minutes in 10 of 88 bags: its stillness gate is commands-only, and a ~1 s turn with no
--    command on any topic (a push; the wheels turn at up to 32 deg/s) lands in its 3 s bias buffer (08-26-20-06-34 from
--    181 s, 09-22-11-01-49 from 434 s; the recorded and the loop's debiased gyro both read +150 deg over a 30 s straight).
--    odom_2d's bias_gate_measured (wheels + gyro stillness, on since 2026-10-19) removes the cause. The rotation residual
--    (1.6e4 / dt, quadratic) then outweighs every map constraint (Huber 1e1, capped to a constant pull) and the optimized yaw
--    follows the gyro: 0.89 -> 0.09 m and 0.53 -> 0.15 m p95 without the terms. Map matches are 6-DoF, so roll/pitch/z stay anchored; local SLAM still uses the IMU as before.
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false
POSE_GRAPH.optimization_problem.acceleration_weight = 0.
POSE_GRAPH.optimization_problem.rotation_weight = 0.
-- Diagnostic logging off (2026-10-19, user: nothing unnecessary runs live): the residual histograms span every
-- constraint of the frozen map (~377k) after each optimization, the match histograms every constraint batch. Logs
-- only. UNVALIDATED (live).
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.constraint_builder.log_matches = false

-- The navigation stack, costmaps and EKF are planar and were built around the 2D mode, which publishes
-- the projected pose (jackal2_2d_liveslam_toronto.lua). Localize in 6-DoF, publish map->odom and
-- /tracked_pose as (x, y, yaw) the same way, so nothing downstream sees roll, pitch or map z.
options.publish_frame_projected_to_2d = true
return options
