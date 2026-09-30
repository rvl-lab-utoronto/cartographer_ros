-- LIVE 3D pure localization against the trimmed 3D campus map (stack.launch localization:=localization3d).
-- Everything that shapes the scan matching comes from jackal2_3d_pure_localization_uoft_campus.lua (which
-- takes the mapping side from exp3d_batch.lua, so live scans are built exactly like the map's submaps:
-- /filtered_point_cloud3, debiased IMU, no wheel odometry). The one live-only choice is below.
-- VERIFIED 2026-09-30 (user) on a full replay of wild_uoft_2026-09-22-10-30-01 (655 s, climbs to z 2.2 m): 847 constraints to the map, match offsets
-- median 6 cm / p90 12 cm, published pose within 2.3 cm median of the pose graph on the climbs.
include "jackal2_3d_pure_localization_uoft_campus.lua"

-- The navigation stack, costmaps and EKF are planar and were built around the 2D mode, which publishes
-- the projected pose (jackal2_2d_liveslam_toronto.lua). Localize in 6-DoF, publish map->odom and
-- /tracked_pose as (x, y, yaw) the same way, so nothing downstream sees roll, pitch or map z.
options.publish_frame_projected_to_2d = true
return options
