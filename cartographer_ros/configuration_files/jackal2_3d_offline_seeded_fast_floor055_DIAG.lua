-- DIAGNOSTIC (2026-10-07): the robot's LIVE constraint settings run through the offline pipeline (fast lua threads only):
-- score floors 0.55 and the live bootstrap windows (7 m / 2 m / 45 deg), to compare localization accuracy with the 0.3 floor
-- on the same bag, map and pipeline. (floor055_DIAG v1 kept the offline 1 m / 5 deg bootstrap window with 0.55: the bootstrap
-- found nothing and a false global match then locked the trajectory 322 m away.)
include "jackal2_3d_offline_seeded_fast_localization_uoft_campus.lua"
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.55
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 7.
POSE_GRAPH.constraint_builder.initial_pose_linear_z_search_window = 2.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(45.)
return options
