-- DIAGNOSTIC (2026-10-07): offline fast lua (0.3 floors) with the robot's wide bootstrap windows, to test whether the 09-23 16:20
-- bag failed to anchor because its seed (first map node) is further than 1 m / 5 deg from where the kept submaps say it was.
include "jackal2_3d_offline_seeded_fast_localization_uoft_campus.lua"
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 7.
POSE_GRAPH.constraint_builder.initial_pose_linear_z_search_window = 2.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(45.)
return options
