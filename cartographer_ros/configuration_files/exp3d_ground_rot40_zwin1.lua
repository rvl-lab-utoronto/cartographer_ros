-- Ground-in cloud, z window at the upstream 1 m, rotation prior loosened to 40
-- (upstream 4e2). Groundless this knob was pointless: submap and scan carried
-- the same IMU tilt (common mode). With ground AND turns a body-frame pitch
-- bias tilts the map ground in a direction that rotates with heading, so the
-- submap and the incoming scan disagree and the matcher can see the bias, if
-- the prior lets it move. Test: does the loop close at 1 m z window?
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 40.
return options
