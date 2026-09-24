-- z search window 3 m: the candidate production value once the accelerometer
-- bias is removed (readme section 12). Everything else as tuned.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 3.
return options
