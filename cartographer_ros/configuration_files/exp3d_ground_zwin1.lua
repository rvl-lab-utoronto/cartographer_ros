-- Ground-in cloud (filtered_point_cloud3) with the z search window back at the
-- UPSTREAM 1 m. The 20 m window in the tuned config exists only because the
-- groundless cloud drifted metres in z; if ground pins z, the workaround should
-- be unnecessary and constraint search ~20x cheaper. Everything else as tuned.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 1.
return options
