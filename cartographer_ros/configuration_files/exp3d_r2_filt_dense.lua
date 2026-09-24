-- 3D LOCAL sweep round 2, variant 'filt_dense' (2026-09-24). One knob at a time from the TUNED
-- config, which closes the 534 m loop at 0.28 m big-loop error against the tuned 2D map's
-- 0.40 m. Run in MAPPING mode (optimize_every_n_nodes 90, default sampling 0.3), not the
-- global-SLAM-off tuning mode: it is 13x faster (2 min vs 27) and it measures the
-- DELIVERABLE config directly. Caveat, stated because it matters: mapping mode does not
-- isolate local SLAM the way the cartographer guide's protocol asks, so this is a screen.
-- Whatever wins here gets confirmed in tuning mode before it goes into the lua.
include "jackal2_3d_mapping_uoft_campus.lua"
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 1.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 300
return options
