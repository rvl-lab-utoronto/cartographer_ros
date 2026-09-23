-- 3D z probe 'rot_w4' (z_probe_3d.sh, 2026-09-21).
include "jackal2_3d_mapping_uoft_campus.lua"
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4.
return options
