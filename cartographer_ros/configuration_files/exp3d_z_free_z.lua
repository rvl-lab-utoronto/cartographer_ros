-- 3D z probe 'free_z' (z_probe_3d.sh, 2026-09-21).
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
return options
