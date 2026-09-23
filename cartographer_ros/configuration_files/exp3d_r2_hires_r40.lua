-- 3D LOCAL sweep round 2, variant 'hires_r40' (2026-09-21). One knob at a time from the TUNED
-- baseline (translation_weight 1, xy window 10 m, z window 20 m, fix_z_in_3d false), which
-- closes the 534 m loop at 0.35 m in this tuning mode. Round 1 swept from upstream defaults,
-- where the loop never closed, so those results could not separate local quality from
-- closure. Everything here is judged by pb3d.big_loop_error and whether closure still fires.
include "jackal2_3d_mapping_uoft_campus.lua"
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.sampling_ratio = 1.0
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 40.   -- upstream 20; cloud p99 range is 32 m
return options
