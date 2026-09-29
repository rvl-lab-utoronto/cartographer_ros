-- Place ONE extra bag on the FROZEN untrimmed 2D campus map (load_frozen_state=true), to seed its
-- 3D placement. Unseeded: every constraint to the map comes from the 2D global sampler, which pulses
-- once per (node, unconnected submap), so the ratio scales with the map's 2840 submaps.
include "jackal2_2d_mapping_uoft_campus.lua"
POSE_GRAPH.global_sampling_ratio = 0.002   -- 2840 submaps -> ~6 global searches per node (0.005: ~50 min for an 83-node bag)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.optimize_every_n_nodes = 200
return options
