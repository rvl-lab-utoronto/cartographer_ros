-- Extend the FROZEN merged campus map with one more bag (cartographer_offline_node
-- -load_state_filename=MERGED -load_frozen_state=true). Same local SLAM + matchers as the batches;
-- a seeded bag (-initial_trajectory_poses) is Connected to its to_trajectory and gets local search
-- there; every other frozen trajectory needs a global-sampler hit first. The sampler pulses once per
-- (node, unconnected submap), so the ratio scales with the map's submap count (~900 here).
include "exp3d_batch.lua"
POSE_GRAPH.global_sampling_ratio = 0.002   -- per unconnected submap: 831 -> ~1.7 global searches per node. The seed connects to_trajectory (local search); 0.01 stalled 18-18-06-28 for 1 h at 25%
return options
