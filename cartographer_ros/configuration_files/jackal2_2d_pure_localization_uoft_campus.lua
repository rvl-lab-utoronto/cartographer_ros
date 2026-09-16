-- Pure localization against the U of T campus map (2026-09-16).
--
-- Inherits the live Toronto outdoor config, which is the same base the campus
-- map itself was built on (jackal2_2d_mapping_uoft_campus.lua line 20), so the
-- sensor setup, frames and scan matcher all match what produced the submaps.
-- Do NOT inherit from jackal2_2d_mapping_uoft_campus.lua instead: that one is
-- offline-only, it nils both trimmers, sets optimize_every_n_nodes = 90 and
-- pins 12 background + 12 Ceres threads sized for a 12-core workstation.
include "jackal2_2d_liveslam_toronto.lua"

-- Keep only the live trajectory's recent submaps. The loaded campus map is
-- frozen and untouched by this; without the trimmer the live trajectory would
-- grow a second map alongside it for the whole run.
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- This map is 35 trajectories / 382 submaps / ~40k nodes, two orders of
-- magnitude past what the live config was tuned for, so the pose graph is the
-- bottleneck here, not local SLAM. Both settings below exist to keep it from
-- falling behind real time. A first attempt at 0.05 / 1 (2026-09-16) buried
-- the work queue 39k items deep, stalled the clock and never produced a
-- constraint - see maps/uoft_campus_2026-09/HANDOVER_localization.md.

-- Freezing the loaded submaps makes them constant parameter blocks, but every
-- frozen node and constraint is still a residual block in the problem, so a
-- full solve is expensive no matter how few live submaps there are. 20 is
-- upstream's own pure-localization value.
POSE_GRAPH.optimize_every_n_nodes = 20

-- Leave global search at the default. PoseGraph2D::ComputeConstraint gates on
-- trajectory CONNECTIVITY, not distance, and an unconnected trajectory only
-- gets global_sampling_ratio-sampled whole-map branch-and-bound - against all
-- 382 submaps here, which is ruinous. But we never start unconnected:
-- start_pose_uoft_campus.py always supplies an initial pose, and
-- InitializeGlobalSubmapPoses calls ConnectAlsoToAllFrozen for any trajectory
-- that has one (pose_graph_2d.cc:82-88), so node 0 already gets the cheap
-- prior-guided local matcher. Global search is therefore only a mid-run
-- recovery path, and the inherited rate is the right one for that.
POSE_GRAPH.global_sampling_ratio = 0.003

return options
