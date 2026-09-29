-- Batch-joint run, batch 0 of the sliding-window overlap-verified split (chain3d/batches.json):
-- unfrozen from the start (no load_state), every bag its own trajectory in ONE process, seeded
-- from the untrimmed 2D map (batch0's gen_seeds.py) so global search is not needed to place them.
-- Local SLAM: rotation_weight 4 (measured: does almost all the work, isolation test 2026-09-25,
-- 0.77 deg vs baseline 1.02 deg on the worst bag) + odometry off (principled: the 14% wheel-scale
-- error belongs out of the prior, per the existing chain lua's own rationale).
-- Constraint search: seeded bags start sub-metre from each other (per jackal2_3d_campus_joint.lua's
-- own prior reasoning), so windows are sized to seed error, not to blind single-bag drift -- but
-- widened past the Sept 22 defaults (xy 6/z 10) since that run left 10% of real overlaps with ZERO
-- constraints. global_sampling_ratio raised off its near-zero (Sept22: 0., localization-tuned
-- default) so trajectory pairs keep getting searched even after they first connect somewhere.
include "jackal2_3d_mapping_uoft_campus.lua"
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4.
options.use_odometry = false

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 8.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 15.
POSE_GRAPH.constraint_builder.max_constraint_distance = 25.
POSE_GRAPH.global_sampling_ratio = 0.01
POSE_GRAPH.global_constraint_search_after_n_seconds = 20.
-- Sept22 postmortem's flagged-but-untested fix: solves were the bottleneck (478 in 15 min for 23
-- bags), not the search windows. A small batch doesn't need frequent intermediate solves.
POSE_GRAPH.optimize_every_n_nodes = 1000
return options
