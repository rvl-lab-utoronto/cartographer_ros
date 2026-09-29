-- 3D pure localization against the merged campus pbstream (write_merged_pbstream.py), loaded with
-- load_frozen_state. Mapping-side settings (local SLAM, grids, matcher options) come from the map's
-- own batch config so live scans are built exactly like the map's submaps; the localization-only
-- choices below mirror jackal2_2d_pure_localization_uoft_campus.lua, translated to 3D.
-- UNVALIDATED 2026-09-28: first used by the held-out-bag offline localization test.
include "exp3d_batch.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
-- constraints to the frozen map: local search only, never a global search once localized
POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e11
POSE_GRAPH.global_sampling_ratio = 0.0001
POSE_GRAPH.constraint_builder.sampling_ratio = 0.1
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 4.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(15.)
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
return options
