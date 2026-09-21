-- Campus mapping for a bag whose start pose is KNOWN (seeded via
-- -initial_trajectory_poses), 2026-09-15. Same as the campus mapping lua except
-- that whole-map global constraint search is switched off.
--
-- Why: the initial pose is only a starting value, nothing constrains the
-- trajectory to stay there. In the 18:06 pilot the robot was parked across the
-- handover (so the motion filter produced almost no nodes to match while it sat
-- still), the 10 s local-search window lapsed, search fell back to the whole-map
-- branch-and-bound, and two spurious matches to trajectories 11 and 17 dragged
-- the whole trajectory 438 m off its correct seeded position.
-- With global_sampling_ratio = 0 only the prior-guided local matcher can fire,
-- so a seeded trajectory can gain genuine nearby evidence but cannot be
-- teleported by a far-away false match.
include "jackal2_2d_mapping_uoft_campus.lua"

POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9

-- Lock-on bootstrap (cartographer 6008854; 2026-09-17 19:20, kept after global tuning). A seed is
-- only an initial value, and with global search off a pair of trajectories that has never been
-- connected gets NO cross-trajectory search at all (pose_graph_2d.cc: local search needs a prior
-- connection). So the bootstrap is the only way in: every finished submap within
-- max_constraint_distance is matched on every node, windowed, until the first cross-trajectory
-- constraint. Measured on bag 2 seeded onto bag 1 (tune_global/baseline): locks within seconds,
-- 4335 cross-bag closures, residual median 0.011 m, 94 % of bag 2's walls within 0.2 m of bag 1's.
POSE_GRAPH.constraint_builder.initial_pose_num_nodes = 100000     -- whole bag; ends at the first cross-bag constraint
POSE_GRAPH.constraint_builder.initial_pose_linear_search_window = 15.
POSE_GRAPH.constraint_builder.initial_pose_angular_search_window = math.rad(30.)
POSE_GRAPH.constraint_builder.initial_pose_min_score = 0.55

return options
