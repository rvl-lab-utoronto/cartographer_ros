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

return options
