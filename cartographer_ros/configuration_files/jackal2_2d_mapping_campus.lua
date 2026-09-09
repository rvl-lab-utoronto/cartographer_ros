-- Campus scale multi bag OFFLINE mapping (2026-09-09). UNVALIDATED: written for
-- the first joint cartographer_offline_node run over the carto_input bags
-- listed in slam_jackal/maps/uoft_campus_2026-09/bag_order.txt (one trajectory
-- per bag, none frozen, one final optimization).
--
-- Everything sensor and matcher related is INHERITED from the live outdoor lua
-- (jackal2_2d_liveslam_toronto.lua: max_range 200 m, 0.1 m submaps, voxel 0.1,
-- adaptive filter 100 pts / 1.0 m, hit 0.55 / miss 0.48, global search 15 s
-- after a trajectory is unconnected at the default 0.003 sampling), so the
-- map is built from exactly what live SLAM sees on campus. Only what a joint
-- offline run needs differs:
--  * both trimmers OFF: the live lua keeps 5 submaps for CPU; a pbstream
--    written with trimmers is not a map.
--  * optimize_every_n_nodes 90 (live: 1): the offline node runs a final
--    optimization anyway, and per node optimization over a ~50 trajectory
--    graph would dominate the wall time. ~90 nodes is one submap, enough for
--    constraint proposals along the way. Global search stays ON throughout:
--    every new bag must find its anchor in the earlier bags, and cross day
--    loop closures are the point of the joint run.
include "jackal2_2d_liveslam_toronto.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = nil
POSE_GRAPH.overlapping_submaps_trimmer_2d = nil
POSE_GRAPH.optimize_every_n_nodes = 90

return options
