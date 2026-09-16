-- TRIM PASS for the chunked campus map (2026-09-11). Not a mapping config:
-- run by cartographer_offline_node with -load_state_filename <chunk.pbstream>
-- -load_frozen_state=false and NO bags. Loading UNFROZEN restores the
-- intra-submap constraints the overlapping trimmer needs for submap
-- freshness (map_builder.cc LoadState: a frozen load keeps only node->submap
-- membership, so frozen submaps are invisible to the trimmer and a trimmer
-- inside a frozen chunk is a no-op on the map). The final optimization then
-- jointly re-optimizes the whole trimmed map before it is saved.
--
-- Trimmer semantics (overlapping_submaps_trimmer_2d.cc): per coverage cell,
-- keep the fresh_submaps_count freshest submaps; drop any submap left with
-- less than min_covered_area m^2 of such cells. Freshest = latest node time,
-- so the most recent traversal of each area survives.
include "jackal2_2d_mapping_uoft_campus.lua"

POSE_GRAPH.overlapping_submaps_trimmer_2d = {
  fresh_submaps_count = 2,
  min_covered_area = 50,        -- m^2 of uniquely-covered area to keep a submap
  min_added_submaps_count = 1,  -- fire on the first optimization of the pass
}

return options
