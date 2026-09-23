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
  -- Fork (2026-09-22, user): submaps whose ORIGIN is inside the Myhal
  -- building are a separate population. Its glass walls reflect the LiDAR,
  -- so indoor-started submaps paint phantom structure over the real outdoor
  -- ground and, being often fresher, would evict the genuine outdoor submaps
  -- around the origin. Inside is trimmed against inside, outside against
  -- outside; an inside submap covering outdoor ground is NOT overlap.
  -- Vertices: OSM way 403302492 (Myhal Centre) pushed through the fitted
  -- gps_fit + osm/refine.npy chain into map metres; the origin (0,0) is
  -- inside. Loading dock (way 639848940, x -39..-20) is deliberately not
  -- part of it. Flat x,y list, CCW or CW does not matter (even-odd rule).
  inside_polygon = {
       34.8,   30.6,
       -7.0,   32.5,
       -9.6,   32.7,
      -10.1,   20.4,
      -10.2,   18.9,
      -10.9,    3.7,
      -11.0,    2.3,
      -11.6,  -11.3,
       -9.0,  -11.4,
       32.8,  -13.3,
       32.9,  -10.1,
       33.1,   -6.8,
       34.5,   24.1,
       34.7,   27.6,
  },
}

return options
