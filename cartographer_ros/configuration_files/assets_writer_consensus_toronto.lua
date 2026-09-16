-- Per-bag obstacle raster for the CONSENSUS costmap (2026-09-15). One run per
-- bag against a single-trajectory pbstream; the rasters are then intersected so
-- a cell counts as occupied only where every bag that observed it agrees.
--
-- Differs from assets_writer_obstacle_map_toronto.lua in three ways:
--  * hit/miss probabilities are 0.7/0.3 instead of 0.55/0.49. write_ros_map
--    renders unknown cells as grey 128 and known cells as
--    255*((1-p)-0.1)/0.8, so with the default 0.55/0.49 a single miss lands on
--    131 and a single hit on 112 - only a few grey levels from unknown, which
--    makes "observed free" and "never observed" indistinguishable. At 0.7/0.3 a
--    single miss is 191 and a single hit 64, so the three states separate
--    cleanly and per-cell agreement can actually be counted.
--  * 0.15 m, matching the planner's global costmap, instead of 0.05 m: 35
--    campus-sized rasters at 0.05 m would be ~200 Mpx each.
--  * no voxel_filter_and_remove_moving_objects. Its HybridGrid is anchored at
--    the map origin and capped at 8 doubling levels, so at a 0.05 m voxel it
--    only reaches ~200 m and dies with "Check failed: new_bits <= 8" on any
--    trajectory far from the origin (campus coordinates run to ~550 m).
--    Raising voxel_size to ~0.3 m would fit but is coarser than the 0.15 m
--    output grid. Dropping it is fine here for two reasons:
--    /filtered_point_cloud2 is already the masker's people-removed cloud, and
--    the consensus rule is itself a stronger dynamic-object filter - anything
--    present in only some of the bags fails unanimity by construction.
options = {
  tracking_frame = "os_imu",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.5,
      max_range = 10.0,   -- global costmap point_cloud_sensor1 obstacle_range
    },
    {
      -- z relative to os_imu: point_cloud_sensor1 min/max_obstacle_height in
      -- planning_jackal/config/costmap/global_costmap_params_toronto.yaml.
      action = "vertical_range_filter",
      min_z = -0.3,
      max_z = 0.15,
    },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.7,
        miss_probability = 0.3,
      },
      filestem = "consensus",
      resolution = 0.15,
    },
  },
}
return options
