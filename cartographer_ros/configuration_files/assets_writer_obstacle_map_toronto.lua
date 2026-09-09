-- Obstacle raster for the planner's static layer from a carto_input bag and
-- its pbstream (2026-09-09, ported from geodex_jackal_ws assets_writer_
-- obstacle_map.lua, VALIDATED there 2026-09-07 on the office map). Re-projects
-- EVERY PointCloud2 / LaserScan topic in the bag (the masker's walls cloud
-- /filtered_point_cloud2 and /filtered_scan; strip /filtered_scan from the bag
-- if it is unwanted) through the optimized trajectory into the map frame,
-- keeps the global costmap's obstacle band, removes moving objects and writes
-- <prefix>obstacle_map.pgm/.yaml with map_server thresholds.
-- UNVALIDATED here (campus scale).
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
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = 0.05,
    },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      filestem = "obstacle_map",
      -- 0.05, not the office map's 0.02: the campus is ~1 km across, 0.02
      -- would be a multi gigapixel pgm. The global costmap runs at 0.15.
      resolution = 0.05,
    },
  },
}
return options
