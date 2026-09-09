-- Whole map 3D walls cloud for rviz from a carto_input bag and its pbstream
-- (2026-09-09, ported from geodex_jackal_ws assets_writer_pointcloud.lua).
-- x y z intensity ply in the map frame, z = 0 at the LiDAR, moving objects
-- removed. Voxel 0.05 m. UNVALIDATED here (campus scale: expect a large ply).
VOXEL_SIZE = 0.05
options = {
  tracking_frame = "os_imu",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.5,
      max_range = 200.0,   -- TRAJECTORY_BUILDER_2D.max_range of the live outdoor lua
    },
    {
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = VOXEL_SIZE,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "write_ply",
      filename = "points.ply",
    },
  },
}
return options
