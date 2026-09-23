-- 3D assets from a tuned 3D pbstream (2026-09-21, plans/carto-3d/plan.md step 3).
-- The 2D pipeline's two writers (assets_writer_obstacle_map_uoft_campus.lua for the planner
-- raster, assets_writer_pointcloud_toronto.lua for the rviz ply) both work unchanged on a 3D
-- pbstream, because the assets writer only re-projects the bag through a trajectory. This file
-- adds what only a 3D map can give: orthographic X-ray views down each axis, which is how you
-- see whether the vertical structure is coherent or smeared.
VOXEL_SIZE = 0.05
include "transform.lua"
options = {
  tracking_frame = "os_imu",
  pipeline = {
    { action = "min_max_range_filter", min_range = 0.5, max_range = 200. },
    { action = "dump_num_points" },
    -- top down: the same view as the occupancy render, but intensity-weighted over all heights
    { action = "write_xray_image", voxel_size = VOXEL_SIZE, filename = "xray_xy", transform = XY_TRANSFORM },
    -- the two side views. On a groundless, vertical-feature cloud these should show crisp
    -- facades; vertical smearing here is z drift that the top-down view cannot reveal.
    { action = "write_xray_image", voxel_size = VOXEL_SIZE, filename = "xray_xz", transform = XZ_TRANSFORM },
    { action = "write_xray_image", voxel_size = VOXEL_SIZE, filename = "xray_yz", transform = YZ_TRANSFORM },
    { action = "voxel_filter_and_remove_moving_objects", voxel_size = 0.1 },
    { action = "write_ply", filename = "points_3d.ply" },
  },
}
return options
