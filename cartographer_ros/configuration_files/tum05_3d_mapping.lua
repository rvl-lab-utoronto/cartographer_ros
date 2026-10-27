-- Offline 3D mapping for the Munich tum05 Jackal (slam_jackal/maps/munich_2026), 2026-10-21.
-- The Toronto lr_v4 batch recipe unchanged: exp3d_batch.lua (local: rotation_weight 4, odometry off; global: windows
-- xy 8 / z 15 m, max_constraint_distance 25 m, global_sampling_ratio 0.05, windowed search kept 600 s after a connection,
-- optimize every 1000 nodes) + 200 free-space voxels per ray (Trillium config exp3d_batch_f200.lua, the lr_v4_f200 map).
-- tum05 is the same Jackal + an Ouster OS-1-128 at 2048x10 (Toronto: OS-1-64); tracking frame os_imu as in Toronto.
-- The mapping jobs (mm_map.sbatch) only add thread counts on top of this file.
include "exp3d_batch.lua"
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 200
return options
