-- Trillium: exp3d_batch0 unchanged, but 24 real cores per batch (Legion: 8 CPUs total).
include "exp3d_batch0.lua"
MAP_BUILDER.num_background_threads = 24
return options
