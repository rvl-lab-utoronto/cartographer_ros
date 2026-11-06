-- Frozen-map localization for the nc layer: the seeded offline localization lua with the pure-localization trimmer OFF, so the localized trajectory's nodes survive into the saved state (the trimmer deletes them).
include "jackal2_3d_offline_seeded_localization_uoft_campus.lua"
TRAJECTORY_BUILDER.pure_localization_trimmer = nil
return options
