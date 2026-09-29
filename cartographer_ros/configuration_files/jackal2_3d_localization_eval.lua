-- Offline evaluation of jackal2_3d_pure_localization_uoft_campus.lua: identical except the
-- pure-localization trimmer is off, so the localized trajectory's nodes survive into the saved
-- state and can be scored against the reference (the trimmer deletes them).
include "jackal2_3d_pure_localization_uoft_campus.lua"
TRAJECTORY_BUILDER.pure_localization_trimmer = nil
return options
