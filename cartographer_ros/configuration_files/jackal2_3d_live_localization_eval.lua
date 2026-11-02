-- Offline scoring of the LIVE 3D localization lua (jackal2_3d_live_localization_uoft_campus.lua): identical except the
-- pure-localization trimmer is off, so the localized trajectory's nodes survive into the saved state and can be scored
-- against the reference (the trimmer deletes them). Same relation as jackal2_3d_localization_eval.lua has to
-- jackal2_3d_pure_localization_uoft_campus.lua; this one also carries the live lua's later pose-graph IMU changes.
-- First used 2026-11-02 for the Munich final map (slam_jackal/maps/munich_2026/final/final_b.sbatch).
include "jackal2_3d_live_localization_uoft_campus.lua"
TRAJECTORY_BUILDER.pure_localization_trimmer = nil
return options
