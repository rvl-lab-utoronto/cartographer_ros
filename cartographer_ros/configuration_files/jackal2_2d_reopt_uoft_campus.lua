-- Joint RE-OPTIMIZATION pass for the campus map (2026-09-18, user asked to see Ceres progress).
--
-- Identical to the mapping lua in every way that affects the result: same local SLAM, same pose
-- graph settings, both trimmers off, so loading a state with this and no bag is a pure
-- re-optimization over every trajectory and every constraint, exactly as before. The ONLY
-- difference is that Ceres reports what it did.
--
-- log_solver_summary prints ceres::Solver::Summary::FullReport() after each solve: initial and
-- final cost, iteration count, per-stage timings and the termination reason. That is the
-- observable progress the user wanted for the long re-optimizations, which otherwise run silent
-- for ten to twenty minutes.
--
-- log_solver_summary is deliberately NOT enabled in the mapping lua: during a mapping stage the
-- optimizer fires every optimize_every_n_nodes (90), so a full report each time would bury the
-- log that the placement checks are parsed from.
--
-- The live per-iteration cost stream is a SEPARATE switch and is now on by default: cartographer
-- was patched on 2026-09-18 to expose Ceres's minimizer_progress_to_stdout through
-- common/proto/ceres_solver_options.proto (fields 4 and 5), and the mapping lua sets it for the
-- pose graph optimizer. This file inherits it through the include below, so a reopt pass gets
-- both the per-iteration stream and the full summary. Do not set it again here.
include "jackal2_2d_mapping_uoft_campus.lua"

POSE_GRAPH.optimization_problem.log_solver_summary = true

return options
