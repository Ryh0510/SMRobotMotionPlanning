#pragma once
#include <ProjectMotionPlanning/ExternalTrajectoryEvaluation.h>

namespace motion_planning
{
    struct GaSurfaceSearchOptions
    {
        // Immutable original TCP + distance * original local +Z defines the anchor.
        double sprayDistance = 0.10; // metres; UI displays millimetres
        // Finite feedback rounds, no single-point time deadline. Zero disables feedback.
        int maxConnectionFeedbackRounds = 8;
    };

    // Resolve the one enabled saved tool TCP; never silently substitute the flange.
    bool resolvePlanningTcp(const simulation_project::ProjectDocument& document,
        const std::string& robotId, std::string& linkName,
        Eigen::Isometry3d& linkFromTcp, std::string& error);

    // Restore immutable surface/process references saved by the pipeline before
    // downsampling/replaying its exact output. Reject malformed/stale references.
    bool prepareGaApfDownsamplingRequest(ExternalTrajectoryEvaluationRequest& request,
        std::string& error);

    // Cartesian-only input first enumerates physical IK and selects Top-1 for
    // initialization (a complete leading prefix if missing poses require GA).
    // GA checks control points only. APF connects them and independently replays
    // the returned joints. Optional CDF may retain the accepted APF on failure.
    // All joints at this boundary use actual runtime-model conventions.
    ProjectCdfQpRepairResult runGaApfCdfPipeline(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath,
        ExternalTrajectoryEvaluationRequest request, bool useCdf,
        const GaSurfaceSearchOptions& surface = {});
}
