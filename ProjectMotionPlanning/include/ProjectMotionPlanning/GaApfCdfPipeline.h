#pragma once
#include <ProjectMotionPlanning/ExternalTrajectoryEvaluation.h>

namespace motion_planning
{
    struct GaSurfaceSearchOptions
    {
        // Immutable original TCP + distance * original local +Z defines the anchor.
        double sprayDistance = 0.10; // metres; UI displays millimetres
    };

    // Resolve the one enabled saved tool TCP; never silently substitute the flange.
    bool resolvePlanningTcp(const simulation_project::ProjectDocument& document,
        const std::string& robotId, std::string& linkName,
        Eigen::Isometry3d& linkFromTcp, std::string& error);

    // GA checks control points only. APF connects them and independently replays
    // the returned joints. Optional CDF may retain the accepted APF on failure.
    // All joints at this boundary use actual runtime-model conventions.
    ProjectCdfQpRepairResult runGaApfCdfPipeline(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath,
        ExternalTrajectoryEvaluationRequest request, bool useCdf,
        const GaSurfaceSearchOptions& surface = {});
}
