#pragma once
#include <ProjectMotionPlanning/ExternalTrajectoryEvaluation.h>

namespace motion_planning
{
    struct WorkpiecePoseSearchOptions
    {
        bool enabled = false;
        std::size_t maxCandidates = 24;
        double maxDurationSeconds = 120;
        Eigen::Vector3d lowerPosition = Eigen::Vector3d::Zero();
        Eigen::Vector3d upperPosition = Eigen::Vector3d::Zero();
        Eigen::Vector3d lowerRpy = Eigen::Vector3d(1.5707963267948966,0,-3.141592653589793);
        Eigen::Vector3d upperRpy = Eigen::Vector3d(1.5707963267948966,0,3.141592653589793);
        // Explicit STL-from-UF2 calibration; identity means STL axes are UF2 axes.
        Eigen::Isometry3d workpieceFromConstraintFrame = Eigen::Isometry3d::Identity();
        bool requireYVertical = true;
        Eigen::Vector3d groundNormalWorld = Eigen::Vector3d::UnitZ();
        double axisToleranceRad = 0.000017453292519943296; // 0.001 degrees
    };
    struct WorkpiecePoseAttempt
    {
        simulation_project::TransformDesc pose;
        bool success = false;
        std::string message;
    };
    struct ExternalWorkpiecePlanningResult
    {
        ExternalTrajectoryEvaluationResult evaluation;
        std::optional<simulation_project::TransformDesc> selectedWorkpiecePose;
        std::vector<WorkpiecePoseAttempt> attempts;
        bool timedOut = false;
        std::size_t rejectedByConstraints = 0;
    };
    class ProjectExternalWorkpiecePlanner
    {
    public:
        static bool poseAllowed(const simulation_project::TransformDesc& pose,
            const WorkpiecePoseSearchOptions& options);
        static ExternalWorkpiecePlanningResult plan(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath,
            const ExternalTrajectoryEvaluationRequest& evaluation,
            const WorkpiecePoseSearchOptions& search = {});
    };
}
