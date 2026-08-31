#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <filesystem>
#include <string>
#include <vector>

namespace motion_planning
{
    enum class TrajectoryImportDataKind
    {
        Unknown,
        JointTrajectory,
        CartesianControlPoints
    };

    struct TrajectoryImportOptions
    {
        std::string robotId;
        std::vector<std::string> jointNames;
        std::string planId;
        std::string planName;
        double defaultTimeStep = 0.1;
        double defaultDuration = 10.0;
        double cartesianPositionScale = 0.001;
    };

    struct TrajectoryImportDiagnostic
    {
        std::string code;
        std::string message;
    };

    struct TrajectoryImportResult
    {
        bool success = false;
        TrajectoryImportDataKind dataKind = TrajectoryImportDataKind::Unknown;
        StoredMotionPlan plan;
        std::vector<TrajectoryImportDiagnostic> diagnostics;

        std::string message() const;
    };

    class ProjectTrajectoryImporter
    {
    public:
        static TrajectoryImportResult importFile(
            const std::filesystem::path& path,
            const TrajectoryImportOptions& options = TrajectoryImportOptions());
    };
}
