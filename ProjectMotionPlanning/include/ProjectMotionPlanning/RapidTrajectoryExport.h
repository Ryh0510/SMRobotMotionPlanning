#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <Eigen/Geometry>
#include <filesystem>
#include <functional>
#include <string>

namespace simulation_project { struct ProjectDocument; }

namespace motion_planning
{
    struct RapidTrajectoryExportOptions
    {
        // Actual world TCP, consuming runtime model angles (radians).
        // Must be an independent snapshot; exporting must not move the live robot.
        std::function<Eigen::Isometry3d(const std::vector<double>&)> worldTcpForwardKinematics;
        bool mapIrb4600StoredJointSigns = false;
    };

    struct RapidTrajectoryExportResult
    {
        bool success = false;
        std::size_t pointCount = 0;
        std::string program;
        std::string error;
    };

    class ProjectRapidTrajectoryExporter
    {
    public:
        // Generates the supplied template's robtarget/MoveL format with the actual
        // TCP coordinates, but tool0 as explicitly requested (no tool compensation).
        // robconf remains the supplied template placeholder; no tooldata is emitted.
        // This exports poses, not a certification of controller interpolation.
        static RapidTrajectoryExportResult generate(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath,
            const StoredMotionPlan& storedPlan,
            const RapidTrajectoryExportOptions& options);

        // Direct world Cartesian poses in meters, without IK/FK or tool compensation.
        // Shares the CDF template: MainModule, MoveL/v50/z10/tool0/wobj0.
        static RapidTrajectoryExportResult generateCartesian(const StoredMotionPlan& plan);
    };
}
