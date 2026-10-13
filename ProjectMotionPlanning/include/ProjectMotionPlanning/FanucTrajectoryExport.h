#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <Eigen/Geometry>
#include <functional>
#include <string>

namespace motion_planning
{
    struct FanucTrajectoryExportOptions
    {
        std::string programName = "JHA";
        // Independent actual-model snapshot, runtime joint radians -> world TCP meters.
        std::function<Eigen::Isometry3d(const std::vector<double>&)> worldTcpForwardKinematics;
        bool mapIrb4600StoredJointSigns = false;
    };

    struct FanucTrajectoryExportResult
    {
        bool success = false;
        std::size_t pointCount = 0;
        std::string program;
        std::string error;
    };

    class ProjectFanucTrajectoryExporter
    {
    public:
        // JHA.LS Cartesian template: UF2/UT1, L, 400mm/sec, CNT100, XYZ(mm), WPR(deg).
        // CONFIG 'N U T, 0, 0, 0' is a template value, NOT derived FANUC IK data.
        // World coordinates are assigned to UF2; configure that frame and UT1 to
        // match the simulation. No controller calibration or joint-branch conversion.
        static FanucTrajectoryExportResult generate(
            const StoredMotionPlan& plan, const FanucTrajectoryExportOptions& options);
    };
}
