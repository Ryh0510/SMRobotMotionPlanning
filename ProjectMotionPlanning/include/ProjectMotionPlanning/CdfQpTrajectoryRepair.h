#pragma once

#include <MotionPlanningCore/MotionPlanning.h>
#include <ProjectMotionPlanning/TrajectoryQuality.h>

#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace motion_planning
{
    struct ProjectCdfQpRepairOptions
    {
        std::string detectorId = "cdf_abb4600_burnner_detector";
        std::string obstacleId = "burnner";
        std::string obstacleName = "burnner";
        std::string obstacleRobotSourcePath = "data/drake_models/burnner/urdf/burnner.urdf";
        double obstacleX = 0.65;
        double obstacleY = 0.85;
        double obstacleZ = 0.65;
        double obstacleRoll = 0.0;
        double obstaclePitch = 1.57079632679489661923;
        double obstacleYaw = 1.57079632679489661923;

        // Hard position corridor relative to the original TCP control-point polyline.
        // Zero explicitly disables it for callers without actual calibrated FK.
        double apfMaxTcpDeviation = 0.10;
        // Optional original Cartesian input (world TCP), one pose per seed knot.
        // Alternative IK is opt-in; full original endpoint poses remain fixed.
        std::vector<Eigen::Isometry3d> cartesianTargets;
        bool allowEquivalentEndpointConfigurations = false;
        double safetyMargin = 0.01;
        double targetClearance = 0.0;
        double finiteDifferenceStep = 5.0e-4;
        double distanceThreshold = 5.0;
        double trustRegion = 0.012;
        double seedCorridor = 0.06;
        double smoothWeight = 2.0;
        double seedTrackingWeight = 0.55;
        double repairGain = 0.12;
        int segmentIntermediateSamples = 1;
        int maxIterations = 1;
        // Guided mode: each three strength units enable one 128..2-node scale cycle.
        int postSmoothingIterations = 6;
        double postSmoothingStep = 0.50;
        double postSmoothingSeedWeight = 0.10;
        bool keepEndpoints = true;
        bool retimeOutput = true;
        // Used only when the actual robot model omits a dynamic limit (rad/s, rad/s^2).
        double fallbackMaxVelocity = 1.0471975511965976;
        double fallbackMaxAcceleration = 2.0943951023931953;
        // Actual world TCP snapshot; consumes runtime joint signs, worker-thread only.
        TrajectoryForwardKinematics worldForwardKinematics;
        // Optimization knots are independent of the fine collision sampling.
        double optimizationMaxJointStep = 0.04;
        double validationMaxJointStep = 0.02;
        // Execution only: 0 = auto (up to 4 isolated query scenes), 1 = serial.
        // Does not change sample spacing, QP tolerances or optimization settings.
        int queryWorkers = 0;
        // Optional synchronous observation; the callback must not mutate the scene.
        std::function<void(const std::string&)> progress;
    };

    struct ProjectCdfQpRepairStatistics
    {
        int inputWaypointCount = 0;
        int iterations = 0;
        int qpIterations = 0;
        int acceptedQpSteps = 0;
        int collisionQueries = 0;
        int clampedSeedValues = 0;
        int invalidSegmentCount = 0;
        double initialMinimumPhi = 0.0;
        double finalMinimumPhi = 0.0;
        double maximumCorrection = 0.0;
        double maximumSlack = 0.0;
    };

    struct CdfTrajectoryStage
    {
        std::string name;
        StoredMotionPlan plan;
        TrajectoryQuality quality;
        std::vector<Eigen::Vector3d> referenceTcpPositions;
        double minimumPhi = 0.0;
        int invalidSegments = 0;
        bool tcpCorridorValid = true;
        double tcpDeviationLimit = 0.0;
        double elapsedSeconds = 0.0;
    };

    struct ProjectCdfQpRepairResult
    {
        bool success = false;
        StoredMotionPlan plan;
        ProjectCdfQpRepairStatistics statistics;
        // Original knots of the actually selected (possibly equivalent-IK) seed.
        robottrajectory::JointTrajectory referenceSeedTrajectory;
        std::vector<CdfTrajectoryStage> stages;
        std::vector<MotionPlanningDiagnostic> diagnostics;
    };

    class ProjectCdfQpTrajectoryRepairService
    {
    public:
        static bool ensureCollisionSetup(
            simulation_project::ProjectDocument& document,
            const std::string& robotId,
            const ProjectCdfQpRepairOptions& options,
            std::string* detectorId = nullptr,
            std::vector<MotionPlanningDiagnostic>* diagnostics = nullptr);

        ProjectCdfQpRepairResult repair(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& projectBasePath,
            const std::string& robotId,
            const std::vector<std::string>& jointNames,
            const robottrajectory::JointTrajectory& seedTrajectory,
            const ProjectCdfQpRepairOptions& options = ProjectCdfQpRepairOptions()) const;
    };
}
