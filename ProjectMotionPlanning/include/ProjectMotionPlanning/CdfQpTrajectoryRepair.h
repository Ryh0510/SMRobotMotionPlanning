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
        // Explicit external workflow: APF stops before QP; CDF rejects unsafe input.
        enum class Stage { Combined, ApfOnly, CdfOnly };
        Stage stage = Stage::Combined;
        bool densifyKnots = true;
        double apfStepSize = .02;
        double apfInfluenceDistance = .08;
        double apfRepulsionGain = 2.5;
        int apfIterationBudget = 1200;
        std::string detectorId = "cdf_abb4600_burnner_detector";
        // Explicit generic mode uses existing selected detectors unchanged.
        std::vector<std::string> detectorIds;
        std::function<bool()> cancelled;
        std::string obstacleId = "A";
        std::string obstacleName = "A.STL";
        // Legacy import metadata only; collision setup never creates scene entities.
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
        // Optional process constraint, evaluated with ordered reference TCP positions.
        // Empty preserves the existing position-only planning behavior.
        std::function<bool(const std::vector<double>&, const Eigen::Vector3d&)> tcpStateAdmissible;
        // Map a proposed orientation through its ordered surface anchor, then solve IK.
        // The indices refer to the ORIGINAL seed edge and its interpolation fraction.
        std::function<bool(std::vector<double>&, std::size_t, double)> surfaceProject;
        // Optimization knots are independent of the fine collision sampling.
        double optimizationMaxJointStep = 0.04;
        double validationMaxJointStep = 0.02;
        // Execution only: 0 = auto (up to 4 isolated query scenes), 1 = serial.
        // Does not change sample spacing, QP tolerances or optimization settings.
        int queryWorkers = 0;
        // Optional synchronous observation; the callback must not mutate the scene.
        std::function<void(const std::string&)> progress;
        std::function<void(const std::vector<double>&)> previewState;
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

    // Failed local APF connection in the submitted seed clock, never a proof of infeasibility.
    struct ApfConnectionFailure
    {
        double beginTime = 0.0;
        double endTime = 0.0;
    };

    struct ProjectCdfQpRepairResult
    {
        bool success = false;
        bool cancelled = false;
        StoredMotionPlan plan;
        std::vector<ApfConnectionFailure> apfConnectionFailures;
        ProjectCdfQpRepairStatistics statistics;
        // Original knots of the actually selected (possibly equivalent-IK) seed.
        robottrajectory::JointTrajectory referenceSeedTrajectory;
        // Immutable source process times, one per final joint knot after retiming.
        std::vector<double> sourceReferenceTimes;
        // Metres per original source control; immutable after GA selection.
        std::vector<double> gaSelectedSprayDistances;
        std::vector<CdfTrajectoryStage> stages;
        std::vector<MotionPlanningDiagnostic> diagnostics;
    };

    class ProjectCdfQpTrajectoryRepairService
    {
    public:
        static bool collisionSetupConfigured(const simulation_project::ProjectDocument& document,
            const std::string& robotId, const ProjectCdfQpRepairOptions& options);
        static std::vector<std::string> distanceDetectorIds(
            const simulation_project::ProjectDocument& document, const std::string& robotId);
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
