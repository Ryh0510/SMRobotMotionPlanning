#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <filesystem>
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

        double safetyMargin = 0.01;
        double targetClearance = 0.0;
        double finiteDifferenceStep = 5.0e-4;
        double distanceThreshold = 5.0;
        double trustRegion = 0.02;
        double seedCorridor = 0.10;
        double smoothWeight = 0.16;
        double seedTrackingWeight = 0.40;
        double repairGain = 0.25;
        int segmentIntermediateSamples = 1;
        int maxIterations = 5;
        bool keepEndpoints = true;
        double validationMaxJointStep = 0.03;
    };

    struct ProjectCdfQpRepairStatistics
    {
        int inputWaypointCount = 0;
        int iterations = 0;
        int qpIterations = 0;
        int collisionQueries = 0;
        int clampedSeedValues = 0;
        int invalidSegmentCount = 0;
        double initialMinimumPhi = 0.0;
        double finalMinimumPhi = 0.0;
        double maximumCorrection = 0.0;
        double maximumSlack = 0.0;
    };

    struct ProjectCdfQpRepairResult
    {
        bool success = false;
        StoredMotionPlan plan;
        ProjectCdfQpRepairStatistics statistics;
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
