#pragma once

#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include <ProjectMotionPlanning/ProjectMotionPlanning.h>

#include <limits>

namespace motion_planning::detail
{
    struct SignedDistanceSample
    {
        bool valid = false;
        bool inCollision = false;
        double phi = -std::numeric_limits<double>::max();
        double rawDistance = std::numeric_limits<double>::max();
        collision::Vec3 nearestDirection = collision::Vec3::Zero();
        bool hasNearestDirection = false;
        bool hasNearestFeature = false;
        collision::ObjectID objectA = 0, objectB = 0;
        collision::Vec3 localPointA = collision::Vec3::Zero();
        collision::Vec3 localPointB = collision::Vec3::Zero();
        std::string message;
    };

    struct CdfLinearization
    {
        SignedDistanceSample sample;
        std::vector<double> gradient;
    };

    SignedDistanceSample evaluateSignedPhi(ProjectPlanningSceneSnapshot& scene,
        const std::vector<double>& q, double safetyMargin, double distanceThreshold,
        ProjectCdfQpRepairStatistics& statistics);

    CdfLinearization linearizeSignedPhi(ProjectPlanningSceneSnapshot& scene,
        const std::vector<double>& q, const std::vector<JointBound>& bounds,
        double safetyMargin, double distanceThreshold, double finiteDifferenceStep,
        ProjectCdfQpRepairStatistics& statistics);
}
