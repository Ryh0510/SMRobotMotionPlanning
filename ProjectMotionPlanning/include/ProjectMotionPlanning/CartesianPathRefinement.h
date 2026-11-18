#pragma once

#include <ProjectMotionPlanning/ProjectMotionPlanning.h>

namespace motion_planning {
struct CartesianPathRefinementOptions {
    std::vector<std::string> collisionDetectorIds;
    // Empty resolves the saved mounted tool TCP. Explicit values also support a flange.
    std::string tcpLinkName;
    Eigen::Isometry3d linkFromTcp = Eigen::Isometry3d::Identity();
    bool enforceJointPositionLimits = true;
    double maximumPositionDeviation = .005;
    double maximumOrientationDeviation = .3490658503988659;
    double smoothingSigma = 1.;
    double bendingWeight = 100.;
    // Zero preserves the original fairing objective. Positive values prefer a
    // tool-local axis in the plane normal to worldUp, without increasing its
    // original per-station tilt. Positions retain the same hard corridor.
    double horizontalAxisWeight = 0.;
    Eigen::Vector3d horizontalLocalAxis = Eigen::Vector3d::UnitZ();
    // Zero permits a free swing. A unit axis perpendicular to horizontalLocalAxis
    // restricts orientation candidates to twists about that preserved axis.
    Eigen::Vector3d horizontalPreservedLocalAxis = Eigen::Vector3d::Zero();
    Eigen::Vector3d worldUp = Eigen::Vector3d::UnitZ();
    bool preserveEndpointEncoders = true;
    double validationJointStep = .00025;
    std::size_t maximumPointCount = 2999;
    int maximumSearchIterations = 512;
    int queryWorkers = 8;
    std::function<bool()> cancelled;
    std::function<void(const std::string&)> progress;
};
struct CartesianPathRefinementResult {
    bool success = false;
    StoredMotionPlan plan;
    std::string message;
    std::string statistics;
};
// Ordered, collision-constrained pose fairing. Every input station and both
// endpoint encoders are retained by default. No scene/document mutation.
class ProjectCartesianPathRefinementService {
public:
    CartesianPathRefinementResult refine(const simulation_project::ProjectDocument& document,
        const std::filesystem::path& base, const StoredMotionPlan& input,
        const CartesianPathRefinementOptions& options = {}) const;
};
}
