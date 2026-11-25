#pragma once
#include <ProjectMotionPlanning/ProjectMotionPlanning.h>

namespace motion_planning {
enum class SprayPoseConstraint {
    FixedTcp,
    SurfacePivotFixedDistance,
    SurfacePivotAdjustableDistance
};
struct SprayAxisContinuityOptions {
    // Fixed TCP and first surface hit imply an unchanged spray direction.
    // Tilting about that hit necessarily moves the TCP. Legacy bounded motion
    // remains explicit for callers that permit it.
    SprayPoseConstraint poseConstraint = SprayPoseConstraint::SurfacePivotFixedDistance;
    std::vector<std::string> collisionDetectorIds;
    Eigen::Vector3d sprayAxis = Eigen::Vector3d::UnitZ();
    double maximumRollRadians = 1.5707963267948966;
    double maximumTiltRadians = .5235987755982988;
    double maximumTcpAdjustment = .03;
    // Empty uses scene objects explicitly classified as workpiece.
    std::string workpieceId;
    bool allowNonSprayingTransitions = true;
    // Output controls use actual-model MoveL. Validation subdivisions remain
    // internal and never become exported trajectory controls.
    double clearance = 0.;
    double validationJointStep = .001;
    bool enforceJointPositionLimits = false;
    int workers = 8;
    std::size_t maximumOutputPoints = 30000;
    std::function<bool()> cancelled;
    std::function<void(const std::string&)> progress;
};
struct SprayAxisContinuityResult {
    bool success = false;
    StoredMotionPlan plan;
    std::string message;
    std::string statistics;
};
// Independent of GA/APF and Qt. Sparse controls retain source stations and
// surface hits. MoveL IK/collision failures and cancellation refuse output.
class ProjectSprayAxisContinuityService {
public:
    static SprayAxisContinuityResult repair(const simulation_project::ProjectDocument& document,
        const std::filesystem::path& base, const StoredMotionPlan& input,
        const SprayAxisContinuityOptions& options);
};
}
