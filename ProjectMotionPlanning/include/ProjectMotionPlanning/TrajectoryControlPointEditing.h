#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <cstddef>
#include <functional>
#include <string>

namespace motion_planning
{

enum class CartesianControlPointInsertLocation
{
    Before,
    After
};

enum class CartesianLocalRotationAxis
{
    X,
    Y,
    Z
};

class ProjectTrajectoryControlPointEditor
{
public:
    // World-frame rotation about the origin: fixed X, then Y, then Z; translation last.
    static bool transformCartesianTrajectory(StoredMotionPlan& plan,
        const Eigen::Vector3d& translationMm, const Eigen::Vector3d& rotationDegrees,
        std::string* errorMessage = nullptr);
    // Each pose rotates about its own axis; positions and process metadata stay fixed.
    static bool rotateCartesianTrajectoryLocalAxis(StoredMotionPlan& plan,
        CartesianLocalRotationAxis axis, double angleDegrees,
        std::string* errorMessage = nullptr);
    // Four matrix rows (translation in mm), timestamp in seconds, blank separator.
    static bool serializeCartesianMatrices(const StoredMotionPlan& plan,
        std::string& text, std::string* errorMessage = nullptr);

    // Export every stored joint point through an independent actual-model TCP snapshot.
    // FK consumes runtime radians and returns world TCP meters; recorded times are retained.
    static bool serializeJointTcpMatrices(const StoredMotionPlan& plan,
        const std::function<Eigen::Isometry3d(const std::vector<double>&)>& worldTcpForwardKinematics,
        bool mapIrb4600StoredJointSigns, std::string& text, std::string* errorMessage = nullptr);

    static bool makeInsertedCartesianControlPoint(
        const StoredMotionPlan& plan,
        std::size_t pointIndex,
        CartesianControlPointInsertLocation location,
        robottrajectory::TimedCartesianPoint& point,
        std::string* errorMessage = nullptr);

    static bool insertCartesianControlPoint(
        StoredMotionPlan& plan,
        std::size_t pointIndex,
        CartesianControlPointInsertLocation location,
        const robottrajectory::TimedCartesianPoint& point,
        std::string* errorMessage = nullptr);

    static bool updateCartesianControlPoint(
        StoredMotionPlan& plan,
        std::size_t pointIndex,
        const robottrajectory::TimedCartesianPoint& point,
        std::string* errorMessage = nullptr);

    static bool removeCartesianControlPoint(
        StoredMotionPlan& plan,
        std::size_t pointIndex,
        std::string* errorMessage = nullptr);
};

} // namespace motion_planning
