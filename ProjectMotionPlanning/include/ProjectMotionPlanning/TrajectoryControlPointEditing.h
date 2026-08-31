#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <cstddef>
#include <string>

namespace motion_planning
{

enum class CartesianControlPointInsertLocation
{
    Before,
    After
};

class ProjectTrajectoryControlPointEditor
{
public:
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
