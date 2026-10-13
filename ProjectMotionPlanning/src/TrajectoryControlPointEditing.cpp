#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>

#include <algorithm>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>
#include <locale>

namespace motion_planning
{
namespace
{

void setError(std::string* errorMessage, const std::string& message)
{
    if (errorMessage)
    {
        *errorMessage = message;
    }
}

bool validateCartesianPointIndex(
    const StoredMotionPlan& plan,
    std::size_t pointIndex,
    std::string* errorMessage)
{
    if (plan.cartesianControlPoints.points.empty())
    {
        setError(errorMessage, "The selected trajectory has no Cartesian control points.");
        return false;
    }

    if (pointIndex >= plan.cartesianControlPoints.points.size())
    {
        setError(errorMessage, "The selected control point index is out of range.");
        return false;
    }

    return true;
}

double positiveTimeStepNear(const std::vector<robottrajectory::TimedCartesianPoint>& points, std::size_t pointIndex)
{
    if (pointIndex > 0)
    {
        const double previousStep = points[pointIndex].time - points[pointIndex - 1].time;
        if (std::isfinite(previousStep) && previousStep > 0.0)
        {
            return previousStep;
        }
    }

    if (pointIndex + 1 < points.size())
    {
        const double nextStep = points[pointIndex + 1].time - points[pointIndex].time;
        if (std::isfinite(nextStep) && nextStep > 0.0)
        {
            return nextStep;
        }
    }

    return 0.1;
}

void invalidateSolvedJointTrajectory(StoredMotionPlan& plan)
{
    plan.trajectory.points.clear();
}

} // namespace

bool ProjectTrajectoryControlPointEditor::makeInsertedCartesianControlPoint(
    const StoredMotionPlan& plan,
    std::size_t pointIndex,
    CartesianControlPointInsertLocation location,
    robottrajectory::TimedCartesianPoint& point,
    std::string* errorMessage)
{
    if (!validateCartesianPointIndex(plan, pointIndex, errorMessage))
    {
        return false;
    }

    const auto& points = plan.cartesianControlPoints.points;
    point = points[pointIndex];

    if (location == CartesianControlPointInsertLocation::Before)
    {
        if (pointIndex > 0)
        {
            point.time = 0.5 * (points[pointIndex - 1].time + points[pointIndex].time);
        }
        else
        {
            point.time = std::max(0.0, points[pointIndex].time - positiveTimeStepNear(points, pointIndex));
        }
    }
    else
    {
        if (pointIndex + 1 < points.size())
        {
            point.time = 0.5 * (points[pointIndex].time + points[pointIndex + 1].time);
        }
        else
        {
            point.time = points[pointIndex].time + positiveTimeStepNear(points, pointIndex);
        }
    }

    return true;
}

bool ProjectTrajectoryControlPointEditor::insertCartesianControlPoint(
    StoredMotionPlan& plan,
    std::size_t pointIndex,
    CartesianControlPointInsertLocation location,
    const robottrajectory::TimedCartesianPoint& point,
    std::string* errorMessage)
{
    if (!validateCartesianPointIndex(plan, pointIndex, errorMessage))
    {
        return false;
    }

    auto& points = plan.cartesianControlPoints.points;
    const auto insertIndex = location == CartesianControlPointInsertLocation::Before ? pointIndex : pointIndex + 1;
    points.insert(points.begin() + static_cast<std::ptrdiff_t>(insertIndex), point);
    plan.cartesianControlPoints.sortByTime();
    invalidateSolvedJointTrajectory(plan);
    return true;
}

bool ProjectTrajectoryControlPointEditor::updateCartesianControlPoint(
    StoredMotionPlan& plan,
    std::size_t pointIndex,
    const robottrajectory::TimedCartesianPoint& point,
    std::string* errorMessage)
{
    if (!validateCartesianPointIndex(plan, pointIndex, errorMessage))
    {
        return false;
    }

    plan.cartesianControlPoints.points[pointIndex] = point;
    plan.cartesianControlPoints.sortByTime();
    invalidateSolvedJointTrajectory(plan);
    return true;
}

bool ProjectTrajectoryControlPointEditor::removeCartesianControlPoint(
    StoredMotionPlan& plan,
    std::size_t pointIndex,
    std::string* errorMessage)
{
    if (!validateCartesianPointIndex(plan, pointIndex, errorMessage))
    {
        return false;
    }

    auto& points = plan.cartesianControlPoints.points;
    if (points.size() <= 1)
    {
        setError(errorMessage, "The trajectory must keep at least one Cartesian control point.");
        return false;
    }

    points.erase(points.begin() + static_cast<std::ptrdiff_t>(pointIndex));
    invalidateSolvedJointTrajectory(plan);
    return true;
}

bool ProjectTrajectoryControlPointEditor::transformCartesianTrajectory(StoredMotionPlan& plan,
    const Eigen::Vector3d& translationMm, const Eigen::Vector3d& rotationDegrees,
    std::string* errorMessage)
{
    if(plan.cartesianControlPoints.empty() || !translationMm.allFinite() || !rotationDegrees.allFinite()) {
        setError(errorMessage, "A nonempty Cartesian trajectory and finite transform are required.");
        return false;
    }
    const Eigen::Vector3d radians = rotationDegrees * (3.14159265358979323846 / 180.0);
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() = (Eigen::AngleAxisd(radians.z(), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(radians.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(radians.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
    transform.translation() = translationMm * 0.001;
    auto points = plan.cartesianControlPoints.points;
    for(auto& point : points) {
        point.tcpPose = transform * point.tcpPose;
        if(!point.tcpPose.matrix().allFinite()) {
            setError(errorMessage, "Trajectory contains a nonfinite pose.");
            return false;
        }
    }
    plan.cartesianControlPoints.points = std::move(points);
    invalidateSolvedJointTrajectory(plan);
    return true;
}

bool ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(const StoredMotionPlan& plan,
    std::string& text, std::string* errorMessage)
{
    if(plan.cartesianControlPoints.empty()) {
        setError(errorMessage, "The selected trajectory has no Cartesian control points.");
        return false;
    }
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::scientific << std::setprecision(16);
    for(const auto& point : plan.cartesianControlPoints.points) {
        if(!std::isfinite(point.time) || !point.tcpPose.matrix().allFinite()) {
            setError(errorMessage, "Cannot export a nonfinite pose or timestamp.");
            return false;
        }
        Eigen::Matrix4d matrix = point.tcpPose.matrix();
        matrix.block<3, 1>(0, 3) *= 1000.0;
        for(int row = 0; row < 4; ++row) {
            for(int col = 0; col < 4; ++col) {
                if(col) output << ' ';
                output << matrix(row, col);
            }
            output << '\n';
        }
        output << point.time << "\n\n";
    }
    text = output.str();
    return true;
}

} // namespace motion_planning
