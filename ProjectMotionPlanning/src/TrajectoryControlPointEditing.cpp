#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

#include <exception>

#include <algorithm>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>
#include <locale>
#include <nlohmann/json.hpp>

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

bool ProjectTrajectoryControlPointEditor::rotateCartesianTrajectoryLocalAxis(StoredMotionPlan& plan,
    CartesianLocalRotationAxis axis, double angleDegrees, std::string* errorMessage)
{
    if(plan.cartesianControlPoints.empty() || !std::isfinite(angleDegrees)) {
        setError(errorMessage, "A nonempty Cartesian trajectory and a finite rotation angle are required.");
        return false;
    }
    Eigen::Vector3d direction;
    switch(axis) {
    case CartesianLocalRotationAxis::X: direction = Eigen::Vector3d::UnitX(); break;
    case CartesianLocalRotationAxis::Y: direction = Eigen::Vector3d::UnitY(); break;
    case CartesianLocalRotationAxis::Z: direction = Eigen::Vector3d::UnitZ(); break;
    default: setError(errorMessage, "The local rotation axis must be X, Y or Z."); return false;
    }
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(
        std::remainder(angleDegrees, 360.0) * (3.14159265358979323846 / 180.0), direction).toRotationMatrix();
    auto points = plan.cartesianControlPoints.points;
    for(auto& point : points) {
        if(!point.tcpPose.matrix().allFinite()) {
            setError(errorMessage, "Trajectory contains a nonfinite pose.");
            return false;
        }
        // Right multiplication uses the point's own frame, not the world frame.
        point.tcpPose.linear() = (point.tcpPose.linear() * rotation).eval();
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
    if(plan.trajectory.points.size()==plan.cartesianControlPoints.points.size() && !plan.jointNames.empty()) {
        nlohmann::json joints=nlohmann::json::array();bool valid=true;
        for(std::size_t i=0;i<plan.trajectory.points.size();++i) {
            const auto& point=plan.trajectory.points[i];
            valid &= point.q.size()==plan.jointNames.size() && std::abs(point.time-plan.cartesianControlPoints.points[i].time)<1e-9 &&
                std::all_of(point.q.begin(),point.q.end(),[](double q){return std::isfinite(q);});
            joints.push_back({{"time",point.time},{"q",point.q}});
        }
        const auto metadata=nlohmann::json::parse(plan.optimizationMetadata,nullptr,false);
        const std::string convention=metadata.is_object()?metadata.value("joint_convention",std::string("caller")):"caller";
        if(valid)text="# RWS_JOINT_SEEDS_V1 "+nlohmann::json{{"kind","tcp_joint_seeds_v1"},
            {"robot_id",plan.robotId},{"joint_names",plan.jointNames},{"points",joints},
            {"motion",CartesianMotionSampler::isMoveL(plan)?"cartesian_linear":"joint_linear"},{"joint_convention",convention}}.dump()+"\n"+text;
    }
    return true;
}

bool ProjectTrajectoryControlPointEditor::serializeJointTable(const StoredMotionPlan& plan,
    std::string& text, std::string* errorMessage)
{
    text.clear();
    if(plan.trajectory.empty() || plan.jointNames.empty()) {
        setError(errorMessage, "A complete joint trajectory is required.");
        return false;
    }
    for(std::size_t j = 0; j < plan.jointNames.size(); ++j) {
        const auto& name = plan.jointNames[j];
        if(name.empty() || std::find(plan.jointNames.begin(), plan.jointNames.begin() + j, name) != plan.jointNames.begin() + j) {
            setError(errorMessage, "Joint names must be nonempty and unique.");
            return false;
        }
    }
    auto metadata = nlohmann::json::parse(plan.optimizationMetadata, nullptr, false);
    const std::string convention = metadata.is_object()
        ? metadata.value("joint_convention", std::string("caller")) : "caller";
    if(convention != "caller" && convention != "runtime" && convention != "irb4600_stored") {
        setError(errorMessage, "Unsupported joint convention.");
        return false;
    }
    std::ostringstream output;
    output.imbue(std::locale::classic());
    output << std::fixed << std::setprecision(6);
    output << "# IK joint angle export\n# Time unit: seconds\n# Joint angle unit: degrees\ntime_s";
    for(std::size_t j = 0; j < plan.jointNames.size(); ++j) output << "\tJ" << j + 1 << "_deg";
    output << '\n';
    nlohmann::json rows = nlohmann::json::array();
    for(std::size_t i = 0; i < plan.trajectory.points.size(); ++i) {
        const auto& point = plan.trajectory.points[i];
        if(point.q.size() != plan.jointNames.size() || !std::isfinite(point.time) ||
            (i && point.time <= plan.trajectory.points[i - 1].time) ||
            !std::all_of(point.q.begin(), point.q.end(), [](double q) {
                return std::isfinite(q) && std::isfinite(q * 180.0 / 3.14159265358979323846);
            })) {
            setError(errorMessage, "Invalid joint row or non-increasing time.");
            return false;
        }
        rows.push_back({{"time", point.time}, {"q", point.q}});
    }
    output << "# RWS_JOINT_TABLE_V1 " << nlohmann::json{
        {"kind", "joint_table_v1"}, {"robot_id", plan.robotId}, {"joint_names", plan.jointNames},
        {"motion", CartesianMotionSampler::isMoveL(plan) ? "cartesian_linear" : "joint_linear"},
        {"joint_convention", convention}, {"points", rows}}.dump() << '\n';
    for(const auto& point : plan.trajectory.points) {
        output << point.time;
        for(double q : point.q) output << '\t' << q * 180.0 / 3.14159265358979323846;
        output << '\n';
    }
    text = output.str();
    return true;
}

bool ProjectTrajectoryControlPointEditor::serializeJointTcpMatrices(const StoredMotionPlan& plan,
    const std::function<Eigen::Isometry3d(const std::vector<double>&)>& worldTcpForwardKinematics,
    bool mapIrb4600StoredJointSigns, std::string& text, std::string* errorMessage)
{
    if(plan.trajectory.empty() || plan.jointNames.empty() || !worldTcpForwardKinematics ||
        (mapIrb4600StoredJointSigns && plan.jointNames.size() != 6)) {
        setError(errorMessage, "A complete joint trajectory and actual TCP kinematics are required.");
        return false;
    }
    StoredMotionPlan cartesian;
    cartesian.cartesianControlPoints.points.reserve(plan.trajectory.points.size());
    try {
        for(std::size_t i = 0; i < plan.trajectory.points.size(); ++i) {
            const auto& source = plan.trajectory.points[i];
            if(source.q.size() != plan.jointNames.size() || !std::isfinite(source.time) ||
                !std::all_of(source.q.begin(), source.q.end(), [](double q) { return std::isfinite(q); })) {
                setError(errorMessage, "Invalid joint row or timestamp at point " + std::to_string(i + 1));
                return false;
            }
            // Undo stored ABB signs exactly once, without wrapping legal joint turns.
            const auto q = mapIrb4600StoredJointSigns
                ? ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(source.q) : source.q;
            robottrajectory::TimedCartesianPoint point;
            point.time = source.time;
            point.tcpPose = worldTcpForwardKinematics(q);
            const auto& pose = point.tcpPose;
            if(!pose.matrix().allFinite() || !(pose.translation() * 1000.0).allFinite() ||
                (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() > 1e-7 ||
                std::abs(pose.linear().determinant() - 1.0) > 1e-7 ||
                (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() > 1e-9) {
                setError(errorMessage, "Invalid actual-model TCP pose at point " + std::to_string(i + 1));
                return false;
            }
            cartesian.cartesianControlPoints.points.push_back(std::move(point));
        }
    } catch(const std::exception& error) {
        setError(errorMessage, std::string("TCP forward kinematics failed: ") + error.what());
        return false;
    } catch(...) {
        setError(errorMessage, "TCP forward kinematics failed.");
        return false;
    }
    cartesian.robotId=plan.robotId;cartesian.jointNames=plan.jointNames;cartesian.trajectory=plan.trajectory;
    auto metadata=nlohmann::json::parse(plan.optimizationMetadata,nullptr,false);
    if(!metadata.is_object())metadata=nlohmann::json::object();
    metadata["joint_convention"]="runtime";cartesian.optimizationMetadata=metadata.dump();
    if(mapIrb4600StoredJointSigns)for(auto& point:cartesian.trajectory.points)
        point.q=ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(point.q);
    return serializeCartesianMatrices(cartesian, text, errorMessage);
}

} // namespace motion_planning
