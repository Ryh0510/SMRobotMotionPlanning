#include <ProjectMotionPlanning/RapidTrajectoryExport.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace motion_planning
{
    namespace
    {
        bool validPose(const Eigen::Isometry3d& pose)
        {
            return pose.matrix().allFinite() &&
                (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1.0e-5 &&
                std::abs(pose.linear().determinant() - 1.0) < 1.0e-5 &&
                (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1.0e-9;
        }

        void writePose(std::ostream& out, const Eigen::Isometry3d& pose, Eigen::Quaterniond rotation)
        {
            const Eigen::Vector3d mm = pose.translation() * 1000.0;
            out << std::fixed << std::setprecision(6)
                << "[[" << mm.x() << ',' << mm.y() << ',' << mm.z() << "],["
                << std::setprecision(9) << rotation.w() << ',' << rotation.x() << ','
                << rotation.y() << ',' << rotation.z() << "]]";
        }
    }

    RapidTrajectoryExportResult ProjectRapidTrajectoryExporter::generate(
        const simulation_project::ProjectDocument& document, const std::filesystem::path& basePath,
        const StoredMotionPlan& plan, const RapidTrajectoryExportOptions& options)
    {
        RapidTrajectoryExportResult result;
        try {
            if(plan.trajectory.empty() || plan.jointNames.size() != 6 || !options.worldTcpForwardKinematics) {
                throw std::runtime_error("A six-axis joint trajectory and actual TCP kinematics are required.");
            }
            simulation_runtime::ProjectSimulationRuntime runtime;
            const auto loaded = runtime.loadProject(document, basePath);
            if(!loaded.success) { throw std::runtime_error(loaded.message); }
            const auto* runtimeRobot = runtime.robot(plan.robotId);
            if(!runtimeRobot || !runtimeRobot->instance || runtimeRobot->instance->getState().q.size() != 6) {
                throw std::runtime_error("RAPID export requires a six-axis serial robot without external axes.");
            }
            std::array<robot::RobotJoint, 6> joints;
            for(std::size_t j = 0; j < joints.size(); ++j) {
                const auto found = std::find_if(runtimeRobot->model.joints.begin(), runtimeRobot->model.joints.end(),
                    [&](const auto& joint) { return joint.name == plan.jointNames[j]; });
                if(found == runtimeRobot->model.joints.end() || found->type != robot::JointType::Revolute ||
                    found->dofIndex < 0 || found->isLoop ||
                    (j > 0 && found->parent != joints[j - 1].child)) {
                    throw std::runtime_error("Joint names must follow the six-axis serial chain from base to flange.");
                }
                joints[j] = *found;
            }
            std::vector<Eigen::Isometry3d> tcpPoses;
            tcpPoses.reserve(plan.trajectory.points.size());
            Eigen::Isometry3d flangeToTcp = Eigen::Isometry3d::Identity();
            for(std::size_t i = 0; i < plan.trajectory.points.size(); ++i) {
                const auto& point = plan.trajectory.points[i];
                if(point.q.size() != 6 || !std::isfinite(point.time) ||
                    !std::all_of(point.q.begin(), point.q.end(), [](double q) { return std::isfinite(q); })) {
                    throw std::runtime_error("Invalid joint data at point " + std::to_string(i + 1));
                }
                const auto q = options.mapIrb4600StoredJointSigns
                    ? ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(point.q) : point.q;
                for(std::size_t j = 0; j < joints.size(); ++j) {
                    runtimeRobot->instance->setJoint(joints[j].dofIndex, q[j]);
                }
                runtimeRobot->instance->update();
                const Eigen::Isometry3d flange = runtimeRobot->instance->getLinkTransform(joints.back().child);
                const Eigen::Isometry3d tcp = options.worldTcpForwardKinematics(q);
                if(!validPose(flange) || !validPose(tcp) || !(tcp.translation() * 1000.0).allFinite()) {
                    throw std::runtime_error("Invalid actual-model TCP pose at point " + std::to_string(i + 1));
                }
                const Eigen::Isometry3d local = flange.inverse() * tcp;
                if(i == 0) {
                    flangeToTcp = local;
                } else if((local.translation() - flangeToTcp.translation()).norm() > 1.0e-6 ||
                    Eigen::AngleAxisd(flangeToTcp.linear().transpose() * local.linear()).angle() > 1.0e-6) {
                    throw std::runtime_error("The selected TCP is not fixed to the last-axis flange, or the model snapshot changed.");
                }
                tcpPoses.push_back(tcp);
            }
            if(!validPose(flangeToTcp) || !(flangeToTcp.translation() * 1000.0).allFinite()) {
                throw std::runtime_error("Invalid flange-to-TCP tool definition.");
            }
            std::ostringstream out;
            out.imbue(std::locale::classic());
            out << "MODULE MainModule\r\n"
                << "  ! CDF/QP targets with explicitly selected tool0\r\n"
                << "  ! Generated points: " << tcpPoses.size() << "\r\n"
                << "  ! Positions: simulation world in mm, mapped to controller wobj0.\r\n"
                << "  ! Active tool: tool0 (controller built-in flange TCP).\r\n"
                << "  ! Target coordinates are unchanged; they now specify flange poses.\r\n"
                << "  ! Orientations: normalized quaternion [w,x,y,z].\r\n"
                << "  ! robconf [0,0,0,0] is the supplied template placeholder, NOT an IK branch.\r\n"
                << "  ! Resolve robot configurations in RobotStudio before execution.\r\n"
                << "  ! MoveL/v50/z10 changes joint interpolation and does not preserve CDF timing.\r\n"
                << "  ! Validate controller path, blending and collisions before execution.\r\n\r\n";
            Eigen::Quaterniond previous = Eigen::Quaterniond::Identity();
            for(std::size_t i = 0; i < tcpPoses.size(); ++i) {
                Eigen::Quaterniond rotation(tcpPoses[i].linear());
                rotation.normalize();
                if((i == 0 && rotation.w() < 0.0) || (i > 0 && previous.dot(rotation) < 0.0)) {
                    rotation.coeffs() *= -1.0;
                }
                previous = rotation;
                // robtarget consists of pos, orient, robconf and extax fields.
                std::ostringstream pose;
                pose.imbue(std::locale::classic());
                writePose(pose, tcpPoses[i], rotation);
                std::string fields = pose.str();
                fields.pop_back();
                out << "  CONST robtarget p" << i + 1 << " := " << fields
                    << ",[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];\r\n";
            }
            out << "\r\n  PROC main()\r\n";
            for(std::size_t i = 0; i < tcpPoses.size(); ++i) {
                out << "    MoveL p" << i + 1 << ", v50, z10, tool0\\WObj:=wobj0;\r\n";
            }
            out << "  ENDPROC\r\nENDMODULE\r\n";
            result.program = out.str();
            result.pointCount = tcpPoses.size();
            result.success = true;
        } catch(const std::exception& error) {
            result.error = error.what();
        }
        return result;
    }
}
