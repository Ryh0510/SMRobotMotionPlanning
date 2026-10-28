#include <ProjectMotionPlanning/FanucTrajectoryExport.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <set>
#include <sstream>
#include <stdexcept>

namespace motion_planning
{
    namespace
    {
        constexpr double pi = 3.14159265358979323846;

        bool validProgramName(const std::string& name)
        {
            const auto letter = [](char c) { return c >= 'A' && c <= 'Z'; };
            return !name.empty() && name.size() <= 8 && letter(name.front()) &&
                std::all_of(name.begin(), name.end(), [&](char c) {
                    return letter(c) || (c >= '0' && c <= '9') || c == '_';
                });
        }

        bool validPose(const Eigen::Isometry3d& pose)
        {
            return pose.matrix().allFinite() && (pose.translation() * 1000.0).allFinite() &&
                (pose.linear().transpose() * pose.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-7 &&
                std::abs(pose.linear().determinant() - 1.0) < 1e-7 &&
                (pose.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9;
        }

        Eigen::Vector3d wprRadians(const Eigen::Matrix3d& rotation, const Eigen::Vector3d* previous)
        {
            // FANUC W/P/R: Rz(R) * Ry(P) * Rx(W), not axis-angle or quaternion xyz.
            const double cosine = std::hypot(rotation(0, 0), rotation(1, 0));
            Eigen::Vector3d angles;
            angles.y() = std::atan2(-rotation(2, 0), cosine);
            if(cosine > 1e-10) {
                angles.x() = std::atan2(rotation(2, 1), rotation(2, 2));
                angles.z() = std::atan2(rotation(1, 0), rotation(0, 0));
            } else {
                // At gimbal lock keep the preceding W and solve the coupled R.
                const double sign = angles.y() >= 0.0 ? 1.0 : -1.0;
                angles.x() = previous ? previous->x() : 0.0;
                const double coupled = std::atan2(sign * rotation(0, 1), rotation(1, 1));
                angles.z() = std::remainder(sign * (angles.x() - coupled), 2.0 * pi);
            }
            if(previous) {
                Eigen::Vector3d alternate(angles.x() + pi, pi - angles.y(), angles.z() + pi);
                // Compare equivalent angle lifts near the preceding WPR. Comparing
                // principal values first can select an artificial 180-degree flip.
                for(int j = 0; j < 3; ++j) {
                    angles[j] = (*previous)[j] + std::remainder(angles[j] - (*previous)[j], 2.0 * pi);
                    alternate[j] = (*previous)[j] + std::remainder(alternate[j] - (*previous)[j], 2.0 * pi);
                }
                if((alternate - *previous).squaredNorm() < (angles - *previous).squaredNorm()) angles = alternate;
            }
            return angles;
        }

        double printable(double value)
        {
            return std::abs(value) < 0.0005 ? 0.0 : value;
        }
    }

    static FanucTrajectoryExportResult generateProgram(
        const StoredMotionPlan& plan, const FanucTrajectoryExportOptions& options, bool cartesian)
    {
        FanucTrajectoryExportResult result;
        try {
            if(!validProgramName(options.programName)) {
                throw std::runtime_error("FANUC program name must be 1-8 uppercase ASCII letters/digits/underscores, starting with a letter.");
            }
            if(!cartesian && (plan.trajectory.empty() || plan.jointNames.size() != 6 ||
                std::set<std::string>(plan.jointNames.begin(), plan.jointNames.end()).size() != 6 ||
                std::any_of(plan.jointNames.begin(), plan.jointNames.end(), [](const auto& name) { return name.empty(); }) ||
                !options.worldTcpForwardKinematics)) {
                throw std::runtime_error("A complete six-axis joint trajectory and actual TCP kinematics are required.");
            }
            const std::size_t count = cartesian ? plan.cartesianControlPoints.points.size() : plan.trajectory.points.size();
            if(count == 0) throw std::runtime_error("A Cartesian trajectory is required.");
            std::ostringstream out;
            out.imbue(std::locale::classic());
            out << "/PROG  " << options.programName << "\r\n/ATTR\r\n"
                << "OWNER\t\t= MNEDITOR;\r\nCOMMENT\t\t= \"Spray Trajectory\";\r\n"
                << "PROG_SIZE\t= 0;\r\nVERSION\t\t= 0;\r\n"
                << "LINE_COUNT\t= " << count + 2 << ";\r\n"
                << "MEMORY_SIZE\t= 0;\r\nPROTECT\t\t= READ_WRITE;\r\n"
                << "DEFAULT_GROUP\t= 1,*,*,*,*;\r\n/MN\r\n"
                << "   1:  UFRAME_NUM=2 ;\r\n   2:  UTOOL_NUM=1 ;\r\n";
            for(std::size_t i = 0; i < count; ++i) {
                out << "   " << i + 3 << ":  L P[" << i + 1 << "] 400mm/sec CNT100    ;\r\n";
            }
            out << "/POS\r\n" << std::fixed << std::setprecision(3);
            Eigen::Vector3d previous = Eigen::Vector3d::Zero();
            for(std::size_t i = 0; i < count; ++i) {
                Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
                if(cartesian) {
                    const auto& point = plan.cartesianControlPoints.points[i];
                    if(!std::isfinite(point.time))
                        throw std::runtime_error("Invalid Cartesian time at point " + std::to_string(i + 1));
                    tcp = point.tcpPose;
                } else {
                    const auto& point = plan.trajectory.points[i];
                    if(point.q.size() != 6 || !std::isfinite(point.time) ||
                        !std::all_of(point.q.begin(), point.q.end(), [](double value) { return std::isfinite(value); })) {
                        throw std::runtime_error("Invalid joint trajectory data at point " + std::to_string(i + 1));
                    }
                    // Preserve legal turns; only undo the existing stored ABB sign convention.
                    const auto q = options.mapIrb4600StoredJointSigns
                        ? ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(point.q) : point.q;
                    tcp = options.worldTcpForwardKinematics(q);
                }
                if(!validPose(tcp)) {
                    throw std::runtime_error("Invalid TCP pose at point " + std::to_string(i + 1));
                }
                const Eigen::Vector3d mm = tcp.translation() * 1000.0;
                const auto angles = wprRadians(tcp.linear(), i ? &previous : nullptr);
                previous = angles;
                const Eigen::Vector3d degrees = angles * (180.0 / pi);
                out << "P[" << i + 1 << "]{\r\n   GP1:\r\n"
                    << "\tUF : 2, UT : 1,\t\tCONFIG : 'N U T, 0, 0, 0',\r\n"
                    << "\tX = " << printable(mm.x()) << "  mm,\tY = " << printable(mm.y())
                    << "  mm,\tZ = " << printable(mm.z()) << "  mm,\r\n"
                    << "\tW = " << printable(degrees.x()) << " deg,\tP = " << printable(degrees.y())
                    << " deg,\tR = " << printable(degrees.z()) << " deg\r\n};\r\n";
            }
            out << "/END\r\n";
            result.program = out.str();
            result.pointCount = count;
            result.success = true;
        } catch(const std::exception& error) {
            result.error = error.what();
        }
        return result;
    }

    FanucTrajectoryExportResult ProjectFanucTrajectoryExporter::generate(
        const StoredMotionPlan& plan, const FanucTrajectoryExportOptions& options)
    {
        return generateProgram(plan, options, false);
    }

    FanucTrajectoryExportResult ProjectFanucTrajectoryExporter::generateCartesian(
        const StoredMotionPlan& plan, const FanucTrajectoryExportOptions& options)
    {
        return generateProgram(plan, options, true);
    }

}
