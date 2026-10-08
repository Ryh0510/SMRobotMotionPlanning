#include <ProjectMotionPlanning/RapidTrajectoryExport.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <SimulationProject/ProjectIo.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
    void require(bool condition, const std::string& message)
    {
        if(!condition) { throw std::runtime_error(message); }
    }
}

int main(int argc, char** argv)
{
    try {
        require(argc == 2, "Pass the ABB project path");
        const std::filesystem::path path(argv[1]);
        simulation_project::ProjectDocument document;
        std::string error;
        require(simulation_project::loadProjectDocument(path, document, &error), error);
        for(auto& robot : document.robots) {
            if(robot.id == "ABB4600_urdf") {
                robot.baseTransform.x = 1.7;
                robot.baseTransform.y = -0.8;
                robot.baseTransform.roll = 0.13;
                robot.baseTransform.yaw = -0.6;
            }
        }
        simulation_runtime::ProjectSimulationRuntime runtime;
        const auto loaded = runtime.loadProject(document, path.parent_path());
        require(loaded.success, loaded.message);
        const auto* robot = runtime.robot("ABB4600_urdf");
        require(robot && robot->instance, "Actual ABB model missing");
        motion_planning::StoredMotionPlan plan;
        plan.robotId = "ABB4600_urdf";
        plan.id = "test_cdf_qp_result";
        plan.jointNames = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
        // Include a turn beyond pi; preserve it while evaluating actual FK.
        for(int i = 0; i < 41; ++i) {
            robottrajectory::TimedJointPoint point;
            point.time = 0.2 * i;
            point.q = {0.01 * i, 0.2, -0.3, 3.2 + i * 0.001, 0.4, -0.5};
            plan.trajectory.points.push_back(point);
        }
        Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
        tool.translation() = Eigen::Vector3d(0.11, -0.23, 1.21);
        tool.linear() = (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(-0.9, Eigen::Vector3d::UnitZ())).toRotationMatrix();
        auto fk = [robot, tool](const std::vector<double>& q) -> Eigen::Isometry3d {
            robot->instance->setJoints(q);
            robot->instance->update();
            return robot->instance->getLinkTransform("Link6") * tool;
        };
        motion_planning::RapidTrajectoryExportOptions options;
        options.mapIrb4600StoredJointSigns = true;
        options.worldTcpForwardKinematics = fk;
        const auto result = motion_planning::ProjectRapidTrajectoryExporter::generate(document, path.parent_path(), plan, options);
        require(result.success, result.error);
        require(result.pointCount == plan.trajectory.points.size(), "All source points must be exported");
        require(result.program.find("MODULE MainModule\r\n") == 0, "Module header/CRLF");
        require(result.program.find("tooldata") == std::string::npos &&
            result.program.find("tCdfSpray") == std::string::npos, "Use the controller built-in tool0 only");
        require(result.program.find("MoveL p41, v50, z10, tool0\\WObj:=wobj0;") != std::string::npos,
            "Last point uses tool0 and explicit world workobject");
        require(result.program.find("NOT an IK branch") != std::string::npos, "Do not invent ABB configuration data");
        const auto output = std::filesystem::current_path() / "rapid_export_roundtrip.mod";
        { std::ofstream file(output, std::ios::binary); file << result.program; require(bool(file), "Write fixture"); }
        motion_planning::TrajectoryImportOptions importOptions;
        importOptions.robotId = plan.robotId;
        const auto imported = motion_planning::ProjectTrajectoryImporter::importFile(output, importOptions);
        require(imported.success, imported.message());
        require(imported.plan.cartesianControlPoints.points.size() == result.pointCount, "RAPID round trip point count");
        for(std::size_t i = 0; i < result.pointCount; ++i) {
            const auto q = motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(
                plan.trajectory.points[i].q);
            const auto expected = fk(q);
            const auto& actual = imported.plan.cartesianControlPoints.points[i].tcpPose;
            require((actual.translation() - expected.translation()).norm() < 1.0e-8, "World TCP mm/sign/turn round trip");
            require(Eigen::AngleAxisd(expected.linear().transpose() * actual.linear()).angle() < 1.0e-8,
                "Quaternion wxyz orientation round trip");
        }
        std::error_code ignored;
        std::filesystem::remove(output, ignored);
        auto invalid = plan;
        invalid.trajectory.points.back().q[1] = std::numeric_limits<double>::quiet_NaN();
        const auto badRow = motion_planning::ProjectRapidTrajectoryExporter::generate(document, path.parent_path(), invalid, options);
        require(!badRow.success && badRow.program.empty(), "Invalid late row must not yield a partial program");
        options.worldTcpForwardKinematics = [fk](const auto& q) -> Eigen::Isometry3d {
            Eigen::Isometry3d pose = fk(q);
            pose.translation().x() += q[0] * 0.01;
            return pose;
        };
        const auto movingTool = motion_planning::ProjectRapidTrajectoryExporter::generate(document, path.parent_path(), plan, options);
        require(!movingTool.success && movingTool.program.empty(), "Reject a TCP that is not rigidly attached to the flange");
        options.worldTcpForwardKinematics = {};
        require(!motion_planning::ProjectRapidTrajectoryExporter::generate(document, path.parent_path(), plan, options).success,
            "Do not fall back to nominal DH when actual TCP is missing");
        std::cout << "RAPID export: actual model, translated/rotated base, tool, units, orientation, signs, turns and invalid input passed.\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
