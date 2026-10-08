#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <SimulationProject/ProjectDocument.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace
{
    void require(bool ok, const std::string& message)
    {
        if(!ok) { throw std::runtime_error(message); }
    }

    void verifyRows(const std::filesystem::path& path, const motion_planning::TrajectoryImportOptions& options)
    {
        const auto result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(result.success, result.message());
        require(result.dataKind == motion_planning::TrajectoryImportDataKind::CartesianControlPoints &&
            result.plan.trajectory.empty(), "Process rows must not become matrices or joint angles");
        std::ifstream file(path);
        std::string line;
        std::size_t index = 0;
        while(std::getline(file, line)) {
            if(line.empty() || line.front() == '#') { continue; }
            std::istringstream row(line);
            double v[12];
            for(double& value : v) { require(bool(row >> value), "Fixture row"); }
            require(index < result.plan.cartesianControlPoints.points.size(), "No points lost");
            const auto& actual = result.plan.cartesianControlPoints.points[index++];
            require(actual.time == v[9], "Keep the exact timestamp, including zero");
            require((actual.tcpPose.translation() - Eigen::Vector3d(v[0], v[1], v[2]) * 0.001).norm() < 1e-12,
                "All xyz columns convert mm to meters exactly once");
            const Eigen::Quaterniond rotation(v[3], v[4], v[5], v[6]);
            require((actual.tcpPose.linear() - rotation.normalized().toRotationMatrix()).norm() < 1e-12,
                "Use all quaternion components in wxyz order");
            require(actual.linearSpeed == v[7] * 0.001 && actual.angularSpeed == 0.0,
                "Speed uses m/s; acceleration is not angular speed");
        }
        require(index == result.plan.cartesianControlPoints.points.size(), "One Cartesian point per row");
        simulation_project::ProjectDocument document;
        std::string error;
        require(motion_planning::MotionPlanningProjectStore::upsertPlan(document, result.plan, &error), error);
        const auto stored = motion_planning::MotionPlanningProjectStore::plans(document);
        require(stored.size() == 1 && stored.front().cartesianControlPoints.points.size() == index,
            "Cartesian data survives project storage");
        for(std::size_t i = 0; i < index; ++i) {
            const auto& a = stored.front().cartesianControlPoints.points[i];
            const auto& b = result.plan.cartesianControlPoints.points[i];
            require(a.time == b.time && a.linearSpeed == b.linearSpeed && a.tcpPose.matrix() == b.tcpPose.matrix(),
                "Stored pose, time and speed unchanged");
        }
        std::cout << "Verified every field used by Basic Planning for " << index << " process points.\n";
    }
}

int main(int argc, char** argv)
{
    try {
        motion_planning::TrajectoryImportOptions options;
        options.robotId = "ABB4600_urdf";
        options.jointNames = {"Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"};
        const auto path = std::filesystem::current_path() / "process_pose_import_test.txt";
        const std::string rows =
            "-197.493 6.531 -100.000 0.506 0.506 -0.494 -0.494 400.000 0.000 0.000000 0 40.000\n"
            "-197.480 5.200 -0.000 -0.498 -0.498 0.502 0.502 400.000 0.000 0.000000 0 40.000\n"
            "-197.478 5.566 89.960 0.500 0.500 -0.500 -0.500 400.000 0.000 0.474925 0 40.000\n"
            "-197.478 5.566 89.960 0.500 0.500 -0.500 -0.500 400.000 0.000 0.600000 0 40.000\n";
        auto write = [&](const std::string& text) { std::ofstream file(path); file << text; };
        write(rows);
        verifyRows(path, options);
        for(const std::string bad : {
            "0 0 0 0 0 0 0 400 0 1 0 40\n", "0 0 0 nan 0 0 1 400 0 1 0 40\n",
            "0 0 0 1 0 0 0 400 0 0.1 0 40\n", "0 0 0 1 0 0 0 400 0 1 0\n"}) {
            write(rows + bad);
            require(!motion_planning::ProjectTrajectoryImporter::importFile(path, options).success,
                "Reject invalid late row without partial output/fallback");
        }
        write("1 0 0 100\n0 1 0 200\n0 0 1 300\n0 0 0 1\n");
        auto result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(result.success && result.plan.cartesianControlPoints.points.size() == 1 &&
            (result.plan.cartesianControlPoints.points[0].tcpPose.translation() - Eigen::Vector3d(.1,.2,.3)).norm() < 1e-12,
            "Retain raw 4x4 matrix support");
        write("100 200 300 1 0 0 0 0\n100 200 300 1 0 0 0 0.2\n");
        result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(result.success && result.plan.cartesianControlPoints.points.size() == 2, "Retain 8-column poses");
        write("0 0.1 0.2 0.3 0.4 0.5 0.6\n1 0.2 0.3 0.4 0.5 0.6 0.7\n");
        result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(result.success && result.plan.trajectory.points.size() == 2, "Retain six-axis joint tables");
        options.jointNames.resize(12);
        for(int i = 0; i < 12; ++i) { options.jointNames[i] = "joint" + std::to_string(i); }
        write(rows);
        result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(result.success && result.dataKind == motion_planning::TrajectoryImportDataKind::JointTrajectory,
            "Explicit 12-axis joint layout still takes precedence");
        std::filesystem::remove(path);
        options.jointNames.resize(6);
        if(argc > 1) { verifyRows(std::filesystem::u8path(argv[1]), options); }
        std::cout << "Process pose import and legacy format regressions passed.\n";
        return 0;
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
