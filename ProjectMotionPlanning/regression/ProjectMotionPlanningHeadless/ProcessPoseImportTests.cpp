#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <limits>
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

    void verifyTransform(motion_planning::StoredMotionPlan plan,
        const motion_planning::TrajectoryImportOptions& options)
    {
        using Editor = motion_planning::ProjectTrajectoryControlPointEditor;
        const auto original = plan.cartesianControlPoints.points;
        require(!original.empty(), "Transform fixture must have Cartesian points");
        plan.trajectory.points.push_back(robottrajectory::TimedJointPoint{});
        std::string error, text;
        require(Editor::transformCartesianTrajectory(plan, {100, -200, 300}, {90, 90, 0}, &error), error);
        require(plan.trajectory.empty(), "Rigid transform must invalidate old IK");
        for(std::size_t i = 0; i < original.size(); ++i) {
            const auto& a = original[i];
            const auto& b = plan.cartesianControlPoints.points[i];
            const auto& p = a.tcpPose.translation();
            // Rx(90) then Ry(90) maps (x,y,z) to (y,-z,-x).
            require((b.tcpPose.translation() - Eigen::Vector3d(p.y()+.1, -p.z()-.2, -p.x()+.3)).norm() < 1e-10,
                "World origin, fixed axis order and millimeter conversion");
            require(a.time == b.time && a.linearSpeed == b.linearSpeed && a.angularSpeed == b.angularSpeed,
                "Transform retains timestamp and available process speeds");
            require((original[0].tcpPose.inverse() * a.tcpPose).matrix().isApprox(
                (plan.cartesianControlPoints.points[0].tcpPose.inverse() * b.tcpPose).matrix(), 1e-10),
                "The entire path has a single rigid transform, including orientations");
        }
        require(Editor::serializeCartesianMatrices(plan, text, &error), error);
        const auto path = std::filesystem::current_path() / "transformed_matrix_roundtrip.txt";
        { std::ofstream out(path); out << text; }
        const auto imported = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(imported.success && imported.plan.cartesianControlPoints.points.size() == original.size(), "Matrix export round trip");
        for(std::size_t i = 0; i < original.size(); ++i) {
            require(imported.plan.cartesianControlPoints.points[i].tcpPose.matrix().isApprox(
                plan.cartesianControlPoints.points[i].tcpPose.matrix(), 1e-10), "Export retains full transformed pose");
            require(std::abs(imported.plan.cartesianControlPoints.points[i].time - original[i].time) < 1e-12, "Export retains timestamps");
        }
        std::filesystem::remove(path);
        const auto before = plan.cartesianControlPoints.points[0].tcpPose.matrix().eval();
        require(!Editor::transformCartesianTrajectory(plan, {0, 0, 0}, {0, 0, std::numeric_limits<double>::quiet_NaN()}, &error),
            "Reject invalid transform");
        require(before == plan.cartesianControlPoints.points[0].tcpPose.matrix(), "Invalid input cannot partially modify trajectory");
        simulation_project::ProjectDocument document;
        require(motion_planning::MotionPlanningProjectStore::upsertPlan(document, plan, &error), error);
        auto other = plan; other.id += "_other";
        require(motion_planning::MotionPlanningProjectStore::upsertPlan(document, other, &error), error);
        require(motion_planning::MotionPlanningProjectStore::removePlan(document, plan.id, &error), error);
        const auto retained = motion_planning::MotionPlanningProjectStore::plans(document);
        require(retained.size() == 1 && retained[0].id == other.id, "Delete only selected trajectory");
        require(!motion_planning::MotionPlanningProjectStore::removePlan(document, plan.id), "Missing delete fails");
    }

    void verifyJointTcpExport()
    {
        using Editor = motion_planning::ProjectTrajectoryControlPointEditor;
        motion_planning::StoredMotionPlan plan;
        plan.jointNames = {"J1", "J2", "J3", "J4", "J5", "J6"};
        // More rows than the GUI table: export must include every final node and endpoint.
        for(int i = 0; i < 1205; ++i) {
            robottrajectory::TimedJointPoint point;
            point.time = i * .125;
            point.q = {i * .02, .4, -.2, i * .01, -.7, 8.0};
            plan.trajectory.points.push_back(point);
        }
        // Stale source targets must never substitute for the final joint result.
        plan.cartesianControlPoints.points.push_back(robottrajectory::TimedCartesianPoint{});
        const auto fk = [](const std::vector<double>& q) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(q[0] + 1.2, q[3] - .3, q[5] * .1);
            pose.linear() = (Eigen::AngleAxisd(q[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY())).toRotationMatrix();
            return pose;
        };
        std::string text, error;
        const auto path = std::filesystem::current_path() / "final_tcp_matrix_roundtrip.txt";
        motion_planning::TrajectoryImportOptions options;
        options.robotId = "test";
        options.jointNames = plan.jointNames;
        for(bool mapSigns : {false, true}) {
            require(Editor::serializeJointTcpMatrices(plan, fk, mapSigns, text, &error), error);
            { std::ofstream out(path); out << text; }
            const auto imported = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
            require(imported.success && imported.plan.cartesianControlPoints.points.size() == 1205,
                "Export all final joint nodes, not the original control points or sampled table");
            for(std::size_t i = 0; i < 1205; ++i) {
                auto runtime = plan.trajectory.points[i].q;
                if(mapSigns) for(int axis : {0, 3, 4, 5}) runtime[axis] = -runtime[axis];
                const auto& point = imported.plan.cartesianControlPoints.points[i];
                require(point.tcpPose.matrix().isApprox(fk(runtime).matrix(), 1e-12),
                    "Full TCP matrix, millimeters, ABB signs and multi-turn joints survive round trip");
                require(point.time == plan.trajectory.points[i].time, "Keep every recorded final timestamp");
            }
        }
        std::filesystem::remove(path);
        text = "untouched";
        auto invalid = plan;
        invalid.trajectory.points.back().q.pop_back();
        require(!Editor::serializeJointTcpMatrices(invalid, fk, true, text, &error), "Reject invalid late joint row");
        invalid = plan;
        invalid.trajectory.points.back().time = std::numeric_limits<double>::infinity();
        require(!Editor::serializeJointTcpMatrices(invalid, fk, true, text, &error), "Reject nonfinite timestamp");
        invalid = plan;
        invalid.trajectory.points.back().q[0] = std::numeric_limits<double>::quiet_NaN();
        require(!Editor::serializeJointTcpMatrices(invalid, fk, true, text, &error), "Reject nonfinite joint");
        require(!Editor::serializeJointTcpMatrices(plan, {}, true, text, &error), "Require actual-model TCP snapshot");
        require(!Editor::serializeJointTcpMatrices(plan, [](const auto&) -> Eigen::Isometry3d {
            throw std::runtime_error("FK failed");
        }, true, text, &error), "Report FK exception");
        require(!Editor::serializeJointTcpMatrices(plan, [](const auto&) {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear()(0,0) = 2.0;
            return pose;
        }, true, text, &error), "Reject invalid pose");
        require(text == "untouched", "Failed export never publishes partial output");
        require(plan.trajectory.points.back().q[5] == 8.0 && plan.cartesianControlPoints.points.size() == 1,
            "Export leaves source result and legal turns untouched");
        std::cout << "Final TCP export verified: all nodes, full poses, units, signs, turns, timing and atomic failures.\n";
    }

    void verifyLocalRotations()
    {
        using Editor = motion_planning::ProjectTrajectoryControlPointEditor;
        using Axis = motion_planning::CartesianLocalRotationAxis;
        motion_planning::StoredMotionPlan source;
        source.id = "local_rotation";
        for(int i = 0; i < 3; ++i) {
            robottrajectory::TimedCartesianPoint point;
            point.time = .25 * i;
            point.linearSpeed = .4 + .1 * i;
            point.angularSpeed = .05 * i;
            point.tcpPose.translation() = Eigen::Vector3d(1 + .1 * i, -.2, .6);
            point.tcpPose.linear() = (Eigen::AngleAxisd(.4 + .1 * i, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(.7, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-.2, Eigen::Vector3d::UnitX())).toRotationMatrix();
            source.cartesianControlPoints.points.push_back(point);
        }
        source.trajectory.points.push_back(robottrajectory::TimedJointPoint{});
        std::string error;
        for(int axis = 0; axis < 3; ++axis) for(double degrees : {90.0, -90.0, 450.0}) {
            auto result = source;
            require(Editor::rotateCartesianTrajectoryLocalAxis(result, static_cast<Axis>(axis), degrees, &error), error);
            require(result.trajectory.empty(), "Local rotation invalidates stale IK");
            for(std::size_t i = 0; i < source.cartesianControlPoints.points.size(); ++i) {
                const auto& a = source.cartesianControlPoints.points[i];
                const auto& b = result.cartesianControlPoints.points[i];
                const Eigen::Vector3d worldAxis = a.tcpPose.linear().col(axis);
                // Independent geometric construction: rotate in world about the original local axis.
                const Eigen::Matrix3d expected = Eigen::AngleAxisd(degrees * 3.14159265358979323846 / 180.0,
                    worldAxis).toRotationMatrix() * a.tcpPose.linear();
                require(b.tcpPose.linear().isApprox(expected, 1e-12), "Rotation must use each point's own axis");
                require(b.tcpPose.translation() == a.tcpPose.translation(), "Local rotation preserves positions exactly");
                require(b.tcpPose.linear().col(axis).isApprox(worldAxis, 1e-12), "Selected local axis stays fixed");
                require(b.time == a.time && b.linearSpeed == a.linearSpeed && b.angularSpeed == a.angularSpeed,
                    "Local rotation preserves timestamps and stored process speeds");
            }
            require(Editor::rotateCartesianTrajectoryLocalAxis(result, static_cast<Axis>(axis), -degrees, &error), error);
            for(std::size_t i = 0; i < source.cartesianControlPoints.points.size(); ++i) {
                require(result.cartesianControlPoints.points[i].tcpPose.matrix().isApprox(
                    source.cartesianControlPoints.points[i].tcpPose.matrix(), 1e-12), "Inverse local rotation restores every pose");
            }
        }
        auto invalid = source;
        require(!Editor::rotateCartesianTrajectoryLocalAxis(invalid, Axis::Z, std::numeric_limits<double>::infinity(), &error),
            "Reject nonfinite local angle");
        require(!Editor::rotateCartesianTrajectoryLocalAxis(invalid, static_cast<Axis>(9), 90, &error), "Reject invalid axis");
        invalid.cartesianControlPoints.points.back().tcpPose.linear()(0,0) = std::numeric_limits<double>::quiet_NaN();
        const auto first = invalid.cartesianControlPoints.points.front().tcpPose.matrix().eval();
        require(!Editor::rotateCartesianTrajectoryLocalAxis(invalid, Axis::X, 90, &error), "Reject late invalid pose");
        require(invalid.cartesianControlPoints.points.front().tcpPose.matrix() == first && !invalid.trajectory.empty(),
            "Failed local rotation leaves earlier poses and previous IK untouched");
        std::cout << "Local XYZ rotations verified: position/axis invariants, signs, turns, metadata and atomic failure.\n";
    }

    void verifyRows(const std::filesystem::path& path, const motion_planning::TrajectoryImportOptions& options)
    {
        const auto result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
        require(result.success, result.message());
        require(result.dataKind == motion_planning::TrajectoryImportDataKind::CartesianControlPoints &&
            result.plan.trajectory.empty(), "Process rows must not become matrices or joint angles");
        verifyTransform(result.plan, options);
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
        verifyLocalRotations();
        verifyJointTcpExport();
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
        const std::string matrix = "1 0 0 100\n0 1 0 200\n0 0 1 300\n0 0 0 1\n";
        for(const auto& layout : {matrix + "1.25\n\n" + matrix + "4.75\n", "1.25\n" + matrix + "4.75\n" + matrix}) {
            write(layout);
            result = motion_planning::ProjectTrajectoryImporter::importFile(path, options);
            require(result.success && result.plan.cartesianControlPoints.points.size() == 2 &&
                result.plan.cartesianControlPoints.points[0].time == 1.25 && result.plan.cartesianControlPoints.points[1].time == 4.75,
                "Support postfix and prefix matrix timestamps without shifting points");
            verifyTransform(result.plan, options);
        }
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
        if(argc > 2) {
            const auto matrix = motion_planning::ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[2]), options);
            require(matrix.success, matrix.message());
            verifyTransform(matrix.plan, options);
            std::cout << "Matrix transform verified for " << matrix.plan.cartesianControlPoints.points.size() << " poses.\n";
        }
        std::cout << "Process pose import and legacy format regressions passed.\n";
        return 0;
    } catch(const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
