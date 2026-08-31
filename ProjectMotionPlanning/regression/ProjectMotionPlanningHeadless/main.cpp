#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <ProjectMotionPlanning/CdfJointAngleImport.h>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>

#include <MotionPlanningCore/MotionPlanning.h>
#include <RobotRuntime/RobotTrajectoryExecutionSession.h>
#include <SimulationProject/ProjectIo.h>

#include <Eigen/Geometry>

#include <data_path.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    struct Options
    {
        std::filesystem::path projectPath = std::filesystem::path(PROJECT_SOURCE_PATH) /
            "config/projects/420-red4600-tool.sys.json";
        std::filesystem::path cdfFilePath;
        std::string robotId = "ABB4600_urdf";
        bool discover = false;
        bool importOnly = false;
        bool cdfRepair = false;
    };

    bool parseOptions(int argc, char** argv, Options& options)
    {
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--project" && index + 1 < argc)
                options.projectPath = std::filesystem::u8path(argv[++index]);
            else if (argument == "--discover")
                options.discover = true;
            else if (argument == "--import-only")
                options.importOnly = true;
            else if (argument == "--cdf-file" && index + 1 < argc)
            {
                options.cdfRepair = true;
                options.cdfFilePath = std::filesystem::u8path(argv[++index]);
            }
            else if (argument == "--robot" && index + 1 < argc)
            {
                options.robotId = argv[++index];
            }
            else
            {
                std::cerr << "Unknown or incomplete option: " << argument << "\n";
                return false;
            }
        }
        return true;
    }

    std::string vectorText(const std::vector<double>& values)
    {
        std::ostringstream stream;
        stream << std::setprecision(15);
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            if (index != 0)
                stream << ',';
            stream << values[index];
        }
        return stream.str();
    }

    std::vector<double> degreesToRadians(const std::vector<double>& degrees)
    {
        std::vector<double> radians;
        radians.reserve(degrees.size());
        for(const double value : degrees) {
            radians.push_back(value * kPi / 180.0);
        }
        return radians;
    }

    struct Candidate
    {
        std::vector<double> joints;
        Eigen::Vector3d toolPosition = Eigen::Vector3d::Zero();
    };

    bool findFixture(
        motion_planning::ProjectPlanningSceneSnapshot& scene,
        motion_planning::ProjectPlanningRequest& request,
        std::vector<double>& start,
        std::vector<double>& goal)
    {
        const auto* positioner = scene.simulationRuntime().robot("ATPPZ350");
        if (positioner == nullptr)
            return false;
        const Eigen::Vector3d center = positioner->baseTransform.translation();

        std::mt19937 generator(4204600u);
        std::vector<std::uniform_real_distribution<double>> distributions;
        for (const motion_planning::JointBound& bound : scene.jointBounds())
        {
            const double margin = 0.04 * (bound.upper - bound.lower);
            distributions.emplace_back(bound.lower + margin, bound.upper - margin);
        }

        std::vector<Candidate> negativeSide;
        std::vector<Candidate> positiveSide;
        for (std::size_t sampleIndex = 0; sampleIndex < 6000; ++sampleIndex)
        {
            Candidate candidate;
            candidate.joints.resize(distributions.size());
            for (std::size_t jointIndex = 0; jointIndex < distributions.size(); ++jointIndex)
                candidate.joints[jointIndex] = distributions[jointIndex](generator);

            if (!scene.validateState(candidate.joints).valid)
                continue;
            candidate.toolPosition = scene.attachmentWorldTransform("420_tool_attachment").translation();

            const Eigen::Vector3d relative = candidate.toolPosition - center;
            if (std::abs(relative.x()) > 1.25 || candidate.toolPosition.z() < 0.05 || candidate.toolPosition.z() > 1.9)
                continue;

            std::vector<Candidate>* ownSide = nullptr;
            std::vector<Candidate>* oppositeSide = nullptr;
            if (relative.y() < -0.18)
            {
                ownSide = &negativeSide;
                oppositeSide = &positiveSide;
            }
            else if (relative.y() > 0.18)
            {
                ownSide = &positiveSide;
                oppositeSide = &negativeSide;
            }
            else
            {
                continue;
            }

            for (const Candidate& opposite : *oppositeSide)
            {
                const std::vector<double>& first = relative.y() < 0.0 ? candidate.joints : opposite.joints;
                const std::vector<double>& second = relative.y() < 0.0 ? opposite.joints : candidate.joints;
                if (scene.validateMotion(first, second, request.validation).valid)
                    continue;

                request.start = first;
                request.goal = second;
                request.planner.timeoutSeconds = 3.0;
                const motion_planning::ProjectMotionPlanningService service;
                const motion_planning::MotionPlanningResult planning = service.plan(scene, request);
                if (planning.succeeded())
                {
                    start = first;
                    goal = second;
                    return true;
                }
            }

            if (ownSide->size() < 80)
                ownSide->push_back(std::move(candidate));
        }
        return false;
    }

    bool nearlyEqual(const std::vector<double>& first, const std::vector<double>& second)
    {
        if (first.size() != second.size())
            return false;
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            if (std::abs(first[index] - second[index]) > 1.0e-8)
                return false;
        }
        return true;
    }

    bool sameTrajectory(
        const robottrajectory::JointTrajectory& first,
        const robottrajectory::JointTrajectory& second)
    {
        if (first.name != second.name ||
            first.interpolation != second.interpolation ||
            first.points.size() != second.points.size())
            return false;
        for (std::size_t index = 0; index < first.points.size(); ++index)
        {
            if (std::abs(first.points[index].time - second.points[index].time) > 1.0e-12 ||
                !nearlyEqual(first.points[index].q, second.points[index].q))
                return false;
        }
        return true;
    }

    int fail(const std::string& message)
    {
        std::cerr << "FAILED: " << message << "\n";
        return 1;
    }

    Eigen::Isometry3d irb4600ZeroFlangePose()
    {
        Eigen::Matrix4d matrix;
        matrix <<
            -6.12323399573676e-17, -6.12323399573677e-17, 1.0, 1.27,
            -6.12323399573677e-17, 1.0, 6.12323399573677e-17, 1.22464679914735e-18,
            -1.0, -6.12323399573677e-17, -6.12323399573677e-17, 1.57,
            0.0, 0.0, 0.0, 1.0;

        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.matrix() = matrix;
        return pose;
    }

    int verifyIrb4600InverseKinematicsRegression()
    {
        simulation_project::ProjectDocument document;
        simulation_project::RobotDesc robot;
        robot.id = "ABB4600_urdf";
        robot.name = "ABB4600_urdf";
        robot.sourceType = "urdf";
        robot.sourcePath = "data/Spray420/ABB4600_urdf/urdf/ABB4600_urdf.urdf";
        document.robots.push_back(std::move(robot));

        motion_planning::StoredMotionPlan plan;
        plan.id = "irb4600_ik_regression";
        plan.name = "IRB4600 IK regression";
        plan.robotId = "ABB4600_urdf";
        robottrajectory::TimedCartesianPoint point;
        point.time = 0.0;
        point.tcpPose = irb4600ZeroFlangePose();
        plan.cartesianControlPoints.points.push_back(std::move(point));

        motion_planning::CartesianIkOptions options;
        options.robotId = plan.robotId;
        options.jointNames = motion_planning::ProjectTrajectoryInverseKinematics::defaultIrb4600JointNames();
        options.seedJoints = std::vector<double>(6, 0.0);
        options.toolMode = motion_planning::CartesianIkToolMode::Flange;
        options.tolerance = 1.0e-8;

        const motion_planning::CartesianIkResult result =
            motion_planning::ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(
                document,
                plan,
                options);
        if(!result.success || result.plan.trajectory.points.size() != 1)
            return fail("IRB4600 inverse kinematics regression failed.");
        if(result.plan.jointNames != options.jointNames)
            return fail("IRB4600 inverse kinematics joint names were not preserved.");
        for(double joint : result.plan.trajectory.points.front().q) {
            if(std::abs(joint) > 1.0e-7)
                return fail("IRB4600 inverse kinematics zero-pose solution drifted.");
        }

        const std::vector<double> mappedJoints =
            motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(
                { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 });
        if(mappedJoints != std::vector<double>({ -1.0, 2.0, 3.0, -4.0, -5.0, -6.0 }))
            return fail("IRB4600 robot system joint sign mapping changed.");

        std::string errorMessage;
        if(!motion_planning::ProjectTrajectoryInverseKinematics::applyJointValuesToRobotInitialJoints(
                document,
                plan.robotId,
                result.plan.jointNames,
                result.plan.trajectory.points.front().q,
                &errorMessage))
            return fail("IRB4600 inverse kinematics document joint application failed: " + errorMessage);
        if(document.robots.front().initialJoints.size() != 6)
            return fail("IRB4600 inverse kinematics did not write robot initial joints.");
        return 0;
    }

    int verifyTrajectoryControlPointEditingRegression()
    {
        motion_planning::StoredMotionPlan plan;
        plan.id = "control_point_editing_regression";
        plan.name = "Control point editing regression";
        plan.robotId = "ABB4600_urdf";
        plan.jointNames = motion_planning::ProjectTrajectoryInverseKinematics::defaultIrb4600JointNames();

        robottrajectory::TimedCartesianPoint firstPoint;
        firstPoint.time = 0.0;
        firstPoint.tcpPose = Eigen::Isometry3d::Identity();
        firstPoint.tcpPose.translation().x() = 0.0;

        robottrajectory::TimedCartesianPoint secondPoint = firstPoint;
        secondPoint.time = 2.0;
        secondPoint.tcpPose.translation().x() = 2.0;

        plan.cartesianControlPoints.points.push_back(firstPoint);
        plan.cartesianControlPoints.points.push_back(secondPoint);

        robottrajectory::TimedJointPoint solvedPoint;
        solvedPoint.time = 0.0;
        solvedPoint.q = std::vector<double>(6, 0.0);
        plan.trajectory.points.push_back(solvedPoint);

        robottrajectory::TimedCartesianPoint insertedPoint;
        std::string errorMessage;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::makeInsertedCartesianControlPoint(
               plan,
               0,
               motion_planning::CartesianControlPointInsertLocation::After,
               insertedPoint,
               &errorMessage))
            return fail("Control point insertion seed failed: " + errorMessage);
        if(std::abs(insertedPoint.time - 1.0) > 1.0e-12)
            return fail("Control point insertion seed did not choose midpoint time.");

        insertedPoint.tcpPose.translation().x() = 1.0;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::insertCartesianControlPoint(
               plan,
               0,
               motion_planning::CartesianControlPointInsertLocation::After,
               insertedPoint,
               &errorMessage))
            return fail("Control point insertion failed: " + errorMessage);
        if(plan.cartesianControlPoints.points.size() != 3 ||
            std::abs(plan.cartesianControlPoints.points[1].time - 1.0) > 1.0e-12 ||
            !plan.trajectory.empty())
            return fail("Control point insertion changed the wrong data.");

        plan.trajectory.points.push_back(solvedPoint);
        robottrajectory::TimedCartesianPoint updatedPoint = plan.cartesianControlPoints.points[1];
        updatedPoint.time = 0.5;
        updatedPoint.tcpPose.translation().y() = 0.25;
        if(!motion_planning::ProjectTrajectoryControlPointEditor::updateCartesianControlPoint(
               plan,
               1,
               updatedPoint,
               &errorMessage))
            return fail("Control point update failed: " + errorMessage);
        if(plan.cartesianControlPoints.points.size() != 3 ||
            std::abs(plan.cartesianControlPoints.points[1].time - 0.5) > 1.0e-12 ||
            std::abs(plan.cartesianControlPoints.points[1].tcpPose.translation().y() - 0.25) > 1.0e-12 ||
            !plan.trajectory.empty())
            return fail("Control point update did not persist edited pose or invalidate IK.");

        plan.trajectory.points.push_back(solvedPoint);
        if(!motion_planning::ProjectTrajectoryControlPointEditor::removeCartesianControlPoint(
               plan,
               1,
               &errorMessage))
            return fail("Control point delete failed: " + errorMessage);
        if(plan.cartesianControlPoints.points.size() != 2 || !plan.trajectory.empty())
            return fail("Control point delete did not remove one pose and invalidate IK.");

        if(!motion_planning::ProjectTrajectoryControlPointEditor::removeCartesianControlPoint(
               plan,
               0,
               &errorMessage))
            return fail("Control point delete down to one failed: " + errorMessage);
        if(motion_planning::ProjectTrajectoryControlPointEditor::removeCartesianControlPoint(
               plan,
               0,
               &errorMessage))
            return fail("Control point delete allowed removing the last pose.");

        return 0;
    }

    int verifyCdfJointAngleImportRegression()
    {
        const std::filesystem::path importedCdfPath =
            std::filesystem::current_path() / "import_only_cdf_joint_angles.txt";
        {
            std::ofstream text(importedCdfPath);
            text << "# IK joint angle export\n";
            text << "# Time unit: seconds\n";
            text << "# Joint angle unit: degrees\n";
            text << "time_s\tJ1_deg\tJ2_deg\tJ3_deg\tJ4_deg\tJ5_deg\tJ6_deg\n";
            text << "0.000000\t102.522987\t-8.415644\t43.664456\t-83.828243\t101.005853\t155.247243\n";
            text << "0.998620\t99.516875\t-9.246378\t44.016453\t-85.484835\t98.466115\t154.565709\n";
        }

        const motion_planning::CdfJointAngleImportResult importResult =
            motion_planning::ProjectCdfJointAngleImporter::importFile(importedCdfPath);
        std::error_code removeImportedCdfError;
        std::filesystem::remove(importedCdfPath, removeImportedCdfError);

        if(!importResult.success ||
            importResult.points.size() != 2 ||
            importResult.jointNames.size() != 6 ||
            importResult.jointNames.front() != "J1")
            return fail("CDF joint angle import failed: " + importResult.message());
        if(std::abs(importResult.points.front().timeSeconds) > 1.0e-12 ||
            std::abs(importResult.points.back().timeSeconds - 0.998620) > 1.0e-12 ||
            std::abs(importResult.points.front().jointAnglesDegrees.front() - 102.522987) > 1.0e-12 ||
            std::abs(importResult.points.front().jointAnglesDegrees[3] + 83.828243) > 1.0e-12)
            return fail("CDF joint angle import parsed the wrong values.");

        const std::filesystem::path importedRadCdfPath =
            std::filesystem::current_path() / "import_only_cdf_joint_angles_rad.txt";
        {
            std::ofstream text(importedRadCdfPath);
            text << "# Joint angle unit: radians\n";
            text << "time_s J1_rad J2_rad\n";
            text << "0 3.14159265358979323846 1.57079632679489661923\n";
        }

        const motion_planning::CdfJointAngleImportResult radImport =
            motion_planning::ProjectCdfJointAngleImporter::importFile(importedRadCdfPath);
        std::error_code removeImportedRadCdfError;
        std::filesystem::remove(importedRadCdfPath, removeImportedRadCdfError);

        if(!radImport.success ||
            radImport.points.size() != 1 ||
            std::abs(radImport.points.front().jointAnglesDegrees[0] - 180.0) > 1.0e-9 ||
            std::abs(radImport.points.front().jointAnglesDegrees[1] - 90.0) > 1.0e-9)
            return fail("CDF radian joint angle import did not convert to degrees.");

        return 0;
    }

    int runImportOnly(const Options& options)
    {
        simulation_project::ProjectDocument document;
        std::string errorMessage;
        if (!simulation_project::loadProjectDocument(options.projectPath, document, &errorMessage))
            return fail("Project load failed: " + errorMessage);

        motion_planning::TrajectoryImportOptions importOptions;
        importOptions.robotId = "Red4600";
        importOptions.jointNames = { "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6" };

        const std::filesystem::path importedCsvPath =
            std::filesystem::current_path() / "import_only_motion_plan.csv";
        {
            std::ofstream csv(importedCsvPath);
            csv << "time";
            for (const std::string& jointName : importOptions.jointNames)
                csv << "," << jointName;
            csv << "\n";
            csv << "0,0.1,0.2,0.3,0.4,0.5,0.6\n";
            csv << "1,0.2,0.3,0.4,0.5,0.6,0.7\n";
        }
        const motion_planning::TrajectoryImportResult jointImport =
            motion_planning::ProjectTrajectoryImporter::importFile(importedCsvPath, importOptions);
        std::error_code removeImportedCsvError;
        std::filesystem::remove(importedCsvPath, removeImportedCsvError);
        if (!jointImport.success ||
            jointImport.dataKind != motion_planning::TrajectoryImportDataKind::JointTrajectory ||
            jointImport.plan.trajectory.points.size() != 2 ||
            !jointImport.plan.cartesianControlPoints.empty())
            return fail("Import-only CSV joint trajectory import failed: " + jointImport.message());
        if (!motion_planning::MotionPlanningProjectStore::upsertPlan(document, jointImport.plan, &errorMessage))
            return fail("Import-only joint trajectory store failed: " + errorMessage);

        const std::filesystem::path importedCartesianPath =
            std::filesystem::current_path() / "import_only_cartesian_points.txt";
        {
            std::ofstream text(importedCartesianPath);
            text << "=== Point 1 ===\n";
            text << "Pose (4x4 matrix):\n";
            text << "1 0 0 1,000\n";
            text << "0 1 0 0\n";
            text << "0 0 1 50\n";
            text << "0 0 0 1\n";
            text << "=== Point 2 ===\n";
            text << "Pose (4x4 matrix):\n";
            text << "1 0 0 120\n";
            text << "0 1 0 5\n";
            text << "0 0 1 55\n";
            text << "0 0 0 1\n";
        }
        const motion_planning::TrajectoryImportResult cartesianImport =
            motion_planning::ProjectTrajectoryImporter::importFile(importedCartesianPath, importOptions);
        std::error_code removeImportedCartesianError;
        std::filesystem::remove(importedCartesianPath, removeImportedCartesianError);
        const auto& importedCartesianPoints = cartesianImport.plan.cartesianControlPoints.points;
        if (!cartesianImport.success ||
            cartesianImport.dataKind != motion_planning::TrajectoryImportDataKind::CartesianControlPoints ||
            importedCartesianPoints.size() != 2 ||
            !cartesianImport.plan.trajectory.empty())
            return fail("Import-only cartesian control-point import failed: " + cartesianImport.message());
        if (std::abs(importedCartesianPoints.front().time) > 1.0e-9 ||
            std::abs(importedCartesianPoints.back().time - 5.0) > 1.0e-9 ||
            std::abs(importedCartesianPoints.front().tcpPose.translation().x() - 1.0) > 1.0e-9 ||
            std::abs(importedCartesianPoints.front().tcpPose.translation().z() - 0.05) > 1.0e-9 ||
            std::abs(importedCartesianPoints.back().tcpPose.translation().x() - 0.12) > 1.0e-9)
            return fail("Import-only cartesian control-point TXT parsing mismatch.");
        if (!motion_planning::MotionPlanningProjectStore::upsertPlan(document, cartesianImport.plan, &errorMessage))
            return fail("Import-only cartesian control-point store failed: " + errorMessage);

        const std::filesystem::path roundTripPath = std::filesystem::current_path() /
            "import_only_motion_planning_roundtrip.sys.json";
        if (!simulation_project::saveProjectDocumentV3(roundTripPath, document, &errorMessage))
            return fail("Import-only project save failed: " + errorMessage);
        simulation_project::ProjectDocument reloaded;
        if (!simulation_project::loadProjectDocument(roundTripPath, reloaded, &errorMessage))
            return fail("Import-only project reload failed: " + errorMessage);
        std::error_code removeRoundTripError;
        std::filesystem::remove(roundTripPath, removeRoundTripError);

        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(reloaded);
        const auto importedJointIt = std::find_if(
            plans.begin(),
            plans.end(),
            [&](const motion_planning::StoredMotionPlan& plan) {
                return plan.id == jointImport.plan.id;
            });
        if (importedJointIt == plans.end() || importedJointIt->trajectory.points.size() != 2)
            return fail("Import-only joint trajectory did not survive project round-trip.");
        const auto importedCartesianIt = std::find_if(
            plans.begin(),
            plans.end(),
            [&](const motion_planning::StoredMotionPlan& plan) {
                return plan.id == cartesianImport.plan.id;
            });
        if (importedCartesianIt == plans.end() ||
            importedCartesianIt->cartesianControlPoints.points.size() != 2 ||
            !importedCartesianIt->trajectory.empty())
            return fail("Import-only cartesian control points did not survive project round-trip.");
        if (std::abs(importedCartesianIt->cartesianControlPoints.points.back().time - 5.0) > 1.0e-9 ||
            std::abs(importedCartesianIt->cartesianControlPoints.points.front().tcpPose.translation().x() - 1.0) > 1.0e-9)
            return fail("Import-only cartesian control-point values changed during project round-trip.");

        if(const int ikRegression = verifyIrb4600InverseKinematicsRegression(); ikRegression != 0)
            return ikRegression;
        if(const int editRegression = verifyTrajectoryControlPointEditingRegression(); editRegression != 0)
            return editRegression;
        if(const int cdfRegression = verifyCdfJointAngleImportRegression(); cdfRegression != 0)
            return cdfRegression;

        std::cout << "PASS ProjectMotionPlanning import-only regression\n";
        return 0;
    }

    std::vector<double> maybeMapIrb4600JointSigns(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const std::vector<double>& joints)
    {
        if(robotId.find("4600") == std::string::npos) {
            return joints;
        }

        const auto robotIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId;
            });
        if(robotIt == document.robots.end()) {
            return joints;
        }

        if(robotIt->sourcePath.find("4600") == std::string::npos &&
            robotIt->sourcePath.find("ABB4600") == std::string::npos &&
            robotIt->id.find("4600") == std::string::npos)
        {
            return joints;
        }

        return motion_planning::ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(joints);
    }

    int runCdfRepair(const Options& options)
    {
        simulation_project::ProjectDocument document;
        std::string errorMessage;
        if(!simulation_project::loadProjectDocument(options.projectPath, document, &errorMessage)) {
            return fail("Project load failed: " + errorMessage);
        }
        if(options.cdfFilePath.empty()) {
            return fail("CDF repair requires --cdf-file.");
        }

        const motion_planning::CdfJointAngleImportResult importResult =
            motion_planning::ProjectCdfJointAngleImporter::importFile(options.cdfFilePath);
        if(!importResult.success) {
            return fail("CDF joint angle import failed: " + importResult.message());
        }
        if(importResult.points.size() < 2) {
            return fail("CDF joint angle file must contain at least two points.");
        }

        std::vector<std::string> jointNames;
        if(options.robotId.find("4600") != std::string::npos) {
            jointNames = motion_planning::ProjectTrajectoryInverseKinematics::defaultIrb4600JointNames();
        } else {
            jointNames = importResult.jointNames;
        }
        if(jointNames.empty()) {
            jointNames = importResult.jointNames;
        }

        motion_planning::ProjectCdfQpRepairOptions repairOptions;
        repairOptions.detectorId = "cdf_abb4600_burnner_detector";
        repairOptions.obstacleId = "burnner";
        repairOptions.obstacleName = "burnner";
        repairOptions.obstacleRobotSourcePath = "data/drake_models/burnner/urdf/burnner.urdf";

        const std::vector<std::string> robotJointNames = jointNames;
        motion_planning::ProjectCdfQpTrajectoryRepairService repairService;

        std::vector<robottrajectory::TimedJointPoint> seedPoints;
        seedPoints.reserve(importResult.points.size());
        for(const motion_planning::CdfJointAnglePoint& importedPoint : importResult.points) {
            robottrajectory::TimedJointPoint point;
            point.time = importedPoint.timeSeconds;
            point.q = maybeMapIrb4600JointSigns(
                document,
                options.robotId,
                degreesToRadians(importedPoint.jointAnglesDegrees));
            seedPoints.push_back(std::move(point));
        }

        robottrajectory::JointTrajectory seedTrajectory;
        seedTrajectory.name = "cdf_import_seed";
        seedTrajectory.interpolation = robottrajectory::TrajectoryInterpolation::Linear;
        seedTrajectory.points = std::move(seedPoints);
        seedTrajectory.sortByTime();

        const std::string robotId = options.robotId;
        const std::filesystem::path projectBase = options.projectPath.parent_path();

        std::string setupDetectorId;
        std::vector<motion_planning::MotionPlanningDiagnostic> setupDiagnostics;
        if(!motion_planning::ProjectCdfQpTrajectoryRepairService::ensureCollisionSetup(
               document,
               robotId,
               repairOptions,
               &setupDetectorId,
               &setupDiagnostics))
        {
            return fail(setupDiagnostics.empty()
                ? "Failed to set up ABB4600_urdf-burnner collision detector."
                : setupDiagnostics.front().message);
        }

        const motion_planning::ProjectCdfQpRepairResult repairResult =
            repairService.repair(
                document,
                projectBase,
                robotId,
                robotJointNames,
                seedTrajectory,
                repairOptions);

        std::cout << "CDF repair points: " << repairResult.plan.trajectory.points.size() << "\n";
        std::cout << "success: " << (repairResult.success ? "yes" : "no") << "\n";
        std::cout << "min phi: " << repairResult.statistics.initialMinimumPhi
                  << " -> " << repairResult.statistics.finalMinimumPhi << "\n";
        std::cout << "invalid segments: " << repairResult.statistics.invalidSegmentCount << "\n";
        std::cout << "max slack: " << repairResult.statistics.maximumSlack << "\n";
        std::cout << "max correction: " << repairResult.statistics.maximumCorrection << "\n";
        for(const motion_planning::MotionPlanningDiagnostic& diagnostic : repairResult.diagnostics) {
            std::cout << diagnostic.code << ": " << diagnostic.message << "\n";
        }

        return repairResult.success ? 0 : 1;
    }
}

int main(int argc, char** argv)
{
    Options options;
    if (!parseOptions(argc, argv, options))
        return 2;
    if (options.cdfRepair)
        return runCdfRepair(options);
    if (options.importOnly)
        return runImportOnly(options);

    simulation_project::ProjectDocument document;
    std::string errorMessage;
    if (!simulation_project::loadProjectDocument(options.projectPath, document, &errorMessage))
        return fail("Project load failed: " + errorMessage);

    motion_planning::ProjectPlanningRequest request;
    request.robotId = "Red4600";
    request.collisionDetectorIds = { "default_collision" };
    request.planner.randomSeed = 4204600u;
    request.planner.timeoutSeconds = 8.0;
    request.validation.maxJointStep = 0.04;
    request.postProcess.duration = 6.0;
    request.postProcess.minimumWaypointCount = 60;

    motion_planning::ProjectPlanningRequest invalidDetectorRequest = request;
    invalidDetectorRequest.collisionDetectorIds = { "missing_detector" };
    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> invalidDetectorScene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            document,
            options.projectPath.parent_path(),
            invalidDetectorRequest,
            &errorMessage);
    if (invalidDetectorScene || errorMessage.find("not found") == std::string::npos)
        return fail("Missing collision detector was not rejected.");

    simulation_project::ProjectDocument disabledDetectorDocument = document;
    const auto disabledDetectorIt = std::find_if(
        disabledDetectorDocument.collision.detectors.begin(),
        disabledDetectorDocument.collision.detectors.end(),
        [](const simulation_project::CollisionDetectorDesc& detector) {
            return detector.id == "default_collision";
        });
    if (disabledDetectorIt == disabledDetectorDocument.collision.detectors.end())
        return fail("Default collision detector is missing from the fixture project.");
    disabledDetectorIt->enabled = false;
    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> disabledDetectorScene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            disabledDetectorDocument,
            options.projectPath.parent_path(),
            request,
            &errorMessage);
    if (disabledDetectorScene || errorMessage.find("disabled") == std::string::npos)
        return fail("Disabled collision detector was not rejected.");

    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> scene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            document,
            options.projectPath.parent_path(),
            request,
            &errorMessage);
    if (!scene)
        return fail("Planning scene build failed: " + errorMessage);

    request.jointNames = scene->jointNames();
    std::vector<double> start = {
        0.659405553148032,
        -0.919209792140321,
        -0.844791768088929,
        -4.8446095729532,
        -1.72245218831655,
        2.56813378610512
    };
    std::vector<double> goal = {
        -0.934876835266256,
        -0.633262433922526,
        1.11553781428474,
        -5.91669352284723,
        1.09495329977677,
        5.42211279330075
    };
    bool fixtureReady = start.size() == scene->jointNames().size() &&
        scene->validateState(start).valid &&
        scene->validateState(goal).valid &&
        !scene->validateMotion(start, goal, request.validation).valid;

    if (options.discover || !fixtureReady)
    {
        if (!findFixture(*scene, request, start, goal))
            return fail("Could not discover a stable turntable-left/right fixture.");
        std::cout << "DISCOVERED_START=" << vectorText(start) << "\n";
        std::cout << "DISCOVERED_GOAL=" << vectorText(goal) << "\n";
    }

    request.start = start;
    request.goal = goal;
    if (scene->validateState(start).valid == false)
        return fail("Start state is invalid.");
    const Eigen::Vector3d startTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
    if (scene->validateState(goal).valid == false)
        return fail("Goal state is invalid.");
    const Eigen::Vector3d goalTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
    if (startTool.y() >= goalTool.y())
        return fail("Fixture does not place the tool on ordered opposite sides of the turntable.");
    if (scene->validateMotion(start, goal, request.validation).valid)
        return fail("Direct joint interpolation is collision-free; the fixture does not prove obstacle avoidance.");

    std::vector<double> collidingState;
    for (std::size_t step = 1; step < 100 && collidingState.empty(); ++step)
    {
        const double ratio = static_cast<double>(step) / 100.0;
        std::vector<double> sample(start.size(), 0.0);
        for (std::size_t index = 0; index < sample.size(); ++index)
            sample[index] = start[index] + (goal[index] - start[index]) * ratio;
        if (!scene->validateState(sample).valid)
            collidingState = std::move(sample);
    }
    if (collidingState.empty())
        return fail("Could not locate the collision state reported by direct interpolation.");

    const motion_planning::ProjectMotionPlanningService service;

    motion_planning::ProjectPlanningRequest invalidStartRequest = request;
    invalidStartRequest.start = collidingState;
    invalidStartRequest.goal = goal;
    const motion_planning::MotionPlanningResult invalidStartResult = service.plan(*scene, invalidStartRequest);
    if (invalidStartResult.status != motion_planning::MotionPlanningStatus::InvalidRequest)
        return fail("Colliding start state was not rejected.");

    motion_planning::CancellationToken cancellation;
    cancellation.cancel();
    const motion_planning::MotionPlanningResult cancelledResult = service.plan(*scene, request, &cancellation);
    if (cancelledResult.status != motion_planning::MotionPlanningStatus::Cancelled)
        return fail("Pre-cancelled planning request did not return Cancelled.");

    const motion_planning::MotionPlanningResult result = service.plan(*scene, request);
    if (!result.succeeded())
    {
        const std::string detail = result.diagnostics.empty() ? "unknown planning error" : result.diagnostics.front().message;
        return fail("OMPL planning failed: " + detail);
    }
    if (result.trajectory.points.size() < 2)
        return fail("Planner returned an empty trajectory.");

    const motion_planning::MotionPlanningResult repeatedResult = service.plan(*scene, request);
    if (!repeatedResult.succeeded() || !sameTrajectory(result.trajectory, repeatedResult.trajectory))
        return fail("Fixed-seed planning did not reproduce the same trajectory.");

    for (std::size_t index = 1; index < result.trajectory.points.size(); ++index)
    {
        if (!scene->validateMotion(
                result.trajectory.points[index - 1].q,
                result.trajectory.points[index].q,
                request.validation).valid)
            return fail("Final trajectory segment revalidation failed.");
    }

    simulation_project::ProjectDocument roundTripDocument = document;
    motion_planning::StoredMotionPlan storedPlan;
    storedPlan.id = "ompl_headless_regression";
    storedPlan.name = "OMPL headless regression";
    storedPlan.robotId = request.robotId;
    storedPlan.jointNames = request.jointNames;
    storedPlan.trajectory = result.trajectory;
    if (!motion_planning::MotionPlanningProjectStore::upsertPlan(roundTripDocument, storedPlan, &errorMessage))
        return fail("Trajectory store failed: " + errorMessage);

    motion_planning::TrajectoryImportOptions importOptions;
    importOptions.robotId = request.robotId;
    importOptions.jointNames = request.jointNames;

    const std::filesystem::path importedCsvPath =
        std::filesystem::current_path() / "imported_motion_plan.csv";
    {
        std::ofstream csv(importedCsvPath);
        csv << "time";
        for (const std::string& jointName : request.jointNames)
            csv << "," << jointName;
        csv << "\n";
        for (std::size_t index = 0; index < std::min<std::size_t>(3, result.trajectory.points.size()); ++index)
        {
            const robottrajectory::TimedJointPoint& point = result.trajectory.points[index];
            csv << point.time;
            for (double value : point.q)
                csv << "," << value;
            csv << "\n";
        }
    }
    const motion_planning::TrajectoryImportResult jointImport =
        motion_planning::ProjectTrajectoryImporter::importFile(importedCsvPath, importOptions);
    std::error_code removeImportedCsvError;
    std::filesystem::remove(importedCsvPath, removeImportedCsvError);
    if (!jointImport.success ||
        jointImport.dataKind != motion_planning::TrajectoryImportDataKind::JointTrajectory ||
        jointImport.plan.trajectory.points.size() != 3)
        return fail("CSV joint trajectory import failed: " + jointImport.message());
    if (!motion_planning::MotionPlanningProjectStore::upsertPlan(roundTripDocument, jointImport.plan, &errorMessage))
        return fail("Imported joint trajectory store failed: " + errorMessage);

    const std::filesystem::path importedCartesianPath =
        std::filesystem::current_path() / "imported_cartesian_points.txt";
    {
        std::ofstream text(importedCartesianPath);
        text << "=== Point 1 ===\n";
        text << "Pose (4x4 matrix):\n";
        text << "1 0 0 100\n";
        text << "0 1 0 0\n";
        text << "0 0 1 50\n";
        text << "0 0 0 1\n";
        text << "=== Point 2 ===\n";
        text << "Pose (4x4 matrix):\n";
        text << "1 0 0 120\n";
        text << "0 1 0 5\n";
        text << "0 0 1 55\n";
        text << "0 0 0 1\n";
    }
    const motion_planning::TrajectoryImportResult cartesianImport =
        motion_planning::ProjectTrajectoryImporter::importFile(importedCartesianPath, importOptions);
    std::error_code removeImportedCartesianError;
    std::filesystem::remove(importedCartesianPath, removeImportedCartesianError);
    const auto& importedCartesianPoints = cartesianImport.plan.cartesianControlPoints.points;
    if (!cartesianImport.success ||
        cartesianImport.dataKind != motion_planning::TrajectoryImportDataKind::CartesianControlPoints ||
        importedCartesianPoints.size() != 2 ||
        !cartesianImport.plan.trajectory.empty())
        return fail("Cartesian trajectory control-point import failed: " + cartesianImport.message());
    if (std::abs(importedCartesianPoints.front().time) > 1.0e-9 ||
        std::abs(importedCartesianPoints.back().time - 5.0) > 1.0e-9)
        return fail("Cartesian trajectory control-point TXT timing mismatch.");
    if (std::abs(importedCartesianPoints.front().tcpPose.translation().x() - 0.1) > 1.0e-9 ||
        std::abs(importedCartesianPoints.back().tcpPose.translation().x() - 0.12) > 1.0e-9)
        return fail("Cartesian trajectory control-point TXT unit conversion mismatch.");
    if (!motion_planning::MotionPlanningProjectStore::upsertPlan(roundTripDocument, cartesianImport.plan, &errorMessage))
        return fail("Imported cartesian control-point store failed: " + errorMessage);

    const std::filesystem::path roundTripPath = std::filesystem::current_path() /
        "ompl_motion_planning_roundtrip.sys.json";
    if (!simulation_project::saveProjectDocumentV3(roundTripPath, roundTripDocument, &errorMessage))
        return fail("Round-trip project save failed: " + errorMessage);
    simulation_project::ProjectDocument reloaded;
    if (!simulation_project::loadProjectDocument(roundTripPath, reloaded, &errorMessage))
        return fail("Round-trip project reload failed: " + errorMessage);
    std::error_code removeError;
    std::filesystem::remove(roundTripPath, removeError);

    const std::vector<motion_planning::StoredMotionPlan> plans =
        motion_planning::MotionPlanningProjectStore::plans(reloaded);
    const auto storedIt = std::find_if(plans.begin(), plans.end(), [](const motion_planning::StoredMotionPlan& plan) {
        return plan.id == "ompl_headless_regression";
    });
    if (storedIt == plans.end() || storedIt->trajectory.points.size() != result.trajectory.points.size())
        return fail("Stored trajectory did not survive project round-trip.");
    const auto importedJointIt = std::find_if(
        plans.begin(),
        plans.end(),
        [&](const motion_planning::StoredMotionPlan& plan) {
            return plan.id == jointImport.plan.id;
        });
    if (importedJointIt == plans.end() || importedJointIt->trajectory.points.size() != 3)
        return fail("Imported joint trajectory did not survive project round-trip.");
    const auto importedCartesianIt = std::find_if(
        plans.begin(),
        plans.end(),
        [&](const motion_planning::StoredMotionPlan& plan) {
            return plan.id == cartesianImport.plan.id;
        });
    if (importedCartesianIt == plans.end() ||
        importedCartesianIt->cartesianControlPoints.points.size() != 2 ||
        !importedCartesianIt->trajectory.empty())
        return fail("Imported cartesian control points did not survive project round-trip.");

    robotruntime::RobotTrajectoryExecutionSession execution;
    if (!execution.load(request.robotId, storedIt->id, storedIt->trajectory).success)
        return fail("Trajectory execution load failed.");
    if (!execution.start().success)
        return fail("Trajectory execution start failed.");
    while (execution.snapshot().state != robotruntime::RobotRunExecutionState::Completed)
    {
        if (!execution.step(0.05).success)
            return fail("Trajectory execution step failed.");
        if (!scene->setState(execution.snapshot().jointValues, &errorMessage))
            return fail("Runtime trajectory application failed: " + errorMessage);
    }
    if (!nearlyEqual(execution.snapshot().jointValues, goal))
        return fail("Trajectory execution did not finish at the goal state.");

    std::cout << "PASS ProjectMotionPlanning headless regression\n";
    std::cout << "  joints=" << request.jointNames.size()
              << " waypoints=" << result.trajectory.points.size()
              << " sampledStates=" << result.sampledStateCount
              << " planningSeconds=" << result.planningTimeSeconds << "\n";
    std::cout << "  startTool=" << startTool.transpose() << "\n";
    std::cout << "  goalTool=" << goalTool.transpose() << "\n";
    std::cout << "  start=" << vectorText(start) << "\n";
    std::cout << "  goal=" << vectorText(goal) << "\n";
    return 0;
}
