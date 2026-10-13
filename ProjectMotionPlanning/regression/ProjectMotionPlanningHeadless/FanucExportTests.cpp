#include <ProjectMotionPlanning/FanucTrajectoryExport.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <locale>
#include <regex>
#include <stdexcept>

namespace
{
    constexpr double pi = 3.14159265358979323846;
    void require(bool value, const std::string& message)
    {
        if(!value) throw std::runtime_error(message);
    }

    Eigen::Isometry3d poseFor(const std::vector<double>& q)
    {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(q[0], q[1], q[2]);
        pose.linear() = (Eigen::AngleAxisd(q[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(q[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(q[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
        return pose;
    }

    struct CommaDecimal : std::numpunct<char> { char do_decimal_point() const override { return ','; } };
}

int main()
{
    try {
        motion_planning::StoredMotionPlan plan;
        plan.jointNames = {"J1", "J2", "J3", "J4", "J5", "J6"};
        for(int i = 0; i < 1007; ++i) {
            robottrajectory::TimedJointPoint point;
            point.time = i * 0.1;
            // Exercise turns, both WPR solutions, near singularities and exact poles.
            point.q = {1.1234567 + i * 0.001, -.2000004, .4, 3.2 + i * .007, -.8 + i * .017, -.5 + i * .013};
            if(i < 6) point.q[4] = (i % 2 ? -1 : 1) * pi / 2 + (i / 2) * 1e-11;
            plan.trajectory.points.push_back(point);
        }
        motion_planning::FanucTrajectoryExportOptions options;
        options.mapIrb4600StoredJointSigns = true;
        std::size_t evaluated = 0;
        options.worldTcpForwardKinematics = [&](const std::vector<double>& q) -> Eigen::Isometry3d {
            auto expected = plan.trajectory.points.at(evaluated++).q;
            for(int j : {0, 3, 4, 5}) expected[j] = -expected[j];
            require(q == expected, "Stored signs mapped exactly once; turns never wrapped");
            return poseFor(q);
        };
        const auto originalLocale = std::locale();
        std::locale::global(std::locale(originalLocale, new CommaDecimal));
        const auto result = motion_planning::ProjectFanucTrajectoryExporter::generate(plan, options);
        std::locale::global(originalLocale);
        require(result.success, result.error);
        require(result.pointCount == 1007 && evaluated == 1007, "Export all original rows");
        require(result.program.find("/PROG  JHA\r\n/ATTR\r\n") == 0, "JHA program header");
        require(result.program.find("LINE_COUNT\t= 1009;\r\n") != std::string::npos, "Actual MN line count");
        require(result.program.find("1009:  L P[1007] 400mm/sec CNT100") != std::string::npos, "Final instruction");
        require(result.program.find("/END\r\n") == result.program.size() - 6, "Complete trailer");
        for(std::size_t i = 0; i < result.program.size(); ++i) {
            require(static_cast<unsigned char>(result.program[i]) < 128, "ASCII program without BOM");
            if(result.program[i] == '\n') require(i && result.program[i-1] == '\r', "CRLF line endings");
        }
        const std::regex position(R"(P\[(\d+)\]\{\s+GP1:\s+UF : 2, UT : 1,\s+CONFIG : 'N U T, 0, 0, 0',\s+X = ([\-0-9.]+)  mm,\s+Y = ([\-0-9.]+)  mm,\s+Z = ([\-0-9.]+)  mm,\s+W = ([\-0-9.]+) deg,\s+P = ([\-0-9.]+) deg,\s+R = ([\-0-9.]+) deg\s+\};)");
        std::size_t count = 0;
        for(auto it = std::sregex_iterator(result.program.begin(), result.program.end(), position); it != std::sregex_iterator(); ++it) {
            const auto& match = *it;
            require(std::stoul(match[1]) == count + 1, "Sequential position references");
            std::vector<double> q(6);
            for(int j = 0; j < 6; ++j) q[j] = std::stod(match[j+2]) * (j < 3 ? .001 : pi / 180.0);
            const auto actual = poseFor(q);
            auto input = plan.trajectory.points[count].q;
            for(int j : {0, 3, 4, 5}) input[j] = -input[j];
            const auto expected = poseFor(input);
            require((actual.translation() - expected.translation()).norm() <= 9e-7, "XYZ mm accuracy");
            require(Eigen::AngleAxisd(expected.linear().transpose() * actual.linear()).angle() < 3e-5,
                "WPR rotation reconstruction, including positive and negative gimbal lock");
            ++count;
        }
        require(count == result.pointCount, "Parse every POS record");
        options.mapIrb4600StoredJointSigns = false;
        options.worldTcpForwardKinematics = poseFor;
        const auto reject = [&](const auto& input, const auto& settings) {
            const auto bad = motion_planning::ProjectFanucTrajectoryExporter::generate(input, settings);
            require(!bad.success && bad.program.empty() && bad.pointCount == 0 && !bad.error.empty(), "Reject atomically");
        };
        auto bad = plan;
        bad.trajectory.points.back().q[0] = std::numeric_limits<double>::quiet_NaN(); reject(bad, options);
        bad = plan; bad.trajectory.points.back().q.pop_back(); reject(bad, options);
        bad = plan; bad.trajectory.points.back().time = std::numeric_limits<double>::infinity(); reject(bad, options);
        bad = plan; bad.jointNames.back() = bad.jointNames.front(); reject(bad, options);
        bad = plan; bad.trajectory.points.clear(); reject(bad, options);
        auto invalid = options; invalid.worldTcpForwardKinematics = {}; reject(plan, invalid);
        invalid = options; invalid.programName = "BAD\r\n/MN"; reject(plan, invalid);
        invalid = options; invalid.programName = "LONG_NAME"; reject(plan, invalid);
        invalid = options; invalid.worldTcpForwardKinematics = [](const auto&) -> Eigen::Isometry3d {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity(); pose.linear()(0,0) = -1; return pose;
        }; reject(plan, invalid);
        invalid.worldTcpForwardKinematics = [](const auto&) -> Eigen::Isometry3d { throw std::runtime_error("FK failed"); };
        reject(plan, invalid);
        // Check the non-ABB path, custom legal name, and cancellation-free generation.
        bad = plan; bad.trajectory.points.resize(1); bad.trajectory.points[0].q = {.1,.2,.3,0,0,0};
        options.programName = "TEST_123";
        const auto simple = motion_planning::ProjectFanucTrajectoryExporter::generate(bad, options);
        require(simple.success && simple.program.find("/PROG  TEST_123") == 0 &&
            simple.program.find("X = 100.000  mm") != std::string::npos &&
            simple.program.find("W = 0.000 deg,\tP = 0.000 deg,\tR = 0.000 deg") != std::string::npos,
            "Non-ABB axes and canonical identity orientation");
        std::cout << "FANUC LS: all rows, names, template, units, WPR/poles, signs/turns, locale and invalid input passed.\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
