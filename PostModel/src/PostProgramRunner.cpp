#include <PostModel/PostProgramRunner.h>

#include <PostModel/ABB_3.h>
#include <PostModel/ABB_7.h>
#include <PostModel/ABB_RAPID_IRC5.h>
#include <PostModel/ABB_RAPID_SC4.h>
#include <PostModel/AUBO.h>
#include <PostModel/Fanuc_R30iA.h>
#include <PostModel/Fanuc_RJ3.h>
#include <PostModel/Fanuc_RJ3_DripFeed.h>
#include <PostModel/KUKA_IIWA.h>
#include <PostModel/KUKA_KRC2.h>
#include <PostModel/KUKA_KRC4.h>
#include <PostModel/KUKA_custom.h>
#include <PostModel/Kawasaki.h>
#include <PostModel/Motoman.h>
#include <PostModel/Motoman_Cartesian.h>
#include <PostModel/Motoman_CartesianOnly.h>
#include <PostModel/Staubli_S6.h>
#include <PostModel/Staubli_VAL3.h>
#include <PostModel/Universal_Robots.h>
#include <PostModel/Universal_Robots_URP.h>
#include <PostModel/robomath.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <stdexcept>

namespace post_model
{
    namespace
    {
        using FileStamp = std::pair<std::uintmax_t, std::filesystem::file_time_type>;

        robomath::Mat makePose(const std::array<double, 16>& values)
        {
            return robomath::Mat({
                { values[0], values[1], values[2], values[3] },
                { values[4], values[5], values[6], values[7] },
                { values[8], values[9], values[10], values[11] },
                { values[12], values[13], values[14], values[15] }
            });
        }

        bool isFinitePose(const std::array<double, 16>& pose)
        {
            return std::all_of(pose.begin(), pose.end(),
                [](double value) { return std::isfinite(value); });
        }

        bool isIdentityPose(const std::array<double, 16>& pose)
        {
            constexpr std::array<double, 16> identity{
                1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0
            };
            for(std::size_t index = 0; index < pose.size(); ++index) {
                if(std::abs(pose[index] - identity[index]) > 1.0e-12) {
                    return false;
                }
            }
            return true;
        }

        std::map<std::filesystem::path, FileStamp> snapshotFiles(
            const std::filesystem::path& directory)
        {
            std::map<std::filesystem::path, FileStamp> result;
            std::error_code error;
            if(!std::filesystem::exists(directory, error)) {
                return result;
            }
            for(std::filesystem::recursive_directory_iterator iterator(
                    directory,
                    std::filesystem::directory_options::skip_permission_denied,
                    error), end;
                iterator != end;
                iterator.increment(error)) {
                if(error) {
                    error.clear();
                    continue;
                }
                if(!iterator->is_regular_file(error)) {
                    error.clear();
                    continue;
                }
                const std::filesystem::path path = iterator->path();
                result[path] = {
                    iterator->file_size(error),
                    iterator->last_write_time(error)
                };
                error.clear();
            }
            return result;
        }

        std::vector<std::string> changedFiles(
            const std::filesystem::path& directory,
            const std::map<std::filesystem::path, FileStamp>& before)
        {
            std::vector<std::string> result;
            const auto after = snapshotFiles(directory);
            for(const auto& entry : after) {
                const auto old = before.find(entry.first);
                if(old == before.end() || old->second != entry.second) {
                    result.push_back(entry.first.u8string());
                }
            }
            std::sort(result.begin(), result.end());
            return result;
        }

        std::string readPreview(const std::vector<std::string>& files)
        {
            constexpr std::uintmax_t maximumPreviewBytes = 2U * 1024U * 1024U;
            std::ostringstream preview;
            for(const std::string& fileName : files) {
                const std::filesystem::path path = std::filesystem::u8path(fileName);
                std::error_code error;
                const std::uintmax_t size = std::filesystem::file_size(path, error);
                if(error || size == 0 || size > maximumPreviewBytes) {
                    continue;
                }
                std::ifstream input(path, std::ios::binary);
                std::string content{
                    std::istreambuf_iterator<char>(input),
                    std::istreambuf_iterator<char>() };
                if(content.find('\0') != std::string::npos) {
                    continue;
                }
                if(preview.tellp() > 0) {
                    preview << "\n\n";
                }
                preview << "===== " << path.filename().u8string() << " =====\n";
                preview << content;
            }
            return preview.str();
        }

        template<typename RobotPost>
        void appendProgram(RobotPost& robot, const ProgramRequest& request)
        {
            robot.ProgStart(request.programName);
            if(!isIdentityPose(request.framePoseMillimeters)) {
                robot.setFrame(makePose(request.framePoseMillimeters));
            }
            if(!isIdentityPose(request.toolPoseMillimeters)) {
                robot.setTool(makePose(request.toolPoseMillimeters));
            }
            for(const MotionCommand& command : request.commands) {
                robot.setSpeed(command.speedMillimetersPerSecond);
                const robomath::Mat pose = makePose(command.poseMillimeters);
                if(command.type == MotionType::Joint) {
                    robot.MoveJ(pose, command.jointsDegrees, command.configuration);
                } else {
                    robot.MoveL(pose, command.jointsDegrees, command.configuration);
                }
            }
            robot.ProgFinish(request.programName);
            robot.ProgSave(request.outputDirectory, request.programName, false, "");
        }

        template<typename RobotPost>
        void generateWithTemplate(const ProgramRequest& request)
        {
            RobotPost robot(
                request.templateName,
                request.robotName,
                6,
                std::vector<std::string>(6, "R"),
                "");
            appendProgram(robot, request);
        }

        template<typename RobotPost>
        void generateFanuc(const ProgramRequest& request)
        {
            typename RobotPost::Config config;
            // MakeTP is optional and is not part of the portable post-processing
            // pipeline. The generated LS file remains the controller source.
            config.pathMakeTP.clear();
            RobotPost robot(
                request.templateName,
                request.robotName,
                6,
                std::vector<std::string>(6, "R"),
                "",
                config);
            appendProgram(robot, request);
        }

        void dispatch(const ProgramRequest& request)
        {
            const std::string& name = request.templateName;
            if(name == "ABB_3") return generateWithTemplate<ABB_3::RobotPost>(request);
            if(name == "ABB_7") return generateWithTemplate<ABB_7::RobotPost>(request);
            if(name == "ABB_RAPID_IRC5") return generateWithTemplate<ABB_RAPID_IRC5::RobotPost>(request);
            if(name == "ABB_RAPID_SC4") return generateWithTemplate<ABB_RAPID_SC4::RobotPost>(request);
            if(name == "AUBO") return generateWithTemplate<AUBO::RobotPost>(request);
            if(name == "Fanuc_R30iA") return generateFanuc<Fanuc_R30iA::RobotPost>(request);
            if(name == "Fanuc_RJ3") return generateFanuc<Fanuc_RJ3::RobotPost>(request);
            if(name == "Fanuc_RJ3_DripFeed") return generateFanuc<Fanuc_RJ3_DripFeed::RobotPost>(request);
            if(name == "KUKA_IIWA") return generateWithTemplate<KUKA_IIWA::RobotPost>(request);
            if(name == "KUKA_KRC2") return generateWithTemplate<KUKA_KRC2::RobotPost>(request);
            if(name == "KUKA_KRC4") return generateWithTemplate<KUKA_KRC4::RobotPost>(request);
            if(name == "KUKA_custom") return generateWithTemplate<KUKA_custom::RobotPost>(request);
            if(name == "Kawasaki") return generateWithTemplate<Kawasaki::RobotPost>(request);
            if(name == "Motoman") return generateWithTemplate<Motoman::RobotPost>(request);
            if(name == "Motoman_Cartesian") return generateWithTemplate<Motoman_Cartesian::RobotPost>(request);
            if(name == "Motoman_CartesianOnly") return generateWithTemplate<Motoman_CartesianOnly::RobotPost>(request);
            if(name == "Staubli_S6") return generateWithTemplate<Staubli_S6::RobotPost>(request);
            if(name == "Staubli_VAL3") return generateWithTemplate<Staubli_VAL3::RobotPost>(request);
            if(name == "Universal_Robots") return generateWithTemplate<Universal_Robots::RobotPost>(request);
            if(name == "Universal_Robots_URP") return generateWithTemplate<Universal_Robots_URP::RobotPost>(request);
            throw std::invalid_argument("Unknown post-processing template: " + name);
        }

        std::string validate(const ProgramRequest& request)
        {
            if(request.templateName.empty() || request.robotName.empty() ||
                request.programName.empty() || request.outputDirectory.empty()) {
                return "Template, robot, program name, and output directory are required.";
            }
            if(!isFinitePose(request.framePoseMillimeters) ||
                !isFinitePose(request.toolPoseMillimeters)) {
                return "Frame and tool poses must contain finite values.";
            }
            if(request.commands.empty()) {
                return "At least one motion command is required.";
            }
            for(std::size_t index = 0; index < request.commands.size(); ++index) {
                const MotionCommand& command = request.commands[index];
                if(!isFinitePose(command.poseMillimeters) ||
                    command.jointsDegrees.size() != 6 ||
                    !std::all_of(command.jointsDegrees.begin(), command.jointsDegrees.end(),
                        [](double value) { return std::isfinite(value); }) ||
                    !std::isfinite(command.speedMillimetersPerSecond) ||
                    command.speedMillimetersPerSecond <= 0.0) {
                    return "Motion command " + std::to_string(index + 1) +
                        " does not contain a finite pose, six finite joints, and a positive speed.";
                }
            }
            return {};
        }
    }

    std::vector<std::string> PostProgramRunner::availableTemplates()
    {
        return {
            "ABB_3",
            "ABB_7",
            "ABB_RAPID_IRC5",
            "ABB_RAPID_SC4",
            "AUBO",
            "Fanuc_R30iA",
            "Fanuc_RJ3",
            "Fanuc_RJ3_DripFeed",
            "KUKA_IIWA",
            "KUKA_KRC2",
            "KUKA_KRC4",
            "KUKA_custom",
            "Kawasaki",
            "Motoman",
            "Motoman_Cartesian",
            "Motoman_CartesianOnly",
            "Staubli_S6",
            "Staubli_VAL3",
            "Universal_Robots",
            "Universal_Robots_URP"
        };
    }

    ProgramResult PostProgramRunner::generate(const ProgramRequest& request)
    {
        ProgramResult result;
        result.errorMessage = validate(request);
        if(!result.errorMessage.empty()) {
            return result;
        }
        const std::vector<std::string> templates = availableTemplates();
        if(std::find(templates.begin(), templates.end(), request.templateName) ==
            templates.end()) {
            result.errorMessage = "The selected post-processing template is not available.";
            return result;
        }
        try {
            const std::filesystem::path outputDirectory =
                std::filesystem::u8path(request.outputDirectory);
            std::filesystem::create_directories(outputDirectory);
            const auto before = snapshotFiles(outputDirectory);
            dispatch(request);
            result.outputFiles = changedFiles(outputDirectory, before);
            if(result.outputFiles.empty()) {
                result.errorMessage = "The post processor did not create or update any output files.";
                return result;
            }
            result.previewText = readPreview(result.outputFiles);
            result.success = true;
        } catch(const std::exception& exception) {
            result.errorMessage = exception.what();
        }
        return result;
    }
}
