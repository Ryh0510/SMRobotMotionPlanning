#include <PostModel/PostProgramRunner.h>

#include <filesystem>
#include <iostream>
#include <string>

namespace
{
    std::array<double, 16> pose(double x, double y, double z)
    {
        return {
            1.0, 0.0, 0.0, x,
            0.0, 1.0, 0.0, y,
            0.0, 0.0, 1.0, z,
            0.0, 0.0, 0.0, 1.0
        };
    }

    bool expect(bool condition, const std::string& message)
    {
        if(!condition) {
            std::cerr << message << '\n';
        }
        return condition;
    }
}

int main(int argc, char** argv)
{
    const std::vector<std::string> templates =
        post_model::PostProgramRunner::availableTemplates();
    if(!expect(templates.size() == 20, "Expected all 20 built-in post templates.")) {
        return 1;
    }

    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "rs2026_post_program_runner_test";
    std::error_code error;
    std::filesystem::remove_all(root, error);
    std::filesystem::create_directories(root);

    for(const std::string& templateName : templates) {
        if(argc > 1 && templateName != argv[1]) {
            continue;
        }
        std::cout << "Testing " << templateName << std::endl;
        post_model::ProgramRequest request;
        request.templateName = templateName;
        request.robotName = "SixAxisRobot";
        request.programName = "PostSmoke";
        request.outputDirectory = (root / templateName).u8string();
        request.framePoseMillimeters = pose(0.0, 0.0, 0.0);
        request.toolPoseMillimeters = pose(0.0, 0.0, 100.0);

        post_model::MotionCommand joint;
        joint.type = post_model::MotionType::Joint;
        joint.poseMillimeters = pose(400.0, 0.0, 500.0);
        joint.jointsDegrees = { 0.0, -30.0, 45.0, 0.0, 60.0, 0.0 };
        joint.speedMillimetersPerSecond = 200.0;
        request.commands.push_back(joint);

        post_model::MotionCommand linear = joint;
        linear.type = post_model::MotionType::Linear;
        linear.poseMillimeters = pose(450.0, 25.0, 500.0);
        linear.jointsDegrees = { 2.0, -28.0, 43.0, 1.0, 61.0, 2.0 };
        linear.speedMillimetersPerSecond = 50.0;
        request.commands.push_back(linear);

        const post_model::ProgramResult result =
            post_model::PostProgramRunner::generate(request);
        if(!expect(result.success,
                templateName + " failed: " + result.errorMessage) ||
            !expect(!result.outputFiles.empty(),
                templateName + " produced no output files.")) {
            return 1;
        }
        for(const std::string& outputFile : result.outputFiles) {
            const std::filesystem::path path = std::filesystem::u8path(outputFile);
            if(!expect(std::filesystem::is_regular_file(path),
                    templateName + " reported a missing output file: " + outputFile) ||
                !expect(std::filesystem::file_size(path) > 0,
                    templateName + " produced an empty output file: " + outputFile)) {
                return 1;
            }
        }
    }

    post_model::ProgramRequest invalid;
    invalid.templateName = templates.front();
    invalid.robotName = "SixAxisRobot";
    invalid.programName = "Invalid";
    invalid.outputDirectory = (root / "invalid").u8string();
    invalid.framePoseMillimeters = pose(0.0, 0.0, 0.0);
    invalid.toolPoseMillimeters = pose(0.0, 0.0, 0.0);
    post_model::MotionCommand invalidCommand;
    invalidCommand.poseMillimeters = pose(0.0, 0.0, 0.0);
    invalidCommand.jointsDegrees = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    invalid.commands.push_back(invalidCommand);
    if(!expect(!post_model::PostProgramRunner::generate(invalid).success,
            "A command without exactly six joints must be rejected.")) {
        return 1;
    }

    std::filesystem::remove_all(root, error);
    return 0;
}
