#pragma once

#include <PostModel/PostInterface-global.h>

#include <array>
#include <string>
#include <vector>

namespace post_model
{
    enum class MotionType
    {
        Joint,
        Linear
    };

    struct MotionCommand
    {
        MotionType type{ MotionType::Linear };
        std::array<double, 16> poseMillimeters{};
        std::vector<double> jointsDegrees;
        std::vector<double> configuration{ 0.0, 0.0, 0.0 };
        double speedMillimetersPerSecond{ 100.0 };
    };

    struct ProgramRequest
    {
        std::string templateName;
        std::string robotName;
        std::string programName;
        std::string outputDirectory;
        std::array<double, 16> framePoseMillimeters{};
        std::array<double, 16> toolPoseMillimeters{};
        std::vector<MotionCommand> commands;
    };

    struct ProgramResult
    {
        bool success{ false };
        std::string errorMessage;
        std::vector<std::string> outputFiles;
        std::string previewText;
    };

    class POSTINTERFACE_SHAREDLIB PostProgramRunner
    {
    public:
        static std::vector<std::string> availableTemplates();
        static ProgramResult generate(const ProgramRequest& request);
    };
}
