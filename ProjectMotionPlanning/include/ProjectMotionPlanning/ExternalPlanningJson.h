#pragma once
#include <SimulationProject/ProjectDocument.h>
#include <filesystem>
#include <functional>
#include <string>

namespace motion_planning
{
    // Shared JSON boundary for CLI and live GUI snapshot jobs. Executes synchronously
    // on the caller's worker; catches failures and returns a structured JSON report.
    std::string runExternalPlanningJson(const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath, const std::string& requestJson,
        const std::filesystem::path& requestBasePath,
        std::function<bool()> cancelled = {},
        std::function<void(const std::string&)> progress = {});
}
