#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

#include <filesystem>
#include <string>
#include <vector>

namespace motion_planning
{

enum class CdfJointAngleUnit
{
    Degrees,
    Radians
};

struct CdfJointAnglePoint
{
    double timeSeconds = 0.0;
    std::vector<double> jointAnglesDegrees;
};

struct CdfJointAngleImportResult
{
    bool success = false;
    std::string sourceName;
    CdfJointAngleUnit sourceUnit = CdfJointAngleUnit::Degrees;
    std::vector<std::string> jointNames;
    std::vector<CdfJointAnglePoint> points;
    std::vector<std::string> diagnostics;
    // Exact source commands and motion law; populated only for a validated versioned export.
    bool hasMotionMetadata = false;
    StoredMotionPlan plan;

    std::string message() const;
};

class ProjectCdfJointAngleImporter
{
public:
    static CdfJointAngleImportResult importFile(const std::filesystem::path& path);
};

} // namespace motion_planning
