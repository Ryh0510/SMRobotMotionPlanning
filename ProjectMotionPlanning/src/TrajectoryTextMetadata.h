#pragma once
#include <ProjectMotionPlanning/ProjectMotionPlanning.h>
#include <nlohmann/json.hpp>

namespace motion_planning { namespace detail {
// Optional V1 header fields; the historical collision report stays in the
// project/report, and is never promoted to a fresh scene certificate on import.
void exportTrajectoryProcessMetadata(const StoredMotionPlan&, nlohmann::json& header);
void importTrajectoryProcessMetadata(const nlohmann::json& header, StoredMotionPlan&, std::size_t count);
}}
