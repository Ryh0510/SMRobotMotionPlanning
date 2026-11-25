#pragma once
#include <ProjectMotionPlanning/TrajectoryImport.h>
namespace motion_planning { namespace detail {
TrajectoryImportResult importFanucLs(const std::filesystem::path& path, const TrajectoryImportOptions& options);
}}
