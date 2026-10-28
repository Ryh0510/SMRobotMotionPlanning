#pragma once
#include <Eigen/Core>
#include <functional>
#include <filesystem>
#include <string>
#include <vector>

namespace motion_planning {
struct ZigzagSegmentationOptions {
    double directionAngleDegrees = 60.0;
    double minimumScanLength = 0.005; // metres; reversal hysteresis
};
struct ZigzagTrajectorySegment {
    std::size_t first = 0, last = 0; // inclusive; neighbours share one boundary
    double length = 0;
};
std::vector<ZigzagTrajectorySegment> segmentZigzagTrajectory(
    const std::vector<Eigen::Vector3d>& positions,
    const ZigzagSegmentationOptions& options = {},
    std::function<void(const ZigzagTrajectorySegment&)> ready = {},
    std::function<bool()> cancelled = {});
// SDK trajectory JSON: points[].position_m. Preserve all source point metadata.
// Each segment is written before its callback, followed by an atomic manifest.
std::string segmentZigzagTrajectoryJson(const std::string& trajectoryJson,
    const std::filesystem::path& outputDirectory,
    const ZigzagSegmentationOptions& options = {},
    std::function<bool()> cancelled = {});
}
