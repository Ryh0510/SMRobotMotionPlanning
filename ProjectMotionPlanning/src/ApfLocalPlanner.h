#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace motion_planning::detail
{
    using ApfState = std::vector<double>;
    using ApfPath = std::vector<ApfState>;

    // Coordinates are locally unwrapped by the caller. Negative distance is
    // forbidden; a non-finite distance is an oracle failure, not free space.
    struct ApfOracle
    {
        std::function<double(const ApfState&)> distance;
        // Optional equivalent distance observations in input order (e.g. finite differences).
        std::function<std::vector<double>(const ApfPath&)> distances;
        std::function<void(int, int)> progress;
        std::function<void(int, std::size_t, const ApfState&)> failedStation;
        std::function<bool(const ApfState&, const ApfState&)> motionValid;
        // Equivalent batch motion checks, one result per raw adjacent segment.
        std::function<std::vector<bool>(const ApfPath&)> motionsValid;
        // Optional actual-model projection, before cost and full path validation.
        std::function<bool(ApfState&, std::size_t)> project;
        // Optional equivalent batch check for a complete smoothing window.
        std::function<bool(const ApfPath&, std::size_t)> pathValid;
    };

    struct ApfGuidance
    {
        // Actual TCP position, in metres. Positions keep the input point order.
        std::function<ApfState(const ApfState&)> position;
        ApfPath positions;
        double maxDeviation = 0.10;
    };

    // Validates correspondence along the whole joint-linear segment, not just
    // its endpoints. No wrapping: the caller supplies legal turn coordinates.
    bool followsApfGuide(const ApfState& from, const ApfState& to,
        const ApfState& guideFrom, const ApfState& guideTo,
        const ApfGuidance& guidance, double jointStep = 0.001);

    bool followsApfPath(const ApfPath& path, const ApfGuidance& guidance, std::size_t begin = 0);

    // A finite proposal only; the caller must validate the full motion afterwards.
    bool projectApfPosition(ApfState& q, const ApfState& target,
        const ApfState& lower, const ApfState& upper, const ApfGuidance& guidance,
        double radius, int iterations = 12);

    bool planApfPath(const ApfPath& reference, const ApfState& lower,
        const ApfState& upper, const ApfOracle& oracle, double clearance,
        ApfPath* output, const ApfGuidance* guidance = nullptr);
}
