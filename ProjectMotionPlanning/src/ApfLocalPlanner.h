#pragma once

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
        std::function<bool(const ApfState&, const ApfState&)> motionValid;
        // Optional equivalent batch check for a complete smoothing window.
        std::function<bool(const ApfPath&)> pathValid;
    };

    bool planApfPath(const ApfPath& reference, const ApfState& lower,
        const ApfState& upper, const ApfOracle& oracle, double clearance,
        ApfPath* output);
}
