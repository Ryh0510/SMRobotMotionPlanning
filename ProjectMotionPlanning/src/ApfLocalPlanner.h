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
        std::function<bool(const ApfState&, const ApfState&)> motionValid;
    };

    bool planApfPath(const ApfPath& reference, const ApfState& lower,
        const ApfState& upper, const ApfOracle& oracle, double clearance,
        ApfPath* output);
}
