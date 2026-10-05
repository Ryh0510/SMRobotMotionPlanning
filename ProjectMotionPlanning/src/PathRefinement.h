#pragma once

#include "ApfLocalPlanner.h"

namespace motion_planning::detail
{
    // Input coordinates must share one continuous (unwrapped) joint chart.
    double jointPathLength(const ApfPath& path);
    double jointPathBending(const ApfPath& path, const std::vector<double>& times);
    // Restore ordered Cartesian shape using collision-limited, spatially smooth
    // displacement. Endpoints/turn coordinates stay fixed; failure is atomic.
    bool restoreApfTcpShape(ApfPath* path, const ApfState& lower, const ApfState& upper,
        const ApfOracle& oracle, const ApfGuidance& guidance);
    // Shape restoration followed by joint fairing projected onto the restored
    // TCP curve. The original oracle remains the final acceptance gate.
    bool refineApfTcpShape(ApfPath* path, const std::vector<double>& times,
        const ApfState& lower, const ApfState& upper, const ApfOracle& oracle,
        const ApfGuidance& guidance, int passes, double step);
    // orderedCorridor requires pathValid to enforce the ordered TCP corridor and
    // real joint bounds; the APF-seed joint box is then replaced by that corridor.
    bool smoothValidatedPath(ApfPath* path, const std::vector<double>& times,
        const ApfOracle& oracle, int passes, double step, double maxDeviation, bool orderedCorridor = false);
}
