#pragma once

#include "ApfLocalPlanner.h"

namespace motion_planning::detail
{
    // Input coordinates must share one continuous (unwrapped) joint chart.
    double jointPathLength(const ApfPath& path);
    double jointPathBending(const ApfPath& path, const std::vector<double>& times);
    // orderedCorridor requires pathValid to enforce the ordered TCP corridor and
    // real joint bounds; the APF-seed joint box is then replaced by that corridor.
    bool smoothValidatedPath(ApfPath* path, const std::vector<double>& times,
        const ApfOracle& oracle, int passes, double step, double maxDeviation, bool orderedCorridor = false);
}
