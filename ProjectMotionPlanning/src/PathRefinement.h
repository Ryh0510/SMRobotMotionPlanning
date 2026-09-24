#pragma once

#include "ApfLocalPlanner.h"

namespace motion_planning::detail
{
    // Input coordinates must share one continuous (unwrapped) joint chart.
    double jointPathLength(const ApfPath& path);
    double jointPathBending(const ApfPath& path, const std::vector<double>& times);
    void shortcutApfPath(ApfPath* path, const ApfOracle& oracle);
    bool smoothValidatedPath(ApfPath* path, const std::vector<double>& times,
        const ApfOracle& oracle, int passes, double step, double maxDeviation);
}
