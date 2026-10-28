#pragma once

#include <MotionPlanningIk/IkTypes.h>

namespace motion_planning::ik
{
    class SprayIkPlanner
    {
    public:
        static IkResult solve(const IkRequest& request);
    };
}
