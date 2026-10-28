#pragma once

#include <MotionPlanningIk/IkTypes.h>

namespace motion_planning::ik
{
    class IkSolver
    {
    public:
        static IkResult solve(const IkRequest& request);
    };
}
