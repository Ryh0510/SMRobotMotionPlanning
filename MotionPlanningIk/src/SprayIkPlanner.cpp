#include <MotionPlanningIk/SprayIkPlanner.h>

#include <MotionPlanningIk/IkSolver.h>

namespace motion_planning::ik
{
    IkResult SprayIkPlanner::solve(const IkRequest& request)
    {
        return IkSolver::solve(request);
    }
}
