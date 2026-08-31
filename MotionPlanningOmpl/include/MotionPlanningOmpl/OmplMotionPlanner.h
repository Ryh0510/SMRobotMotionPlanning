#pragma once

#include <MotionPlanningCore/MotionPlanning.h>

namespace motion_planning
{
    class OmplMotionPlanner final : public IJointMotionPlanner
    {
    public:
        MotionPlanningResult plan(
            const JointPlanningProblem& problem,
            IPlanningScene& scene,
            CancellationToken* cancellation = nullptr) const override;
    };
}
