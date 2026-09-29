#pragma once

#include "CdfDistanceField.h"

#include <functional>
#include <unordered_map>

namespace motion_planning::detail
{
    // Private to ONE repair of an immutable scene. No quantization, wrapped keys,
    // global cache, or mutable collision runtime shared between workers.
    class CdfQueryBatch
    {
    public:
        CdfQueryBatch(ProjectPlanningSceneSnapshot& primary,
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& base, const ProjectPlanningRequest& request,
            int requestedWorkers, bool cacheEnabled = true);

        std::vector<SignedDistanceSample> distances(const std::vector<std::vector<double>>& path,
            double margin, double horizon, ProjectCdfQpRepairStatistics& statistics);
        std::vector<CdfLinearization> linearizations(const std::vector<std::vector<double>>& path,
            double margin, double horizon, double step, ProjectCdfQpRepairStatistics& statistics);
        std::vector<StateValidationResult> motions(const std::vector<std::vector<double>>& path,
            const MotionValidationOptions& options, bool useCache = true);
        bool pathValid(const std::vector<std::vector<double>>& path, const MotionValidationOptions& options);
        std::string summary() const;
        std::size_t workerCount() const { return m_scenes.size(); }

    private:
        struct StateHash { std::size_t operator()(const std::vector<double>& values) const; };
        void parallelFor(std::size_t count, const std::function<void(std::size_t, std::size_t)>& work);
        void rememberDistance(const std::vector<double>& key, const SignedDistanceSample& sample);
        std::vector<std::unique_ptr<ProjectPlanningSceneSnapshot>> m_owned;
        std::vector<ProjectPlanningSceneSnapshot*> m_scenes;
        std::unordered_map<std::vector<double>, SignedDistanceSample, StateHash> m_distances;
        std::unordered_map<std::vector<double>, StateValidationResult, StateHash> m_motions;
        bool m_cacheEnabled = true;
        std::size_t m_distanceHits = 0, m_motionHits = 0;
        double m_distanceSeconds = 0, m_gradientSeconds = 0, m_motionSeconds = 0;
    };
}
