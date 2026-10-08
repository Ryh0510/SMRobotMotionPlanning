#pragma once

#include "CdfDistanceField.h"

#include <functional>
#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>
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
        ~CdfQueryBatch();

        std::vector<SignedDistanceSample> distances(const std::vector<std::vector<double>>& path,
            double margin, double horizon, ProjectCdfQpRepairStatistics& statistics);
        bool clearanceValid(const std::vector<std::vector<double>>& path, double margin, double horizon,
            double minimumPhi, ProjectCdfQpRepairStatistics& statistics);
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
        void workerLoop(std::size_t worker);
        void runBatch(std::size_t worker);
        std::vector<std::unique_ptr<ProjectPlanningSceneSnapshot>> m_owned;
        std::vector<ProjectPlanningSceneSnapshot*> m_scenes;
        std::unordered_map<std::vector<double>, SignedDistanceSample, StateHash> m_distances;
        std::unordered_map<std::vector<double>, StateValidationResult, StateHash> m_motions;
        bool m_cacheEnabled = true;
        std::size_t m_distanceHits = 0, m_motionHits = 0;
        double m_distanceSeconds = 0, m_gradientSeconds = 0, m_motionSeconds = 0;
        // Each thread has exclusive access to the same private scene for the
        // lifetime of this repair. Every batch joins before the next APF/QP step.
        std::vector<std::thread> m_threads;
        std::mutex m_mutex;
        std::condition_variable m_jobReady, m_jobDone;
        std::function<void(std::size_t, std::size_t)> m_work;
        std::atomic<std::size_t> m_next{0};
        std::size_t m_count = 0, m_activeWorkers = 0, m_remaining = 0, m_generation = 0;
        std::exception_ptr m_error;
        bool m_stopping = false, m_poolStarted = false;
    };
}
