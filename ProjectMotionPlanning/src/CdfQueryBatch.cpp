#include "CdfQueryBatch.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>

namespace motion_planning::detail
{
    namespace
    {
        constexpr std::size_t kCacheEntries = 32768;
        using Clock = std::chrono::steady_clock;
        std::vector<double> distanceKey(const std::vector<double>& q, double margin, double horizon)
        {
            std::vector<double> key{margin, horizon};
            key.insert(key.end(), q.begin(), q.end()); return key;
        }
        std::vector<double> motionKey(const std::vector<double>& a, const std::vector<double>& b, double step)
        {
            std::vector<double> key{step, static_cast<double>(a.size())};
            key.insert(key.end(), a.begin(), a.end()); key.insert(key.end(), b.begin(), b.end()); return key;
        }
        double secondsSince(Clock::time_point start)
        {
            return std::chrono::duration<double>(Clock::now() - start).count();
        }
    }

    std::size_t CdfQueryBatch::StateHash::operator()(const std::vector<double>& values) const
    {
        std::size_t hash = values.size();
        for(double value : values) { hash ^= std::hash<double>{}(value) + 0x9e3779b9 + (hash << 6) + (hash >> 2); }
        return hash;
    }

    CdfQueryBatch::CdfQueryBatch(ProjectPlanningSceneSnapshot& primary,
        const simulation_project::ProjectDocument& document, const std::filesystem::path& base,
        const ProjectPlanningRequest& request, int requestedWorkers, bool cacheEnabled)
        : m_cacheEnabled(cacheEnabled)
    {
        m_scenes.push_back(&primary);
        const int hardware = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
        const int workers = requestedWorkers <= 0 ? std::min(4, std::max(1, hardware / 2)) :
            std::clamp(requestedWorkers, 1, std::min(hardware, 8));
        // Construct runtimes serially; all mutable FK/geometry/query state is private.
        for(int i = 1; i < workers; ++i) {
            try {
                auto scene = ProjectPlanningSceneBuilder::build(document, base, request);
                if(!scene) { break; }
                m_owned.push_back(std::move(scene)); m_scenes.push_back(m_owned.back().get());
            } catch(const std::exception&) { break; } // Resource pressure: retain serial correctness.
        }
    }

    CdfQueryBatch::~CdfQueryBatch()
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_stopping = true;
        }
        m_jobReady.notify_all();
        for(auto& thread : m_threads) { thread.join(); }
    }

    void CdfQueryBatch::runBatch(std::size_t worker)
    {
        try {
            for(std::size_t i = m_next.fetch_add(1); i < m_count; i = m_next.fetch_add(1)) {
                m_work(worker, i);
            }
        } catch(...) {
            std::lock_guard<std::mutex> lock(m_mutex);
            if(!m_error) { m_error = std::current_exception(); }
        }
    }

    void CdfQueryBatch::workerLoop(std::size_t worker)
    {
        std::size_t seenGeneration = 0;
        std::unique_lock<std::mutex> lock(m_mutex);
        for(;;) {
            m_jobReady.wait(lock, [&]() { return m_stopping || m_generation != seenGeneration; });
            if(m_stopping) { return; }
            seenGeneration = m_generation;
            const bool active = worker < m_activeWorkers;
            lock.unlock();
            if(active) { runBatch(worker); }
            lock.lock();
            if(--m_remaining == 0) { m_jobDone.notify_one(); }
        }
    }

    void CdfQueryBatch::parallelFor(std::size_t count,
        const std::function<void(std::size_t, std::size_t)>& work)
    {
        if(count == 0) { return; }
        const std::size_t workers = std::min(m_scenes.size(), std::max<std::size_t>(1, count / 2));
        if(workers == 1) {
            for(std::size_t i = 0; i < count; ++i) { work(0, i); }
            return;
        }
        // Start once, on the first parallel batch. Subsequent short APF/smoothing
        // windows do not create and destroy OS threads thousands of times.
        if(!m_poolStarted) {
            m_threads.reserve(m_scenes.size() - 1);
            m_poolStarted = true;
            for(std::size_t worker = 1; worker < m_scenes.size(); ++worker) {
                try { m_threads.emplace_back([this, worker]() { workerLoop(worker); }); }
                catch(const std::system_error&) { break; }
            }
        }
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_work = work;
            m_count = count;
            m_next.store(0);
            m_activeWorkers = std::min(workers, m_threads.size() + 1);
            m_remaining = m_threads.size();
            m_error = nullptr;
            ++m_generation;
        }
        m_jobReady.notify_all();
        runBatch(0);
        std::unique_lock<std::mutex> lock(m_mutex);
        m_jobDone.wait(lock, [&]() { return m_remaining == 0; });
        m_work = {};
        if(m_error) { std::rethrow_exception(m_error); }
    }

    void CdfQueryBatch::rememberDistance(const std::vector<double>& key, const SignedDistanceSample& sample)
    {
        if(!m_cacheEnabled || !sample.valid) { return; }
        if(m_distances.size() >= kCacheEntries) { m_distances.clear(); }
        auto value = sample;
        // Object IDs belong to one worker's runtime. Cache only the distance
        // observation; never reuse another worker's nearest-feature Jacobian.
        value.hasNearestFeature = false; value.objectA = 0; value.objectB = 0;
        m_distances[key] = std::move(value);
    }

    std::vector<SignedDistanceSample> CdfQueryBatch::distances(const std::vector<std::vector<double>>& path,
        double margin, double horizon, ProjectCdfQpRepairStatistics& statistics)
    {
        const auto started = Clock::now();
        std::vector<SignedDistanceSample> values(path.size());
        std::vector<std::size_t> missing;
        for(std::size_t i = 0; i < path.size(); ++i) {
            const auto found = m_distances.find(distanceKey(path[i], margin, horizon));
            if(m_cacheEnabled && found != m_distances.end()) { values[i] = found->second; ++m_distanceHits; }
            else { missing.push_back(i); }
        }
        std::vector<ProjectCdfQpRepairStatistics> counters(m_scenes.size());
        parallelFor(missing.size(), [&](std::size_t worker, std::size_t offset) {
            const auto i = missing[offset];
            values[i] = evaluateSignedPhi(*m_scenes[worker], path[i], margin, horizon, counters[worker]);
        });
        for(auto i : missing) { rememberDistance(distanceKey(path[i], margin, horizon), values[i]); }
        for(const auto& counter : counters) { statistics.collisionQueries += counter.collisionQueries; }
        m_distanceSeconds += secondsSince(started); return values;
    }

    bool CdfQueryBatch::clearanceValid(const std::vector<std::vector<double>>& path,
        double margin, double horizon, double minimumPhi, ProjectCdfQpRepairStatistics& statistics)
    {
        // A smoothing window is accepted only if EVERY sample passes. Once a
        // sample rejects it, the remaining distances cannot change that decision.
        // Keep distances() for callers that need all values/minima and final QA.
        const auto started = Clock::now();
        std::vector<SignedDistanceSample> values(path.size());
        std::vector<std::size_t> missing;
        for(std::size_t i = 0; i < path.size(); ++i) {
            const auto found = m_distances.find(distanceKey(path[i], margin, horizon));
            if(m_cacheEnabled && found != m_distances.end()) {
                ++m_distanceHits;
                if(!found->second.valid || found->second.phi < minimumPhi) {
                    m_distanceSeconds += secondsSince(started);
                    return false;
                }
            } else { missing.push_back(i); }
        }
        std::atomic<bool> accepted{true};
        std::vector<ProjectCdfQpRepairStatistics> counters(m_scenes.size());
        parallelFor(missing.size(), [&](std::size_t worker, std::size_t offset) {
            if(!accepted.load()) { return; }
            const auto i = missing[offset];
            values[i] = evaluateSignedPhi(*m_scenes[worker], path[i], margin, horizon, counters[worker]);
            if(!values[i].valid || values[i].phi < minimumPhi) { accepted.store(false); }
        });
        for(auto i : missing) { rememberDistance(distanceKey(path[i], margin, horizon), values[i]); }
        for(const auto& counter : counters) { statistics.collisionQueries += counter.collisionQueries; }
        m_distanceSeconds += secondsSince(started);
        return accepted.load();
    }

    std::vector<CdfLinearization> CdfQueryBatch::linearizations(const std::vector<std::vector<double>>& path,
        double margin, double horizon, double step, ProjectCdfQpRepairStatistics& statistics)
    {
        const auto started = Clock::now();
        std::vector<CdfLinearization> values(path.size());
        std::vector<ProjectCdfQpRepairStatistics> counters(m_scenes.size());
        parallelFor(path.size(), [&](std::size_t worker, std::size_t i) {
            auto& scene = *m_scenes[worker];
            values[i] = linearizeSignedPhi(scene, path[i], scene.jointBounds(), margin, horizon, step, counters[worker]);
        });
        for(std::size_t i = 0; i < path.size(); ++i) {
            rememberDistance(distanceKey(path[i], margin, horizon), values[i].sample);
        }
        for(const auto& counter : counters) { statistics.collisionQueries += counter.collisionQueries; }
        m_gradientSeconds += secondsSince(started); return values;
    }

    std::vector<StateValidationResult> CdfQueryBatch::motions(const std::vector<std::vector<double>>& path,
        const MotionValidationOptions& options, bool useCache)
    {
        const auto started = Clock::now();
        const auto count = path.empty() ? 0 : path.size() - 1;
        std::vector<StateValidationResult> values(count);
        std::vector<std::size_t> missing;
        for(std::size_t i = 0; i < count; ++i) {
            const auto found = m_motions.find(motionKey(path[i], path[i + 1], options.maxJointStep));
            if(useCache && m_cacheEnabled && found != m_motions.end()) { values[i] = found->second; ++m_motionHits; }
            else { missing.push_back(i); }
        }
        parallelFor(missing.size(), [&](std::size_t worker, std::size_t offset) {
            const auto i = missing[offset];
            values[i] = m_scenes[worker]->validateMotion(path[i], path[i + 1], options);
        });
        if(m_cacheEnabled && useCache) {
            for(auto i : missing) {
                if(m_motions.size() >= kCacheEntries) { m_motions.clear(); }
                // Cache only reliable valid results, never a transient runtime failure.
                if(values[i].valid) { m_motions[motionKey(path[i], path[i + 1], options.maxJointStep)] = values[i]; }
            }
        }
        m_motionSeconds += secondsSince(started); return values;
    }

    bool CdfQueryBatch::pathValid(const std::vector<std::vector<double>>& path, const MotionValidationOptions& options)
    {
        // Used for small smoothing windows: stop remaining work once any edge is
        // invalid, preserving acceptance while avoiding unnecessary mesh queries.
        const auto started = Clock::now();
        const auto count = path.empty() ? 0 : path.size() - 1;
        std::vector<StateValidationResult> values(count);
        std::vector<std::size_t> missing;
        for(std::size_t i = 0; i < count; ++i) {
            const auto found = m_motions.find(motionKey(path[i], path[i + 1], options.maxJointStep));
            if(m_cacheEnabled && found != m_motions.end()) { ++m_motionHits; }
            else { missing.push_back(i); }
        }
        std::atomic<bool> valid{true};
        parallelFor(missing.size(), [&](std::size_t worker, std::size_t offset) {
            if(!valid.load()) { return; }
            const auto i = missing[offset];
            values[i] = m_scenes[worker]->validateMotion(path[i], path[i + 1], options);
            if(!values[i].valid) { valid.store(false); }
        });
        if(m_cacheEnabled) {
            for(auto i : missing) {
                if(m_motions.size() >= kCacheEntries) { m_motions.clear(); }
                if(values[i].valid) { m_motions[motionKey(path[i], path[i + 1], options.maxJointStep)] = values[i]; }
            }
        }
        m_motionSeconds += secondsSince(started); return valid.load();
    }

    std::string CdfQueryBatch::summary() const
    {
        std::ostringstream out;
        out << "Query workers=" << m_scenes.size() << ", exact distance cache hits=" << m_distanceHits
            << ", exact motion cache hits=" << m_motionHits << ", batch distance seconds=" << m_distanceSeconds
            << ", gradient seconds=" << m_gradientSeconds << ", motion seconds=" << m_motionSeconds << ".";
        return out.str();
    }
}
