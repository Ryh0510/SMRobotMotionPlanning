#include <ProjectMotionPlanning/LayeredIkGraph.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <sstream>
#include <tuple>

namespace motion_planning
{
    namespace
    {
        struct Prefix
        {
            double cost = 0.0;
            std::uint32_t previousNode = 0;
            std::uint32_t previousRank = 0;
        };
        using Layer = std::vector<std::vector<Prefix>>;
        struct Entry
        {
            double cost;
            std::size_t node;
            std::size_t rank;
            bool operator>(const Entry& other) const
            {
                return std::tie(cost, node, rank) > std::tie(other.cost, other.node, other.rank);
            }
        };
        using Heap = std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>>;
    }

    LayeredIkGraphResult ProjectLayeredIkGraph::filter(const CartesianMultiIkResult& ik,
        const LayeredIkGraphOptions& options)
    {
        LayeredIkGraphResult result;
        const auto fail = [&](const std::string& message) {
            result.paths.clear(); result.message = message; return result;
        };
        const auto cancelled = [&]() { return options.cancelled && options.cancelled(); };
        const auto cancel = [&]() { result.cancelled = true; return fail("Layered graph search cancelled."); };
        if(cancelled()) { return cancel(); }
        const auto count = ik.layers.size();
        const auto dimension = ik.source.jointNames.size();
        if(!ik.success || ik.cancelled || count == 0 || dimension == 0 ||
            count != ik.source.cartesianControlPoints.points.size()) {
            return fail("A complete validated multi-IK result is required; no empty layers may be skipped.");
        }
        if(options.maxPaths == 0 || options.maxPaths > 1000 || options.maxStoredPrefixes == 0) {
            return fail("Top-M must be between 1 and 1000 and the prefix budget must be positive.");
        }
        auto weights = options.jointWeights;
        if(weights.empty()) { weights.assign(dimension, 1.0); }
        if(weights.size() != dimension || !std::all_of(weights.begin(), weights.end(),
            [](double w) { return std::isfinite(w) && w >= 0.0; })) {
            return fail("Joint weights must be finite, non-negative and match the joint count.");
        }
        // Exact count of retained prefixes per node, saturated at M. Refuse an
        // oversized job before allocation; never prune edges to satisfy the budget.
        std::size_t previousPathCount = 1, required = 0;
        for(std::size_t i = 0; i < count; ++i) {
            const auto& layer = ik.layers[i];
            if(layer.pointIndex != i || layer.candidates.empty() ||
                layer.candidates.size() > std::numeric_limits<std::uint32_t>::max() || !std::isfinite(layer.time)) {
                return fail("Invalid or empty IK layer at control point " + std::to_string(i + 1));
            }
            for(const auto& candidate : layer.candidates) {
                if(cancelled()) { return cancel(); }
                if(candidate.joints.size() != dimension || candidate.turns.size() != dimension ||
                    !std::all_of(candidate.joints.begin(), candidate.joints.end(), [](double q) { return std::isfinite(q); }) ||
                    !std::isfinite(candidate.positionError) || candidate.positionError < 0 ||
                    !std::isfinite(candidate.orientationError) || candidate.orientationError < 0) {
                    return fail("Invalid candidate data at control point " + std::to_string(i + 1));
                }
            }
            const std::size_t perNode = std::min(options.maxPaths, previousPathCount);
            if(layer.candidates.size() > (options.maxStoredPrefixes - required) / perNode) {
                return fail("Exact Top-M prefix budget exceeded. Reduce M or the multi-IK search range.");
            }
            const auto entries = layer.candidates.size() * perNode;
            required += entries;
            previousPathCount = std::min(options.maxPaths, entries);
        }
        std::vector<Layer> prefixes(count);
        prefixes[0].resize(ik.layers[0].candidates.size());
        for(auto& node : prefixes[0]) { node.push_back({}); }
        if(options.progress) { options.progress(1, count); }
        for(std::size_t i = 1; i < count; ++i) {
            const auto& previous = prefixes[i - 1];
            auto& current = prefixes[i];
            current.resize(ik.layers[i].candidates.size());
            for(std::size_t k = 0; k < current.size(); ++k) {
                if(cancelled()) { return cancel(); }
                std::vector<double> edges(previous.size());
                Heap heap;
                // Each predecessor provides a sorted list shifted by one edge cost.
                // A k-way merge obtains the exact M best prefixes for this node.
                for(std::size_t j = 0; j < previous.size(); ++j) {
                    if(cancelled()) { return cancel(); }
                    const auto& from = ik.layers[i - 1].candidates[j].joints;
                    const auto& to = ik.layers[i].candidates[k].joints;
                    double cost = 0.0;
                    for(std::size_t r = 0; r < dimension; ++r) {
                        if(weights[r] == 0.0) { continue; }
                        const double delta = to[r] - from[r];
                        cost += weights[r] * delta * delta;
                    }
                    const double sum = cost + previous[j][0].cost;
                    if(!std::isfinite(sum)) { return fail("Joint displacement cost overflow."); }
                    edges[j] = cost;
                    heap.push({sum, j, 0});
                }
                auto& best = current[k];
                while(!heap.empty() && best.size() < options.maxPaths) {
                    if(cancelled()) { return cancel(); }
                    const auto entry = heap.top(); heap.pop();
                    best.push_back({entry.cost, static_cast<std::uint32_t>(entry.node), static_cast<std::uint32_t>(entry.rank)});
                    const auto next = entry.rank + 1;
                    if(next < previous[entry.node].size()) {
                        const double cost = edges[entry.node] + previous[entry.node][next].cost;
                        if(!std::isfinite(cost)) { return fail("Accumulated graph cost overflow."); }
                        heap.push({cost, entry.node, next});
                    }
                }
            }
            if(options.progress) { options.progress(i + 1, count); }
        }
        Heap finals;
        const auto& last = prefixes.back();
        for(std::size_t j = 0; j < last.size(); ++j) { finals.push({last[j][0].cost, j, 0}); }
        while(!finals.empty() && result.paths.size() < options.maxPaths) {
            if(cancelled()) { return cancel(); }
            const auto entry = finals.top(); finals.pop();
            LayeredIkPath path;
            path.cost = entry.cost;
            path.selections.resize(count);
            auto node = entry.node, rank = entry.rank;
            for(std::size_t i = count; i-- > 0;) {
                if(cancelled()) { return cancel(); }
                path.selections[i] = node;
                const auto& prefix = prefixes[i][node][rank];
                node = prefix.previousNode; rank = prefix.previousRank;
            }
            result.paths.push_back(std::move(path));
            const auto next = entry.rank + 1;
            if(next < last[entry.node].size()) { finals.push({last[entry.node][next].cost, entry.node, next}); }
        }
        result.success = true;
        std::ostringstream message;
        message << "Top-" << result.paths.size() << " / requested " << options.maxPaths << ", " << count
            << " layers. Complete adjacent-layer connectivity; no collision checks or dynamic limits.";
        if(std::any_of(ik.layers.begin(), ik.layers.end(), [](const auto& layer) { return layer.truncated; })) {
            message << " Input IK enumeration was truncated.";
        }
        result.message = message.str();
        return result;
    }
}
