#include <ProjectMotionPlanning/SafeTrajectoryDownsampling.h>
#include <algorithm>
#include <cmath>
#include <set>
#include <queue>

namespace motion_planning
{
    SafeTrajectoryDownsamplingResult downsampleTrajectorySafely(
        const robottrajectory::JointTrajectory& source,
        const SafeTrajectoryDownsamplingOptions& options)
    {
        SafeTrajectoryDownsamplingResult result;
        const auto& points=source.points;
        const auto stop=[&]() {return options.cancelled && options.cancelled();};
        const auto fail=[&](const std::string& message) {
            result.success=false;result.cancelled=stop();result.message=message;
            result.trajectory={};result.sourceIndices.clear();return result;
        };
        if(points.size()<2 || options.targetPoints<2 || !options.edgeAdmissible ||
            source.interpolation!=robottrajectory::TrajectoryInterpolation::Linear)
            return fail("At least two points, a target >= 2 and an edge validator are required.");
        const auto dimension=points.front().q.size();double last=-1;
        for(const auto& point:points) {
            if(stop())return fail("Downsampling cancelled.");
            if(dimension==0 || point.q.size()!=dimension || !std::isfinite(point.time) || point.time<=last ||
                !std::all_of(point.q.begin(),point.q.end(),[](double q){return std::isfinite(q);}))
                return fail("Invalid joint trajectory or source clock.");
            last=point.time;
        }
        const auto target=std::min(options.targetPoints,points.size());
        std::set<std::size_t> seeds;
        for(std::size_t i=0;i<target;++i)seeds.insert(i*(points.size()-1)/(target-1));
        for(auto i:options.requiredIndices) {
            if(i>=points.size())return fail("Required source index is out of range.");
            seeds.insert(i);
        }
        std::vector<std::size_t> indices(seeds.begin(),seeds.end());
        std::vector<std::pair<std::size_t,std::size_t>> pending;
        for(std::size_t i=indices.size()-1;i>0;--i)pending.emplace_back(indices[i-1],indices[i]);
        result.sourceIndices.push_back(0);
        while(!pending.empty()) {
            if(stop())return fail("Downsampling cancelled.");
            const auto edge=pending.back();pending.pop_back();++result.checkedEdges;
            const bool accepted=options.edgeAdmissible(edge.first,edge.second);
            if(stop())return fail("Downsampling cancelled.");
            if(accepted)result.sourceIndices.push_back(edge.second);
            else {
                if(edge.second==edge.first+1)return fail("An original edge did not pass the configured motion checks.");
                // Restore the source bend with largest joint/time chord error.
                // No wrapped distance: legal encoder turns remain meaningful.
                auto split=edge.first+(edge.second-edge.first)/2;double worst=0;
                const auto& a=points[edge.first];const auto& b=points[edge.second];
                for(auto i=edge.first+1;i<edge.second;++i) {
                    const double f=(points[i].time-a.time)/(b.time-a.time);double error=0;
                    for(std::size_t j=0;j<dimension;++j) {
                        const double d=points[i].q[j]-((1-f)*a.q[j]+f*b.q[j]);error+=d*d;
                    }
                    if(error>worst){worst=error;split=i;}
                }
                pending.emplace_back(split,edge.second);pending.emplace_back(edge.first,split);
            }
            if(options.progress && (result.checkedEdges%100==0 || pending.empty()))
                options.progress(result.sourceIndices.size(),pending.size());
        }
        // Uniform seeds are a density preference, not mandatory bends. Recover
        // that budget after local safety refinement by removing the least bent
        // remaining node, ONLY when its new neighbor-to-neighbor motion passes.
        // This avoids accumulating uniform seeds plus all restored safety nodes.
        if(result.sourceIndices.size()>target) {
            const auto none=points.size();
            std::vector<std::size_t> previous(points.size(),none),next(points.size(),none);
            std::vector<bool> alive(points.size(),false),pinned(points.size(),false);
            pinned.front()=pinned.back()=true;for(auto i:options.requiredIndices)pinned[i]=true;
            for(std::size_t i=0;i<result.sourceIndices.size();++i) {
                const auto current=result.sourceIndices[i];alive[current]=true;
                if(i)previous[current]=result.sourceIndices[i-1];
                if(i+1<result.sourceIndices.size())next[current]=result.sourceIndices[i+1];
            }
            struct Removal {double error;std::size_t point,from,to;};
            const auto later=[](const Removal& a,const Removal& b) {
                return a.error!=b.error?a.error>b.error:a.point>b.point;
            };
            std::priority_queue<Removal,std::vector<Removal>,decltype(later)> queue(later);
            const auto offer=[&](std::size_t i) {
                if(i==none || !alive[i] || pinned[i] || previous[i]==none || next[i]==none)return;
                const auto a=previous[i],b=next[i];double error=0;
                const double f=(points[i].time-points[a].time)/(points[b].time-points[a].time);
                for(std::size_t j=0;j<dimension;++j) {
                    const double d=points[i].q[j]-((1-f)*points[a].q[j]+f*points[b].q[j]);error+=d*d;
                }
                queue.push({error,i,a,b});
            };
            for(auto i:result.sourceIndices)offer(i);
            auto count=result.sourceIndices.size();
            while(count>target && !queue.empty()) {
                if(stop())return fail("Downsampling cancelled.");
                const auto item=queue.top();queue.pop();
                if(!alive[item.point] || previous[item.point]!=item.from || next[item.point]!=item.to)continue;
                ++result.checkedEdges;
                const bool accepted=options.edgeAdmissible(item.from,item.to);
                if(stop())return fail("Downsampling cancelled.");
                if(accepted) {
                    alive[item.point]=false;next[item.from]=item.to;previous[item.to]=item.from;--count;
                    offer(item.from);offer(item.to);
                }
                if(options.progress && result.checkedEdges%100==0)options.progress(count,queue.size());
            }
            result.sourceIndices.clear();
            for(auto i=std::size_t(0);i!=none;i=next[i])result.sourceIndices.push_back(i);
        }
        result.trajectory=source;result.trajectory.points.clear();
        for(auto i:result.sourceIndices) {
            auto point=points[i];point.qd.clear();point.qdd.clear();
            result.trajectory.points.push_back(std::move(point));
        }
        result.success=true;result.message=result.sourceIndices.size()>options.targetPoints?
            "Additional source points retained for safe connections.":"Target reached with checked connections.";
        return result;
    }
}
