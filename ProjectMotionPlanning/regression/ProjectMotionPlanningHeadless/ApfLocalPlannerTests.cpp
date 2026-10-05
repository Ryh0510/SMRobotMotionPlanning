#include "../../src/ApfLocalPlanner.h"
#include "../../src/PathRefinement.h"
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include <SimulationProject/ProjectDocument.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace motion_planning::detail;

int main()
{
    motion_planning::ProjectCdfQpTrajectoryRepairService service;
    simulation_project::ProjectDocument document;
    robottrajectory::JointTrajectory dummy;
    const auto missing = service.repair(document, {}, "robot", {"joint"}, dummy);
    if(missing.success || !missing.plan.trajectory.empty() || missing.diagnostics.empty() ||
        missing.diagnostics.front().code != "apf_tcp_model_missing") return 26;
    ApfOracle oracle;
    oracle.distance = [](const ApfState& q) { return std::hypot(q[0], q[1]) - 0.3; };
    oracle.motionValid = [&](const ApfState& a, const ApfState& b) {
        for(int i = 0; i <= 400; ++i) {
            ApfState q = a;
            for(std::size_t j = 0; j < q.size(); ++j) q[j] += (b[j] - a[j]) * i / 400.0;
            if(oracle.distance(q) < 0.0) return false;
        }
        return true;
    };
    const ApfPath reference{{-1.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}};
    const ApfState lower{-1.5, -1.5}, upper{1.5, 1.5};
    ApfPath path, repeat;
    if(!planApfPath(reference, lower, upper, oracle, 0.02, &path)) {
        std::cerr << "Failed symmetric obstacle / tangential escape\n";
        return 1;
    }
    if(path.front() != reference.front() || path.back() != reference.back()) return 2;
    for(std::size_t i = 0; i + 1 < path.size(); ++i)
        if(!oracle.motionValid(path[i], path[i + 1])) return 3;
    if(!planApfPath(reference, lower, upper, oracle, 0.02, &repeat) || repeat != path) return 4;
    ApfOracle batched = oracle;
    std::size_t batchCalls = 0;
    batched.distances = [&](const ApfPath& points) {
        ++batchCalls;
        std::vector<double> values;
        for(const auto& q : points) values.push_back(oracle.distance(q));
        return values;
    };
    if(!planApfPath(reference, lower, upper, batched, 0.02, &repeat) || repeat != path || batchCalls == 0) return 15;
    batched.distances = [](const ApfPath&) { return std::vector<double>{}; };
    if(planApfPath(reference, lower, upper, batched, 0.02, &repeat) || !repeat.empty()) return 16;
    // Smooth a noisy arc around the obstacle. Every accepted edge must still
    // clear the circle, even though its endpoint chord intersects the circle.
    ApfPath arc;
    std::vector<double> times;
    for(int i = 0; i <= 64; ++i) {
        const double angle = 3.14159265358979323846 * i / 64.0;
        const double radius = 0.38 + 0.025 * std::sin(13.0 * angle);
        arc.push_back({-radius * std::cos(angle), radius * std::sin(angle)});
        times.push_back(i * 0.1 + i * i * 0.0001);
    }
    const auto initialArc = arc;
    const double initialBending = jointPathBending(arc, times);
    if(!smoothValidatedPath(&arc, times, oracle, 3, 0.5, 0.15) ||
        jointPathBending(arc, times) >= initialBending ||
        jointPathLength(arc) > jointPathLength(initialArc) + 1.0e-10 ||
        arc.front() != initialArc.front() || arc.back() != initialArc.back()) return 11;
    for(std::size_t i = 1; i < arc.size(); ++i)
        if(!oracle.motionValid(arc[i - 1], arc[i])) return 12;
    const auto preserved = arc;
    ApfOracle blocked = oracle;
    blocked.motionValid = [](const auto&, const auto&) { return false; };
    if(smoothValidatedPath(&arc, times, blocked, 3, 0.5, 0.15) || arc != preserved) return 13;
    times[1] = times[0];
    if(smoothValidatedPath(&arc, times, oracle, 3, 0.5, 0.15) || arc != preserved) return 14;
    if(planApfPath({{0.0, 0.0}, {1.0, 0.0}}, lower, upper, oracle, 0.02, &path) || !path.empty()) return 5;
    oracle.distance = [](const ApfState& q) { return std::abs(q[0]) - 0.3; };
    if(planApfPath(reference, lower, upper, oracle, 0.02, &path) || !path.empty()) return 6;
    oracle.distance = [](const ApfState&) { return std::numeric_limits<double>::quiet_NaN(); };
    if(planApfPath(reference, lower, upper, oracle, 0.02, &path)) return 7;
    oracle.distance = [](const ApfState&) { return 1.0; };
    if(!planApfPath(reference, lower, upper, oracle, 0.02, &path) || path != reference) return 8;
    // Free-space returning strokes must retain every original station and turn.
    const ApfPath zigzag{{-1,0},{1,0},{1,0.2},{-1,0.2},{-1,0.4},{1,0.4}};
    if(!planApfPath(zigzag, lower, upper, oracle, 0.02, &path) || path != zigzag) return 17;
    const ApfPath turns{{6.2,0},{6.4,0},{6.6,0}};
    if(!planApfPath(turns, {5.0,-1.0}, {8.0,1.0}, oracle, 0.02, &path) || path != turns) return 18;
    ApfGuidance guide;
    guide.position = [](const ApfState& q) { return q; };
    guide.maxDeviation = 0.05;
    // Endpoints alone cannot certify a nonlinear FK segment.
    ApfGuidance nonlinear = guide;
    nonlinear.position = [](const auto& q) { return ApfState{q[0], 0.1 * std::sin(3.14159265358979323846 * q[0])}; };
    if(followsApfGuide({0,0}, {1,0}, {0,0}, {1,0}, nonlinear)) return 19;
    nonlinear.position = [](const auto&) { return ApfState{std::numeric_limits<double>::quiet_NaN(),0}; };
    if(followsApfGuide({0,0}, {1,0}, {0,0}, {1,0}, nonlinear)) return 20;
    ApfPath scan;
    for(int i=0; i<=100; ++i) scan.push_back({-0.1+0.002*i, 0.0});
    guide.positions = scan;
    oracle.distance = [](const auto& q) { return std::hypot(q[0],q[1]) - 0.025; };
    if(!planApfPath(scan, {-0.2,-0.2}, {0.2,0.2}, oracle, 0.005, &path, &guide) ||
        path.size() != scan.size() || !followsApfPath(path, guide)) {
        std::cerr << "Failed feasible 50 mm ordered detour\n"; return 21;
    }
    for(std::size_t i=1;i<path.size();++i) if(!oracle.motionValid(path[i-1],path[i])) return 22;
    bool anticipates=false;
    for(std::size_t i=1;i+1<path.size();++i)
        if(scan[i][0]<-0.035 && std::abs(path[i][1])>0.003)anticipates=true;
    if(!anticipates){std::cerr<<"Detour starts too close to contact\n";return 28;}
    const auto shaped = path;
    if(!planApfPath(scan, {-0.2,-0.2}, {0.2,0.2}, oracle, 0.005, &repeat, &guide) || shaped != repeat) return 23;
    std::reverse(scan.begin(),scan.end()); guide.positions=scan;
    if(!planApfPath(scan, {-0.2,-0.2}, {0.2,0.2}, oracle, 0.005, &repeat, &guide) ||
        repeat.front()!=scan.front() || repeat.back()!=scan.back() || !followsApfPath(repeat,guide)) return 27;
    std::reverse(scan.begin(),scan.end()); guide.positions=scan;
    guide.maxDeviation = 0.015;
    if(planApfPath(scan, {-0.2,-0.2}, {0.2,0.2}, oracle, 0.005, &path, &guide) || !path.empty()) return 24;
    // Smoothing window keeps its absolute station correspondence, even on
    // returning strokes. Wrong whole-path nearest-point association is forbidden.
    ApfOracle shapeSmoothing = oracle;
    shapeSmoothing.pathValid = [&](const auto& candidate, std::size_t begin) {
        guide.maxDeviation = 0.05;
        if(!followsApfPath(candidate, guide, begin)) return false;
        for(std::size_t i=1;i<candidate.size();++i) if(!oracle.motionValid(candidate[i-1],candidate[i])) return false;
        return true;
    };
    path = shaped; times.clear(); for(std::size_t i=0;i<path.size();++i) times.push_back(0.1*i);
    smoothValidatedPath(&path, times, shapeSmoothing, 3, 0.5, 0.15);
    if(!followsApfPath(path, guide)) return 25;
    // Guided refinement can leave a noisy APF seed's tiny joint box, while
    // preserving ordered stations, fixed anchors and every collision constraint.
    path=shaped;const auto guidedBefore=path;
    const double guidedBending=jointPathBending(path,times);
    if(!smoothValidatedPath(&path,times,shapeSmoothing,3,1.0,0.00001,true) ||
        jointPathBending(path,times)>=guidedBending || !followsApfPath(path,guide) ||
        path.front()!=guidedBefore.front() || path.back()!=guidedBefore.back())return 30;
    double movement=0;
    for(std::size_t i=0;i<path.size();++i)for(std::size_t j=0;j<path[i].size();++j)
        movement=std::max(movement,std::abs(path[i][j]-guidedBefore[i][j]));
    if(movement<=0.00001)return 31;
    const auto guidedPreserved=path;
    auto rejectGuided=shapeSmoothing;
    rejectGuided.pathValid=[](const auto&,std::size_t){return false;};
    if(smoothValidatedPath(&path,times,rejectGuided,3,1.0,0.00001,true) || path!=guidedPreserved)return 32;
    rejectGuided.pathValid={};
    if(smoothValidatedPath(&path,times,rejectGuided,3,1.0,0.00001,true) || path!=guidedPreserved)return 33;
    // A wrist transition has a large TCP arc in joint-linear interpolation.
    // Correct its positional seed with actual FK, retaining endpoints and turns.
    ApfGuidance wrist;
    wrist.maxDeviation=0.05;
    wrist.position=[](const auto& q){return ApfState{q[0]+0.2*std::sin(q[2]),q[1]+0.2*std::cos(q[2])};};
    ApfPath wristReference;
    for(int i=0;i<=50;++i) {
        const double t=i/50.0;
        wristReference.push_back({t,0,t*3.14159265358979323846});
        wrist.positions.push_back({t,0.2-0.4*t});
    }
    ApfOracle wristOracle;
    wristOracle.distance=[](const auto&){return 1.0;};
    wristOracle.motionValid=[](const auto&,const auto&){return true;};
    if(followsApfPath(wristReference,wrist) ||
        !planApfPath(wristReference,{-1,-1,-1},{2,1,4},wristOracle,0.005,&path,&wrist) ||
        path.front()!=wristReference.front() || path.back()!=wristReference.back() ||
        !followsApfPath(path,wrist)){std::cerr<<"Nonlinear wrist seed projection failed\n";return 29;}
    auto projected=ApfState{0.5,0.2,1.5};
    if(!projectApfPosition(projected,{0.5,0},{-1,-1,-1},{2,1,4},wrist,0.04) ||
        !followsApfGuide(projected,projected,{0.5,0},{0.5,0},wrist))return 34;
    if(projectApfPosition(projected,{0.5,0},{1,-1,-1},{-1,1,4},wrist,0.04))return 35;
    // TCP shape restoration must undo the crossing of two returning strokes,
    // preserve unwrapped turns, and never publish a collision-limited proposal.
    ApfGuidance lanes;lanes.maxDeviation=0.10;
    lanes.position=[](const auto& q){return ApfState{q[0],q[1],q[2]};};
    ApfPath bowed;
    for(int i=0;i<=200;++i) {
        const double t=i/200.0,x=-0.2+0.4*t;
        lanes.positions.push_back({x,0,0});
        bowed.push_back({x,0.07*std::sin(3.14159265358979323846*t),0,6.4});
    }
    const auto bowedOriginal=bowed;
    ApfOracle restore;
    restore.distance=[](const auto& q){return std::hypot(q[0],q[1])-0.03;};
    restore.distances=[&](const auto& p){std::vector<double> v;for(const auto& q:p)v.push_back(restore.distance(q));return v;};
    restore.motionValid=[&](const auto& a,const auto& b){for(int k=0;k<=50;++k){auto q=a;
        for(std::size_t j=0;j<q.size();++j)q[j]+=(b[j]-a[j])*k/50.0;if(restore.distance(q)<0)return false;}return true;};
    restore.pathValid=[&](const auto& p,std::size_t begin){if(!followsApfPath(p,lanes,begin))return false;
        for(std::size_t i=1;i<p.size();++i)if(!restore.motionValid(p[i-1],p[i]))return false;return true;};
    if(!restoreApfTcpShape(&bowed,{-1,-1,-1,5},{1,1,1,8},restore,lanes) ||
        bowed.front()!=bowedOriginal.front() || bowed.back()!=bowedOriginal.back() || !restore.pathValid(bowed,0))return 36;
    double oldError=0,newError=0;
    for(std::size_t i=0;i<bowed.size();++i){oldError+=std::abs(bowedOriginal[i][1]);newError+=std::abs(bowed[i][1]);if(bowed[i][3]!=6.4)return 37;}
    if(newError>=oldError)return 38;
    const auto safeBowed=bowed;
    restore.motionsValid=[](const auto& p){return std::vector<bool>(p.size()-1,false);};
    if(restoreApfTcpShape(&bowed,{-1,-1,-1,5},{1,1,1,8},restore,lanes) || bowed!=safeBowed)return 39;
    restore.motionsValid={};restore.distance=[](const auto&){return 1.0;};
    lanes.positions.clear();bowed.clear();
    for(int i=0;i<=200;++i){const double t=(i<=100?i:200-i)/100.0,y=i<=100?0.0:0.04;
        lanes.positions.push_back({t,y,0});bowed.push_back({t,y+(i<=100?1:-1)*0.06*std::sin(3.14159265358979323846*t),0,6.4});}
    if(!restoreApfTcpShape(&bowed,{-1,-1,-1,5},{2,1,1,8},restore,lanes))return 40;
    for(std::size_t i=5;i<95;++i)if(bowed[i][1]>=bowed[200-i][1])return 41;
    std::cout << "PASS APF: free path, obstacle, fixed anchors, deterministic escape, blocked corridor, invalid oracle, ordered zigzag, turn preservation, hard TCP corridor, nonlinear FK segment, time-aware smoothing\n";
    return 0;
}
