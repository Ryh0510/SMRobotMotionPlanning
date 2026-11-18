#include <ProjectMotionPlanning/CartesianPathRefinement.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/LayeredIkGraph.h>
#include <ProjectMotionPlanning/TrajectoryQuality.h>
#include "CdfQueryBatch.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <tuple>
#include <limits>

namespace motion_planning {
CartesianPathRefinementResult ProjectCartesianPathRefinementService::refine(
    const simulation_project::ProjectDocument& document, const std::filesystem::path& base,
    const StoredMotionPlan& input, const CartesianPathRefinementOptions& options) const
{
    CartesianPathRefinementResult out;
    const auto began=std::chrono::steady_clock::now();
    const auto elapsed=[&]{return std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();};
    nlohmann::json stats={{"success",false}};
    const auto stop=[&]{return options.cancelled && options.cancelled();};
    const auto progress=[&](const std::string& s){if(options.progress)options.progress(s);};
    try {
        if(input.robotId.empty() || input.jointNames.size()!=6 || input.trajectory.points.size()<3 ||
            input.trajectory.points.size()>options.maximumPointCount || options.maximumPointCount>=3000 ||
            !std::isfinite(options.maximumPositionDeviation) || options.maximumPositionDeviation<0 ||
            !std::isfinite(options.maximumOrientationDeviation) || options.maximumOrientationDeviation<0 ||
            options.maximumOrientationDeviation>1.5707963267948966 ||
            !std::isfinite(options.smoothingSigma) || options.smoothingSigma<=0 || options.smoothingSigma>20 ||
            !std::isfinite(options.bendingWeight) || options.bendingWeight<0 ||
            !std::isfinite(options.horizontalAxisWeight) || options.horizontalAxisWeight<0 ||
            !options.horizontalLocalAxis.allFinite() || std::abs(options.horizontalLocalAxis.norm()-1.)>1e-8 ||
            !options.worldUp.allFinite() || std::abs(options.worldUp.norm()-1.)>1e-8 ||
            !options.horizontalPreservedLocalAxis.allFinite() ||
            (options.horizontalPreservedLocalAxis.norm()>1e-12 &&
                (std::abs(options.horizontalPreservedLocalAxis.norm()-1.)>1e-8 ||
                 std::abs(options.horizontalPreservedLocalAxis.dot(options.horizontalLocalAxis))>1e-8)) ||
            !std::isfinite(options.validationJointStep) || options.validationJointStep<=0 ||
            options.validationJointStep>.001 || options.maximumSearchIterations<1 ||
            options.collisionDetectorIds.empty())
            throw std::runtime_error("Invalid six-axis refinement input/options.");
        ProjectPlanningRequest request;request.robotId=input.robotId;request.jointNames=input.jointNames;
        request.collisionDetectorIds=options.collisionDetectorIds;
        request.enforceJointPositionLimits=options.enforceJointPositionLimits;
        std::string error,tcpLink=options.tcpLinkName;Eigen::Isometry3d linkFromTcp=options.linkFromTcp;
        if(tcpLink.empty() && !resolvePlanningTcp(document,input.robotId,tcpLink,linkFromTcp,error))throw std::runtime_error(error);
        auto scene=ProjectPlanningSceneBuilder::build(document,base,request,&error);
        if(!scene)throw std::runtime_error(error);
        const auto sampler=CartesianMotionSampler::create(*scene,tcpLink,linkFromTcp,error);
        if(!sampler)throw std::runtime_error(error);
        detail::CdfQueryBatch queries(*scene,document,base,request,options.queryWorkers,true,stop);
        queries.setCartesianMotion(sampler);
        const auto count=input.trajectory.points.size();
        const bool horizontal=options.horizontalAxisWeight>0;
        const bool preserveAxis=horizontal && options.horizontalPreservedLocalAxis.norm()>1e-12;
        const auto tilt=[&](const Eigen::Isometry3d& pose) {
            return std::asin(std::clamp(std::abs(options.worldUp.dot(pose.linear()*options.horizontalLocalAxis)),0.,1.));
        };
        std::vector<Eigen::Isometry3d> original(count),targets(count);
        std::vector<Eigen::Quaterniond> rotations(count);
        for(std::size_t i=0;i<count;++i) {
            const auto& p=input.trajectory.points[i];
            if(p.q.size()!=6 || !std::isfinite(p.time) || (i && p.time<=input.trajectory.points[i-1].time))
                throw std::runtime_error("Invalid ordered trajectory station.");
            for(double q:p.q)if(!std::isfinite(q))throw std::runtime_error("Non-finite encoder coordinate.");
            original[i]=sampler->forward(p.q);rotations[i]=Eigen::Quaterniond(original[i].linear());
        }
        // Local quaternion averaging uses each station's own hemisphere, so
        // returning strokes and legal encoder revolutions are never wrapped.
        const int radius=int(std::ceil(4*options.smoothingSigma));
        for(std::size_t i=0;i<count;++i) {
            double weight=0;Eigen::Vector3d position=Eigen::Vector3d::Zero();Eigen::Vector4d quaternion=Eigen::Vector4d::Zero();
            for(int k=-radius;k<=radius;++k) {
                const auto j=std::size_t(std::clamp<long long>(static_cast<long long>(i)+k,0,static_cast<long long>(count)-1));
                const double w=std::exp(-.5*k*k/(options.smoothingSigma*options.smoothingSigma));
                position+=w*original[j].translation();
                const double sign=rotations[j].coeffs().dot(rotations[i].coeffs())<0?-1.:1.;
                quaternion+=w*sign*rotations[j].coeffs();weight+=w;
            }
            targets[i]=original[i];Eigen::Vector3d delta=position/weight-original[i].translation();
            if(delta.norm()>options.maximumPositionDeviation)delta*=options.maximumPositionDeviation/delta.norm();
            targets[i].translation()+=delta;
            if(quaternion.norm()>1e-12) {
                const Eigen::Quaterniond average(quaternion.normalized());
                const double angle=rotations[i].angularDistance(average);
                targets[i].linear()=rotations[i].slerp(angle>options.maximumOrientationDeviation?
                    options.maximumOrientationDeviation/angle:1.,average).toRotationMatrix();
            }
        }
        if(horizontal)for(std::size_t i=0;i<count;++i) {
            if(preserveAxis) {
                const Eigen::Vector3d across=options.horizontalPreservedLocalAxis.cross(options.horizontalLocalAxis);
                const double a=options.worldUp.dot(original[i].linear()*options.horizontalLocalAxis);
                const double b=options.worldUp.dot(original[i].linear()*across);
                const double angle=std::hypot(a,b)<1e-12?0.:std::remainder(std::atan2(-a,b),std::acos(-1.));
                targets[i]=original[i];
                targets[i].linear()=original[i].linear()*Eigen::AngleAxisd(std::clamp(angle,-options.maximumOrientationDeviation,
                    options.maximumOrientationDeviation),options.horizontalPreservedLocalAxis).toRotationMatrix();
                continue;
            }
            const Eigen::Vector3d axis=original[i].linear()*options.horizontalLocalAxis;
            Eigen::Vector3d heading=axis-options.worldUp.dot(axis)*options.worldUp;
            if(heading.norm()<1e-8)heading=options.worldUp.unitOrthogonal();
            heading.normalize();
            const auto swing=Eigen::Quaterniond::FromTwoVectors(axis,heading);
            const double angle=Eigen::Quaterniond::Identity().angularDistance(swing);
            targets[i]=original[i];
            targets[i].linear()=Eigen::Quaterniond::Identity().slerp(angle>options.maximumOrientationDeviation?
                options.maximumOrientationDeviation/angle:1.,swing).toRotationMatrix()*original[i].linear();
        }
        if(options.preserveEndpointEncoders) {targets.front()=original.front();targets.back()=original.back();}
        CartesianMultiIkResult layers;layers.success=true;layers.source=input;
        layers.source.cartesianControlPoints.points.clear();layers.layers.resize(count);
        for(std::size_t i=0;i<count;++i) {
            robottrajectory::TimedCartesianPoint p;p.time=input.trajectory.points[i].time;p.tcpPose=original[i];
            layers.source.cartesianControlPoints.points.push_back(p);
            layers.layers[i].pointIndex=i;layers.layers[i].time=p.time;
        }
        CartesianIkOptions ik;ik.robotId=input.robotId;ik.jointNames=input.jointNames;
        ik.worldForwardKinematics=[sampler](const auto& q){return sampler->forward(q);};
        ik.worldJacobian=[sampler](const auto& q){return sampler->jacobian(q);};
        ik.tolerance=1e-8;ik.maxIterations=200;ik.stepSize=1.;ik.damping=.001;
        if(options.enforceJointPositionLimits)for(const auto& b:scene->jointBounds()){ik.lower.push_back(b.lower);ik.upper.push_back(b.upper);}
        const auto add=[&](std::size_t i,const Eigen::Isometry3d& pose) {
            if(stop())return;
            if((pose.translation()-original[i].translation()).norm()>options.maximumPositionDeviation+1e-10 ||
                Eigen::Quaterniond(pose.linear()).angularDistance(rotations[i])>options.maximumOrientationDeviation+1e-10)return;
            if(horizontal && tilt(pose)>tilt(original[i])+1e-9)return;
            if(preserveAxis && ((pose.linear()-original[i].linear())*options.horizontalPreservedLocalAxis).norm()>1e-8)return;
            StoredMotionPlan one;one.robotId=input.robotId;one.jointNames=input.jointNames;
            robottrajectory::TimedCartesianPoint p;p.time=0.;p.tcpPose=pose;one.cartesianControlPoints.points={p};
            auto localIk=ik;localIk.seedJoints=input.trajectory.points[i].q;
            const auto solved=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,one,localIk);
            if(!solved.success)return;
            const auto& q=solved.plan.trajectory.points.front().q;
            auto& candidates=layers.layers[i].candidates;
            for(const auto& c:candidates) {double d=0;for(int j=0;j<6;++j)d=std::max(d,std::abs(c.joints[j]-q[j]));if(d<1e-7)return;}
            const auto actual=sampler->forward(q);
            if((actual.translation()-pose.translation()).norm()>1e-8 || Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(pose.linear()))>1e-8)return;
            CartesianIkCandidate c;c.joints=q;for(double v:q)c.turns.push_back(int(std::floor((v+std::acos(-1.))/(2*std::acos(-1.)))));
            candidates.push_back(std::move(c));
        };
        progress("Generating ordered pose-fairing candidates");
        queries.kinematics(count,[&](std::size_t i) {
            auto& candidates=layers.layers[i].candidates;
            CartesianIkCandidate source;source.joints=input.trajectory.points[i].q;
            for(double v:source.joints)source.turns.push_back(int(std::floor((v+std::acos(-1.))/(2*std::acos(-1.)))));
            candidates.push_back(std::move(source));
            if(options.preserveEndpointEncoders && (i==0 || i+1==count))return;
            for(double alpha:{1.,.875,.75,.5,.25}) {
                Eigen::Isometry3d target=original[i];
                target.translation()=(1-alpha)*original[i].translation()+alpha*targets[i].translation();
                target.linear()=rotations[i].slerp(alpha,Eigen::Quaterniond(targets[i].linear())).toRotationMatrix();add(i,target);
            }
            if(horizontal) {
                for(double alpha:{.625,.375,.125,.0625,.03125,.015625}) {
                    auto target=original[i];target.linear()=rotations[i].slerp(alpha,Eigen::Quaterniond(targets[i].linear())).toRotationMatrix();add(i,target);
                }
                // Roll freedom about the desired horizontal axis can place a
                // long mounted gun clear of the workpiece without moving TCP.
                const Eigen::Vector3d axis=targets[i].linear()*options.horizontalLocalAxis;
                if(!preserveAxis)for(double roll:{-.7,-.35,.35,.7}) {
                    auto target=targets[i];target.linear()=Eigen::AngleAxisd(roll,axis).toRotationMatrix()*target.linear();add(i,target);
                }
            }
        });
        std::vector<std::vector<bool>> allowed(count);std::vector<std::vector<double>> costs(count);
        const auto checkNodes=[&] {
            std::vector<std::vector<double>> q;std::vector<std::pair<std::size_t,std::size_t>> owners;
            for(std::size_t i=0;i<count;++i)for(std::size_t j=allowed[i].size();j<layers.layers[i].candidates.size();++j) {
                q.push_back(layers.layers[i].candidates[j].joints);owners.emplace_back(i,j);
            }
            const auto checked=queries.states(q,0.);
            for(std::size_t k=0;k<owners.size();++k) {
                const auto i=owners[k].first,j=owners[k].second;
                allowed[i].push_back(checked[k].validation.valid);
                const auto pose=sampler->forward(layers.layers[i].candidates[j].joints);
                const double p=(pose.translation()-targets[i].translation()).norm();
                const double a=Eigen::Quaterniond(pose.linear()).angularDistance(Eigen::Quaterniond(targets[i].linear()));
                costs[i].push_back(4*a*a+400*p*p+options.horizontalAxisWeight*tilt(pose)*tilt(pose));
            }
        };
        checkNodes();stats["candidate_seconds"]=elapsed();stats["iterations"]=nlohmann::json::array();
        std::set<std::tuple<std::size_t,std::size_t,std::size_t>> blocked;
        std::vector<std::vector<double>> originalPath;
        for(const auto& p:input.trajectory.points)originalPath.push_back(p.q);
        const auto parameters=jointPathParameters(originalPath);
        // Pair states retain the preceding encoder vector. Ordinary first-order
        // shortest paths cannot price a change of tangent at the middle station.
        // The reference parameters stay fixed across every candidate comparison.
        const auto smoothPath=[&]() {
            const double infinity=std::numeric_limits<double>::infinity();
            std::vector<std::vector<std::size_t>> back(count);
            const auto edgeCost=[&](std::size_t i,std::size_t a,std::size_t b) {
                if(!allowed[i][a] || !allowed[i+1][b] || blocked.count({i,a,b}))return infinity;
                double value=costs[i+1][b];
                for(int j=0;j<6;++j) {
                    const double d=layers.layers[i+1].candidates[b].joints[j]-layers.layers[i].candidates[a].joints[j];
                    value+=d*d;
                }
                return value;
            };
            const auto first=allowed[0].size(),second=allowed[1].size();
            std::vector<double> previous(first*second,infinity);
            for(std::size_t a=0;a<first;++a)for(std::size_t b=0;b<second;++b)
                previous[a*second+b]=costs[0][a]+edgeCost(0,a,b);
            for(std::size_t i=2;i<count;++i) {
                if(stop())return std::vector<std::size_t>{};
                const auto na=allowed[i-2].size(),nb=allowed[i-1].size(),nc=allowed[i].size();
                std::vector<double> current(nb*nc,infinity);back[i].resize(nb*nc);
                const double left=parameters[i-1]-parameters[i-2],right=parameters[i]-parameters[i-1];
                for(std::size_t b=0;b<nb;++b)for(std::size_t c=0;c<nc;++c) {
                    const double edge=edgeCost(i-1,b,c);if(!std::isfinite(edge))continue;
                    const auto& q=layers.layers[i-1].candidates[b].joints;
                    const auto& r=layers.layers[i].candidates[c].joints;
                    for(std::size_t a=0;a<na;++a) {
                        if(!std::isfinite(previous[a*nb+b]))continue;
                        const auto& p=layers.layers[i-2].candidates[a].joints;
                        double bend=0.;
                        for(int j=0;j<6;++j) {
                            const double dv=(r[j]-q[j])/right-(q[j]-p[j])/left;
                            bend+=2*dv*dv/(left+right);
                        }
                        const double value=previous[a*nb+b]+edge+options.bendingWeight*bend;
                        if(value<current[b*nc+c]){current[b*nc+c]=value;back[i][b*nc+c]=a;}
                    }
                }
                previous=std::move(current);
            }
            const auto best=std::min_element(previous.begin(),previous.end());
            if(best==previous.end() || !std::isfinite(*best))return std::vector<std::size_t>{};
            std::vector<std::size_t> path(count);const auto last=std::size_t(best-previous.begin());
            path[count-1]=last%allowed.back().size();path[count-2]=last/allowed.back().size();
            for(std::size_t i=count-1;i>=2;--i)
                path[i-2]=back[i][path[i-1]*allowed[i].size()+path[i]];
            return path;
        };
        std::vector<bool> expanded(count,false),broadened(count,false);
        std::vector<std::size_t> distanceScored(count,0);
        const auto expand=[&](std::size_t i) {
            if((options.preserveEndpointEncoders && (i==0 || i+1==count)) || expanded[i])return;expanded[i]=true;
            if(preserveAxis) {
                for(double alpha:{.9,.8,.7,.6,.4,.3,.2,.1,.05,.025,.0125,.00625}) {
                    auto p=original[i];p.linear()=rotations[i].slerp(alpha,Eigen::Quaterniond(targets[i].linear())).toRotationMatrix();add(i,p);
                }
                return;
            }
            // A small normal translation often clears an interior tool sweep
            // without undoing the orientation fairing at an entire scan stroke.
            for(double alpha:{1.,.5,0.})for(double sign:{-1.,1.})for(double fraction:{.6,1.}) {
                auto p=original[i];p.linear()=rotations[i].slerp(alpha,Eigen::Quaterniond(targets[i].linear())).toRotationMatrix();
                p.translation()+=sign*fraction*options.maximumPositionDeviation*original[i].linear().col(2);add(i,p);
            }
            for(const auto& center:std::vector<Eigen::Isometry3d>{targets[i],original[i]}) {
                for(int axis=0;axis<3;++axis)for(double sign:{-1.,1.}) {
                    for(double angle:{.04,.10}) {
                        auto p=center;p.linear()=center.linear()*Eigen::AngleAxisd(sign*angle,Eigen::Vector3d::Unit(axis)).toRotationMatrix();add(i,p);
                    }
                }
            }
        };
        const auto broaden=[&](std::size_t i) {
            if((options.preserveEndpointEncoders && (i==0 || i+1==count)) || broadened[i])return;broadened[i]=true;
            if(preserveAxis) {
                for(double angle:{-.34,-.18,-.10,-.04,.04,.10,.18,.34}) {
                    auto p=targets[i];p.linear()=p.linear()*Eigen::AngleAxisd(angle,options.horizontalPreservedLocalAxis).toRotationMatrix();add(i,p);
                }
                return;
            }
            // Only a proven cut in the finite local graph enables the larger
            // angular stencil. Keep the same hard pose corridor and endpoint IK.
            for(const auto& center:std::vector<Eigen::Isometry3d>{targets[i],original[i]})
                for(int axis=0;axis<3;++axis)for(double sign:{-1.,1.})for(double angle:{.18,.28,.34}) {
                    auto p=center;p.linear()=center.linear()*Eigen::AngleAxisd(sign*angle,Eigen::Vector3d::Unit(axis)).toRotationMatrix();add(i,p);
                }
            for(double alpha:{0.,.5,1.})for(int axis=0;axis<2;++axis)for(double sign:{-1.,1.}) {
                auto p=original[i];p.linear()=rotations[i].slerp(alpha,Eigen::Quaterniond(targets[i].linear())).toRotationMatrix();
                p.translation()+=sign*options.maximumPositionDeviation*original[i].linear().col(axis);add(i,p);
            }
        };
        const auto scoreClearance=[&] {
            std::vector<std::vector<double>> values;std::vector<std::pair<std::size_t,std::size_t>> owners;
            for(std::size_t i=0;i<count;++i)if(expanded[i]) {
                for(std::size_t j=distanceScored[i];j<layers.layers[i].candidates.size();++j) {
                    values.push_back(layers.layers[i].candidates[j].joints);owners.emplace_back(i,j);
                }
                distanceScored[i]=layers.layers[i].candidates.size();
            }
            ProjectCdfQpRepairStatistics counters;
            const auto observations=queries.distances(values,0.,1.,counters);
            for(std::size_t k=0;k<owners.size();++k)if(observations[k].valid && std::isfinite(observations[k].rawDistance))
                costs[owners[k].first][owners[k].second]+=.0001/(.001+std::max(0.,observations[k].rawDistance));
        };
        MotionValidationOptions validation;validation.maxJointStep=options.validationJointStep;
        validation.preserveJointTurns=true;validation.maxSamples=1000000;validation.cancelled=stop;
        auto screening=validation;screening.maxJointStep=std::max(.004,validation.maxJointStep);
        stats["preflight_joint_step_rad"]=screening.maxJointStep;
        stats["batched_candidate_pairs"]=0;
        std::vector<std::size_t> failureCounts(count-1,0),screenedWidths(count-1,0);
        std::vector<std::size_t> selected;
        for(int iteration=0;iteration<options.maximumSearchIterations && !stop();++iteration) {
            for(std::size_t i=0;i+1<count;++i) {
                const auto width=allowed[i].size()*allowed[i+1].size();
                if(failureCounts[i]<(horizontal?8:32) || screenedWidths[i]==width)continue;
                screenedWidths[i]=width;
                std::vector<std::pair<std::vector<double>,std::vector<double>>> pairs;
                std::vector<std::pair<std::size_t,std::size_t>> owners;
                for(std::size_t a=0;a<allowed[i].size();++a)if(allowed[i][a])
                    for(std::size_t b=0;b<allowed[i+1].size();++b)if(allowed[i+1][b] && !blocked.count({i,a,b})) {
                        pairs.emplace_back(layers.layers[i].candidates[a].joints,layers.layers[i+1].candidates[b].joints);
                        owners.emplace_back(a,b);
                    }
                if(pairs.empty())continue;
                progress("Batch screening "+std::to_string(pairs.size())+" candidate pairs at edge "+std::to_string(i));
                auto checked=queries.independentMotions(pairs,screening);
                std::vector<std::pair<std::vector<double>,std::vector<double>>> retry;
                std::vector<std::size_t> retryIndices;
                for(std::size_t k=0;k<checked.size();++k)if(checked[k].diagnosticCode=="cartesian_ik_failed") {
                    retry.push_back(pairs[k]);retryIndices.push_back(k);
                }
                const auto retried=queries.independentMotions(retry,validation);
                for(std::size_t k=0;k<retried.size();++k)checked[retryIndices[k]]=retried[k];
                for(std::size_t k=0;k<checked.size();++k)if(!checked[k].valid) {
                    if(checked[k].diagnosticCode!="state_in_collision" && checked[k].diagnosticCode!="cartesian_ik_failed")
                        throw std::runtime_error(checked[k].message);
                    blocked.emplace(i,owners[k].first,owners[k].second);
                }
                stats["batched_candidate_pairs"]=stats["batched_candidate_pairs"].get<std::size_t>()+pairs.size();
            }
            LayeredIkGraphOptions graph;graph.maxPaths=1;graph.cancelled=stop;
            graph.nodeAdmissible=[&](std::size_t i,const auto& c){return allowed[i][&c-layers.layers[i].candidates.data()];};
            graph.nodeCost=[&](std::size_t i,const auto& c){return costs[i][&c-layers.layers[i].candidates.data()];};
            graph.edgeAdmissible=[&](std::size_t i,const auto& a,const auto& b) {
                return !blocked.count({i,std::size_t(&a-layers.layers[i].candidates.data()),std::size_t(&b-layers.layers[i+1].candidates.data())});
            };
            LayeredIkGraphResult solution;
            if(options.bendingWeight>0) {
                auto path=smoothPath();solution.success=!path.empty();
                if(solution.success){LayeredIkPath p;p.selections=std::move(path);solution.paths.push_back(std::move(p));}
                else solution.message="No connected smooth candidate path remains.";
            } else solution=ProjectLayeredIkGraph::filter(layers,graph);
            if(!solution.success) {
                // Find the first cut in the currently known graph. Expanding
                // unrelated blocked edges creates thousands of attractive but
                // untested variants and delays recovery of one local interval.
                auto reachable=allowed.front();bool changed=false;
                for(std::size_t i=1;i<count;++i) {
                    std::vector<bool> next(allowed[i].size(),false);
                    for(std::size_t j=0;j<next.size();++j)if(allowed[i][j])
                        for(std::size_t k=0;k<reachable.size();++k)if(reachable[k] && !blocked.count({i-1,k,j})){next[j]=true;break;}
                    if(std::none_of(next.begin(),next.end(),[](bool v){return v;})) {
                        for(auto station:{i-1,i})if(!options.preserveEndpointEncoders || (station && station+1<count)) {
                            if(!expanded[station]){expand(station);changed=true;}
                            else if(!broadened[station]){broaden(station);changed=true;}
                        }
                        break;
                    }
                    reachable=std::move(next);
                }
                if(!changed)throw std::runtime_error(solution.message);
                checkNodes();scoreClearance();continue;
            }
            selected=solution.paths.front().selections;std::vector<std::vector<double>> path;
            for(std::size_t i=0;i<count;++i)path.push_back(layers.layers[i].candidates[selected[i]].joints);
            progress("Screening MoveL candidate "+std::to_string(iteration+1));
            auto checks=queries.motions(path,screening,true);
            // A sparse numerical IK failure is not an infeasibility certificate:
            // retry it with the original full-resolution connection before pruning.
            for(std::size_t i=0;i<checks.size();++i)if(checks[i].diagnosticCode=="cartesian_ik_failed")
                checks[i]=queries.motions({path[i],path[i+1]},validation,true).front();
            const bool fullResolution=std::all_of(checks.begin(),checks.end(),[](const auto& r){return r.valid;});
            if(fullResolution) {
                progress("Full precision MoveL verification of candidate "+std::to_string(iteration+1));
                checks=queries.motions(path,validation,true,[&](std::size_t done,std::size_t total) {
                progress("MoveL edges "+std::to_string(done)+"/"+std::to_string(total));
                });
            }
            std::vector<std::size_t> failures;
            for(std::size_t i=0;i<checks.size();++i)if(!checks[i].valid) {
                if(checks[i].diagnosticCode!="state_in_collision" && checks[i].diagnosticCode!="cartesian_ik_failed")
                    throw std::runtime_error(checks[i].message);
                blocked.emplace(i,selected[i],selected[i+1]);failures.push_back(i);
                ++failureCounts[i];
            }
            stats["iterations"].push_back({{"iteration",iteration+1},{"phase",fullResolution?"full_precision":"screening"},
                {"failed_edges",failures},{"seconds",elapsed()}});
            progress("Candidate "+std::to_string(iteration+1)+": "+std::to_string(failures.size())+" blocked edges");
            if(!failures.empty())progress("First blocked edge (zero-based): "+std::to_string(failures.front()));
            if(failures.empty()) {
                stats["full_resolution_validated"]=fullResolution;
                out.plan=input;out.plan.cartesianControlPoints.points.clear();
                for(std::size_t i=0;i<count;++i) {
                    out.plan.trajectory.points[i].q=std::move(path[i]);
                    out.plan.trajectory.points[i].qd.clear();out.plan.trajectory.points[i].qdd.clear();
                    robottrajectory::TimedCartesianPoint p;p.time=out.plan.trajectory.points[i].time;
                    p.tcpPose=sampler->forward(out.plan.trajectory.points[i].q);
                    out.plan.cartesianControlPoints.points.push_back(p);
                }
                auto metadata=nlohmann::json::parse(input.optimizationMetadata,nullptr,false);
                if(!metadata.is_object())metadata=nlohmann::json::object();
                metadata["motion"]="cartesian_linear";metadata["joint_convention"]="runtime";
                metadata["refinement"]="ordered_collision_constrained_pose_fairing";
                out.plan.optimizationMetadata=metadata.dump();out.success=true;break;
            }
        }
        if(!out.success)throw std::runtime_error(stop()?"Refinement cancelled.":"Pose candidate search exhausted; no unverified output returned.");
        double maxPosition=0,maxAngle=0;
        for(std::size_t i=0;i<count;++i) {
            const auto pose=sampler->forward(out.plan.trajectory.points[i].q);
            maxPosition=std::max(maxPosition,(pose.translation()-original[i].translation()).norm());
            maxAngle=std::max(maxAngle,Eigen::Quaterniond(pose.linear()).angularDistance(rotations[i]));
        }
        stats["maximum_station_position_deviation_m"]=maxPosition;stats["maximum_station_orientation_deviation_rad"]=maxAngle;
        stats["input_points"]=count;stats["output_points"]=out.plan.trajectory.points.size();
        stats["endpoint_encoders_exact"]=out.plan.trajectory.points.front().q==input.trajectory.points.front().q &&
            out.plan.trajectory.points.back().q==input.trajectory.points.back().q;
        stats["query_profile"]=queries.summary();stats["validation_joint_step_rad"]=validation.maxJointStep;
        stats["bending_weight"]=options.bendingWeight;
        if(horizontal) {
            double before=0,after=0,maximum=0;std::size_t withinOne=0;
            for(std::size_t i=0;i<count;++i) {
                before+=tilt(original[i]);const double value=tilt(sampler->forward(out.plan.trajectory.points[i].q));
                after+=value;maximum=std::max(maximum,value);if(value<=std::acos(-1.)/180.)++withinOne;
            }
            stats["horizontal"]={{"weight",options.horizontalAxisWeight},{"local_axis",{options.horizontalLocalAxis.x(),options.horizontalLocalAxis.y(),options.horizontalLocalAxis.z()}},
                {"preserved_local_axis",{options.horizontalPreservedLocalAxis.x(),options.horizontalPreservedLocalAxis.y(),options.horizontalPreservedLocalAxis.z()}},
                {"world_up",{options.worldUp.x(),options.worldUp.y(),options.worldUp.z()}},{"mean_input_tilt_deg",before/count*180/std::acos(-1.)},
                {"mean_output_tilt_deg",after/count*180/std::acos(-1.)},{"maximum_output_tilt_deg",maximum*180/std::acos(-1.)},{"within_one_degree",withinOne}};
        }
        out.message="All ordered pose stations and full MoveL connections verified.";
    } catch(const std::exception& e) {out.success=false;out.plan={};out.message=e.what();}
    stats["success"]=out.success;stats["total_seconds"]=elapsed();stats["message"]=out.message;
    out.statistics=stats.dump();return out;
}
}
