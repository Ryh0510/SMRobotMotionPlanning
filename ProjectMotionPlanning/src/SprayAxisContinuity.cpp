#include <ProjectMotionPlanning/SprayAxisContinuity.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/TrajectoryInverseKinematics.h>
#include <ProjectMotionPlanning/VirtualBaffleEnvironment.h>
#include <RobotInstance/RobotInstance.h>
#include "CdfQueryBatch.h"
#include "SpraySurfaceReference.h"
#include "SprayMoveLBounds.h"
#include "SprayPoseConstraint.h"
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <numeric>
#include <map>
#include <set>
#include <random>

namespace motion_planning {
namespace {
using Json = nlohmann::json;
struct Knot { double station=0; std::vector<double> q; };
// Freeze detector geometry once per isolated worker. Reusing a frozen snapshot
// avoids rebuilding the complete CAD/attachment motion scene for every chord.
class MoveLCertificate {
    ProjectPlanningSceneSnapshot& m_scene;
    VirtualBaffleEnvironment m_baffles;
    bool m_hasBaffles=false;
    std::vector<collision::CollisionMotionSnapshot> m_snapshots;
    std::vector<std::vector<std::array<double,6>>> m_levers;
    std::vector<std::vector<bool>> m_tcpRigid;
    std::vector<std::vector<std::shared_ptr<const detail::MeshSeparation>>> m_meshes;
    const CartesianMotionSampler& m_sampler;
    double m_reach;
    std::array<int,6> m_depth{};
    struct DistanceCache {double distance=-1;collision::Transform3 a,b;};
    std::vector<std::vector<DistanceCache>> m_distanceCache;
    collision::MotionQueryRequest m_request;
    std::function<void(const std::string&)> m_progress;
    bool m_trace=true;
    double m_collisionParameter=.5;
public:
    double collisionParameter() const {return m_collisionParameter;}
    MoveLCertificate(ProjectPlanningSceneSnapshot& scene,const CartesianMotionSampler& sampler,double reach,const SprayAxisContinuityOptions& o):m_scene(scene),m_sampler(sampler),m_reach(reach) {
        m_progress=o.progress;
        const auto baffleStatus=Json::parse(scene.virtualBaffleStatus());
        if(!baffleStatus.value("error",std::string()).empty())throw std::runtime_error("Virtual baffle geometry unavailable.");
        for(const auto& b:baffleStatus.at("objects"))m_hasBaffles |= b.value("enabled",false);
        if(m_hasBaffles) {
            m_baffles.build(scene.simulationRuntime());
            if(!m_baffles.status().value("error",std::string()).empty())throw std::runtime_error("Virtual baffle motion geometry unavailable.");
            if(m_progress)m_progress("Continuous virtual-baffle joint-ball bounds enabled (all robot links and mounted tools)");
        }
        m_request.clearance=o.clearance;m_request.maxEvaluations=131072;m_request.maxPairTests=1000000;
        m_request.minParameterSpan=1e-9;m_request.cancelled=o.cancelled;
        const auto* robot=scene.simulationRuntime().robot(scene.robotId());
        const Eigen::Vector3d base=robot->baseTransform.translation();
        for(int j=0;j<6;++j) {
            std::string link=robot->model.joints.at(robot->model.jointNameToIndex.at(scene.jointNames()[j])).child;
            for(std::size_t visit=0;link!=robot->model.root && visit<robot->model.links.size();++visit) {
                const auto joint=std::find_if(robot->model.joints.begin(),robot->model.joints.end(),[&](const auto& x){return x.child==link;});
                if(joint==robot->model.joints.end())throw std::runtime_error("Incomplete MoveL serial tree.");
                link=joint->parent;++m_depth[j];
            }
        }
        for(const auto& id:scene.collisionDetectorIds()) {
            const auto& detectors=scene.collisionRuntime().detectors();
            const auto d=std::find_if(detectors.begin(),detectors.end(),[&](const auto& x){return x.id==id;});
            if(d==detectors.end() || !d->valid || !d->enabled)throw std::runtime_error("Invalid conservative detector.");
            collision::CollisionMotionSnapshot snapshot;std::string error;
            if(scene.collisionRuntime().scene().motionSnapshot(d->options,snapshot,&error)!=collision::CollisionStatus::Success)
                throw std::runtime_error("Cannot freeze collision detector: "+error);
            // SelectedObjects detectors resolve selection sets in the project
            // query service. Freeze its effective pairs, not a broadphase list.
            const auto pairs=scene.collisionRuntime().pairDiagnostics(id);
            if(!pairs.empty()) {
                snapshot.pairs.clear();
                for(const auto& pair:pairs)if(pair.effective) {
                    const auto a=std::find_if(snapshot.objects.begin(),snapshot.objects.end(),[&](const auto& x){return x.key==pair.first.object;});
                    const auto b=std::find_if(snapshot.objects.begin(),snapshot.objects.end(),[&](const auto& x){return x.key==pair.second.object;});
                    if(a==snapshot.objects.end() || b==snapshot.objects.end())throw std::runtime_error("Effective collision pair missing from frozen geometry.");
                    snapshot.pairs.emplace_back(std::size_t(a-snapshot.objects.begin()),std::size_t(b-snapshot.objects.begin()));
                }
            }
            if(o.progress)o.progress("Frozen detector "+id+": "+std::to_string(snapshot.objects.size())+" objects, "+std::to_string(snapshot.pairs.size())+" pairs");
            if(snapshot.pairs.empty())continue;
            std::vector<std::array<double,6>> levers;std::vector<bool> rigid;
            std::vector<std::shared_ptr<const detail::MeshSeparation>> meshes;
            const int lastJoint=int(std::max_element(m_depth.begin(),m_depth.end())-m_depth.begin());
            const auto tcpParent=robot->model.joints.at(robot->model.jointNameToIndex.at(scene.jointNames()[lastJoint])).child;
            for(const auto& object:snapshot.objects) {
                const auto& box=object.geometry->localAabb();double radius=0;
                for(int k=0;k<8;++k) {
                    const Eigen::Vector3d p(k&1?box.max.x():box.min.x(),k&2?box.max.y():box.min.y(),k&4?box.max.z():box.min.z());
                    radius=std::max(radius,(object.localTransform*p-base).norm());
                }
                // A mounted object offset is bounded by its initial root
                // radius + one tree reach; another two reaches cover any
                // ancestor axis and downstream fixed translations.
                std::array<double,6> coefficients;coefficients.fill(radius+3*reach);
                const auto* environment=scene.collisionRuntime().scene().findEnvironmentInfo(object.key);
                const auto fixed=std::find_if(scene.simulationRuntime().objects().begin(),scene.simulationRuntime().objects().end(),[&](const auto& x){
                    return x.objectType=="workpiece" && (x.runtimeId==object.key ||
                        (environment && environment->info.elementName==x.documentId));
                });
                if(fixed!=scene.simulationRuntime().objects().end())coefficients.fill(0.);
                const auto* info=scene.collisionRuntime().scene().find(object.key);
                std::string ownerLink=info?info->linkName:std::string();
                if(environment)for(const auto& attachment:scene.simulationRuntime().mountedAttachments())
                    if(attachment.owner.valid && attachment.owner.robotId==scene.robotId() &&
                        (environment->info.elementName==attachment.documentId || environment->info.elementName=="attachment:"+attachment.documentId ||
                        environment->info.source=="attachment:"+attachment.documentId))ownerLink=attachment.owner.linkName;
                std::string rigidAncestor=ownerLink;
                for(std::size_t visit=0;rigidAncestor!=tcpParent && visit<robot->model.links.size();++visit) {
                    const auto joint=std::find_if(robot->model.joints.begin(),robot->model.joints.end(),[&](const auto& x){return x.child==rigidAncestor;});
                    if(joint==robot->model.joints.end() || joint->type!=robot::JointType::Fixed)break;
                    rigidAncestor=joint->parent;
                }
                rigid.push_back(rigidAncestor==tcpParent);
                if(o.progress)o.progress("Bound object "+std::to_string(levers.size())+": owner="+ownerLink+
                    ", TCP rigid="+std::to_string(rigid.back()));
                if(robot->model.links.count(ownerLink)) {
                    double localRadius=0;const auto linkInverse=scene.linkWorldTransform(ownerLink).inverse();
                    for(int c=0;c<8;++c) {
                        const Eigen::Vector3d p(c&1?box.max.x():box.min.x(),c&2?box.max.y():box.min.y(),c&4?box.max.z():box.min.z());
                        localRadius=std::max(localRadius,(linkInverse*object.localTransform*p).norm());
                    }
                    coefficients.fill(0.);std::string link=ownerLink;double length=localRadius;
                    std::size_t visited=0;
                    while(link!=robot->model.root && ++visited<=robot->model.links.size()) {
                        const auto joint=std::find_if(robot->model.joints.begin(),robot->model.joints.end(),[&](const auto& j){return j.child==link;});
                        if(joint==robot->model.joints.end()){coefficients.fill(radius+3*reach);break;}
                        length+=robot->model.links.at(link).T_local.translation().norm()+joint->T_joint_child.translation().norm();
                        const auto planned=std::find(scene.jointNames().begin(),scene.jointNames().end(),joint->name);
                        if(planned!=scene.jointNames().end())coefficients[planned-scene.jointNames().begin()]=length;
                        length+=joint->T_parent_joint.translation().norm();link=joint->parent;
                    }
                    if(visited>robot->model.links.size())throw std::runtime_error("Cycle in conservative robot lever tree.");
                }
                meshes.push_back(std::make_shared<detail::MeshSeparation>(*object.geometry));
                levers.push_back(coefficients);
            }
            m_snapshots.push_back(std::move(snapshot));m_levers.push_back(std::move(levers));m_tcpRigid.push_back(std::move(rigid));
            if(o.progress) {
                std::size_t supported=0;for(const auto& mesh:meshes)if(mesh->valid())++supported;
                o.progress("Mesh bounds available: "+std::to_string(supported)+" / "+std::to_string(meshes.size()));
            }
            m_meshes.push_back(std::move(meshes));
            m_distanceCache.emplace_back(m_snapshots.back().pairs.size());
        }
        if(m_snapshots.empty())throw std::runtime_error("No effective frozen collision pairs.");
    }
    StateValidationResult check(const std::vector<double>& a,const std::vector<double>& b) {
        const auto checkBegan=std::chrono::steady_clock::now();
        const auto poseA=m_sampler.forward(a),poseB=m_sampler.forward(b);
        const Eigen::AngleAxisd rotation(poseA.linear().transpose()*poseB.linear());
        for(std::size_t k=0;k<m_snapshots.size();++k) {
            const auto& snapshot=m_snapshots[k];
            const auto evaluate=[&](double u,std::vector<collision::Transform3>& transforms,std::string& error) {
                std::vector<double> q;
                if(!m_sampler.sample(a,b,u,q,error))return false;
                if(!m_scene.setState(q,&error))return false;transforms.resize(snapshot.objects.size());
                for(std::size_t i=0;i<transforms.size();++i)
                    if(!m_scene.collisionRuntime().scene().objectTransform(snapshot.objects[i].key,transforms[i])) {error="Frozen collision object disappeared.";return false;}
                return true;
            };
            // The installed general motion query proves separation with AABBs
            // only. Tight CAD clearances need exact geometry distance as the
            // interval lower bound: distance is Lipschitz with respect to the
            // sum of the two objects' maximum point displacements.
            using Transforms=std::vector<collision::Transform3>;
            struct Interval {double from,to;std::shared_ptr<Transforms> a,b;};
            auto firstTransforms=std::make_shared<Transforms>(),lastTransforms=std::make_shared<Transforms>();std::string endpointError;
            if(!evaluate(0,*firstTransforms,endpointError) || !evaluate(1,*lastTransforms,endpointError))return StateValidationResult::failure("motion_indeterminate",endpointError);
            std::vector<Interval> pending{{0,1,firstTransforms,lastTransforms}};
            std::size_t evaluations=0;std::size_t unbounded=0;
            const auto worldBox=[](const collision::CollisionGeometrySetEntry& object,const collision::Transform3& tf) {
                const auto& local=object.geometry->localAabb();collision::Aabb box;
                box.min=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());box.max=-box.min;
                for(int c=0;c<8;++c) {
                    const Eigen::Vector3d p=tf*Eigen::Vector3d(c&1?local.max.x():local.min.x(),c&2?local.max.y():local.min.y(),c&4?local.max.z():local.min.z());
                    box.min=box.min.cwiseMin(p);box.max=box.max.cwiseMax(p);
                }
                return box;
            };
            while(!pending.empty()) {
                if(m_request.cancelled && m_request.cancelled())return StateValidationResult::failure("cancelled","Cancelled conservative MoveL audit.");
                if(++evaluations>m_request.maxEvaluations)return StateValidationResult::failure("motion_indeterminate","Conservative MoveL interval budget exceeded; inverse subdivisions="+std::to_string(unbounded)+".");
                if(evaluations%16384==0 && m_progress)m_progress("MoveL audit subdivision "+std::to_string(evaluations)+", inverse-only "+std::to_string(unbounded));
                const auto interval=pending.back();pending.pop_back();const double middle=.5*(interval.from+interval.to),half=.5*(interval.to-interval.from);
                m_collisionParameter=middle;
                auto middleTransforms=std::make_shared<Transforms>();std::string error;
                if(!evaluate(middle,*middleTransforms,error))return StateValidationResult::failure("motion_indeterminate",error);
                const auto& transforms=*middleTransforms;
                // A validated inverse-function ball encloses the entire MoveL
                // IK lift. This is not an endpoint velocity estimate: the map
                // F=(p/reach, vee(skew(Rmid^T R))) has a Jacobian Lipschitz bound
                // 24 on a six-revolute serial chain in a Euclidean joint ball.
                // Each translational column changes by <=2*sum|dq| and each
                // angular chart column by <=3*sum|dq|, hence 6*sqrt(13)<24.
                std::vector<double> qmid,qa,qb;
                if(!m_sampler.sample(a,b,middle,qmid,error) ||
                    !m_sampler.sample(a,b,interval.from,qa,error) || !m_sampler.sample(a,b,interval.to,qb,error))
                    return StateValidationResult::failure("cartesian_ik_failed",error);
                const auto pose=m_sampler.forward(qmid);
                const Eigen::Matrix<double,6,6> worldJac=m_sampler.jacobian(qmid);
                Eigen::Matrix<double,6,6> jac=worldJac;
                jac.topRows<3>()/=m_reach;jac.bottomRows<3>()=pose.linear().transpose()*jac.bottomRows<3>();
                const Eigen::FullPivLU<Eigen::Matrix<double,6,6>> lu(jac);
                bool bounded=lu.isInvertible();double radius=0;
                if(bounded) {
                    const Eigen::Matrix<double,6,6> inverse=lu.inverse();
                    const Eigen::Vector3d dp=(poseB.translation()-poseA.translation())/m_reach;
                    const Eigen::Vector3d axis=pose.linear().transpose()*poseA.linear()*rotation.axis();
                    // A numerical residual reserve covers the sampler stopping
                    // tolerance; contraction encloses exact roots, not samples.
                    const double d=half*(inverse.leftCols<3>()*dp).norm()+
                        std::sin(std::min(1.5707963267948966,half*rotation.angle()))*(inverse.rightCols<3>()*axis).norm()+inverse.norm()*1e-8;
                    double endpoints=0;for(const auto* q:{&qa,&qb}) {
                        double norm=0;for(int j=0;j<6;++j)norm+=std::pow((*q)[j]-qmid[j],2);
                        endpoints=std::max(endpoints,std::sqrt(norm));
                    }
                    radius=std::max(2*d,endpoints*1.001)+1e-10;
                    // Analytic chart Hessian at the center tightens the same
                    // validated ball. A serial FK is a sum of fixed offsets
                    // times rotation products. Every third mixed partial has
                    // scaled task norm <=sqrt(2); the full tensor norm is
                    // <=sqrt(432)<64. This bounds the Hessian remainder.
                    double hessianSquared=0;
                    for(int j=0;j<6;++j)for(int k=0;k<6;++k) {
                        const int parent=m_depth[j]<=m_depth[k]?j:k,child=parent==j?k:j;
                        const auto hessian=detail::moveLChartHessian(worldJac,pose.linear(),m_reach,parent,child);
                        hessianSquared+=(inverse*hessian).squaredNorm();
                    }
                    const double hessian=std::sqrt(hessianSquared),third=64*inverse.norm();
                    const double contraction=hessian*radius+.5*third*radius*radius;
                    bounded=half*rotation.angle()<.5 && radius*std::sqrt(6.)<.5 && contraction<.5 &&
                        d+.5*hessian*radius*radius+third*radius*radius*radius/6<radius;
                    if(m_scene.enforcesJointPositionLimits())for(int j=0;j<6;++j) {
                        const auto& bound=m_scene.jointBounds()[j];
                        if(!bound.continuous && (qmid[j]-radius<bound.lower || qmid[j]+radius>bound.upper))bounded=false;
                    }
                }
                if(!bounded) {
                    ++unbounded;
                    if(2*half<=m_request.minParameterSpan)return StateValidationResult::failure("motion_indeterminate","MoveL inverse-function interval could not be enclosed.");
                    pending.push_back({middle,interval.to,middleTransforms,interval.b});pending.push_back({interval.from,middle,interval.a,middleTransforms});continue;
                }
                std::vector<double> displacementBounds(snapshot.objects.size(),0.);
                for(std::size_t i=0;i<displacementBounds.size();++i) {
                    double squared=0;for(double lever:m_levers[k][i])squared+=lever*lever;
                    displacementBounds[i]=std::sqrt(squared)*radius;
                    // Geometry rigidly mounted on the TCP parent follows the
                    // prescribed Cartesian path directly, independently of IK.
                    if(m_tcpRigid[k][i]) {
                        const auto& box=snapshot.objects[i].geometry->localAabb();double offset=0;
                        for(int c=0;c<8;++c) {
                            const Eigen::Vector3d p(c&1?box.max.x():box.min.x(),c&2?box.max.y():box.min.y(),c&4?box.max.z():box.min.z());
                            offset=std::max(offset,(transforms[i]*p-pose.translation()).norm());
                        }
                        displacementBounds[i]=half*(poseB.translation()-poseA.translation()).norm()+
                            2*std::sin(.5*half*rotation.angle())*offset+1e-8;
                    }
                }
                bool clear=true;
                if(m_hasBaffles) {
                    std::string reason;
                    if(!m_baffles.validMotionBall(m_scene.simulationRuntime(),m_scene.robotId(),radius,
                        m_request.clearance+m_request.numericalTolerance,reason)) {
                        if(!m_baffles.validState(m_scene.simulationRuntime(),m_request.clearance,reason))
                            return StateValidationResult::failure("virtual_baffle_collision",reason);
                        clear=false;
                    }
                }
                for(std::size_t pairIndex=0;pairIndex<snapshot.pairs.size();++pairIndex) {
                    const auto& pair=snapshot.pairs[pairIndex];
                    const auto i=pair.first,j=pair.second;const double displacement=displacementBounds[i]+displacementBounds[j];
                    const auto aa=worldBox(snapshot.objects[i],transforms[i]),bb=worldBox(snapshot.objects[j],transforms[j]);
                    const Eigen::Vector3d gap=(aa.min-bb.max).cwiseMax(bb.min-aa.max).cwiseMax(0.);
                    if(gap.norm()>m_request.clearance+displacement+m_request.numericalTolerance)continue;
                    if(m_meshes[k][i]->valid() && m_meshes[k][j]->valid()) {
                        if(m_meshes[k][i]->separated(*m_meshes[k][j],transforms[i].inverse()*transforms[j],
                            m_request.clearance+displacement+m_request.numericalTolerance))continue;
                        // Separation of all mesh surfaces over the displacement
                        // ball prevents any crossing from the independently
                        // verified collision-free start. No nearest distance is
                        // required. Inconclusive bounds are never accepted.
                        const auto state=m_scene.validateState(qmid);
                        if(!state.valid){m_collisionParameter=middle;return state;}
                        if(half<1e-6 && evaluations%1024==0 && m_progress)m_progress("Tight mesh pair "+std::to_string(i)+"/"+std::to_string(j)+
                            " u="+std::to_string(middle)+", motion bound="+std::to_string(displacement));
                        clear=false;break;
                    }
                    auto& cached=m_distanceCache[k][pairIndex];
                    if(cached.distance>=0) {
                        const auto moved=[&](std::size_t object,const collision::Transform3& old) {
                            const auto& box=snapshot.objects[object].geometry->localAabb();double maximum=0;
                            for(int c=0;c<8;++c){const Eigen::Vector3d p(c&1?box.max.x():box.min.x(),c&2?box.max.y():box.min.y(),c&4?box.max.z():box.min.z());maximum=std::max(maximum,(transforms[object]*p-old*p).norm());}
                            return maximum;
                        };
                        if(cached.distance-moved(i,cached.a)-moved(j,cached.b)>m_request.clearance+displacement+m_request.numericalTolerance)continue;
                    }
                    collision::CollisionGeometrySet first,second;first.entries={snapshot.objects[i]};second.entries={snapshot.objects[j]};
                    first.entries.front().localTransform=transforms[i];second.entries.front().localTransform=transforms[j];
                    collision::GeometrySetQueryRequest query;query.kind=collision::GeometrySetQueryKind::Distance;query.maxResults=1;
                    const auto pairBegan=std::chrono::steady_clock::now();
                    const auto distance=collision::CollisionGeometrySetQuery::query(first,second,query);
                    if(m_trace && evaluations==1 && m_progress)m_progress("Exact pair distance="+std::to_string(distance.minDistance)+
                        " m, query="+std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-pairBegan).count())+" s");
                    if(distance.status!=collision::CollisionStatus::Success || !distance.complete || !std::isfinite(distance.minDistance))
                        return StateValidationResult::failure("motion_indeterminate","Exact pair distance unavailable: "+distance.message);
                    if(distance.collision || distance.minDistance<=m_request.clearance+m_request.numericalTolerance) {
                        m_collisionParameter=middle;
                        return StateValidationResult::failure("motion_in_collision","Exact distance violates continuous clearance at parameter "+std::to_string(middle));
                    }
                    cached={distance.minDistance,transforms[i],transforms[j]};
                    if(distance.minDistance<=m_request.clearance+displacement+m_request.numericalTolerance){clear=false;break;}
                }
                if(!clear) {
                    if(2*half<=m_request.minParameterSpan)return StateValidationResult::failure("motion_indeterminate","Exact-distance interval below parameter tolerance.");
                    pending.push_back({middle,interval.to,middleTransforms,interval.b});pending.push_back({interval.from,middle,interval.a,middleTransforms});
                }
            }
            if(m_trace && m_progress)m_progress("MoveL audit intervals="+std::to_string(evaluations)+", inverse subdivisions="+std::to_string(unbounded));
        }
        if(m_trace && m_progress)m_progress("First continuous MoveL segment certified in "+std::to_string(std::chrono::duration<double>(std::chrono::steady_clock::now()-checkBegan).count())+" s");
        m_trace=false;
        return StateValidationResult::success();
    }
};
Eigen::Isometry3d interpolate(const Eigen::Isometry3d& a,const Eigen::Isometry3d& b,double u) {
    auto p=Eigen::Isometry3d::Identity();p.translation()=(1-u)*a.translation()+u*b.translation();
    p.linear()=Eigen::Quaterniond(a.linear()).slerp(u,Eigen::Quaterniond(b.linear())).toRotationMatrix();return p;
}
bool solve(const CartesianMotionSampler& sampler,const Eigen::Isometry3d& target,
    const std::vector<JointBound>& bounds,bool limits,std::vector<double>& q,const std::function<bool()>& stop) {
    if(!target.matrix().allFinite())return false;
    for(int iteration=0;iteration<100;++iteration) {
        if(stop && stop())return false;
        const auto pose=sampler.forward(q);Eigen::Matrix<double,6,1> e;
        e.head<3>()=target.translation()-pose.translation();Eigen::AngleAxisd aa(target.linear()*pose.linear().transpose());
        e.tail<3>()=aa.axis()*aa.angle();if(e.head<3>().norm()<1e-10 && e.tail<3>().norm()<1e-10)return true;
        const auto j=sampler.jacobian(q);const Eigen::Matrix<double,6,6> normal=j*j.transpose()+1e-10*Eigen::Matrix<double,6,6>::Identity();
        Eigen::Matrix<double,6,1> delta=j.transpose()*normal.ldlt().solve(e);
        if(!delta.allFinite())return false;const double maximum=delta.cwiseAbs().maxCoeff();if(maximum>.12)delta*=.12/maximum;
        bool improved=false;
        for(double alpha=1;alpha>=1./64;alpha*=.5) {
            auto trial=q;for(int k=0;k<6;++k) {
                trial[k]+=alpha*delta[k];
                if(limits && !bounds[k].continuous)trial[k]=std::clamp(trial[k],bounds[k].lower,bounds[k].upper);
            }
            const auto actual=sampler.forward(trial);const Eigen::AngleAxisd angle(target.linear()*actual.linear().transpose());
            if((target.translation()-actual.translation()).squaredNorm()+angle.angle()*angle.angle()<e.squaredNorm()) {
                q=std::move(trial);improved=true;break;
            }
        }
        if(!improved)return false;
    }
    return false;
}

}
SprayAxisContinuityResult ProjectSprayAxisContinuityService::repair(
    const simulation_project::ProjectDocument& document,const std::filesystem::path& base,
    const StoredMotionPlan& input,const SprayAxisContinuityOptions& o) {
    SprayAxisContinuityResult out;Json stats={{"success",false},{"algorithm","surface_preserving_sparse_MoveL"}};
    const auto began=std::chrono::steady_clock::now();
    const auto progress=[&](const std::string& text){if(o.progress)o.progress(text);};
    const auto stop=[&]{return o.cancelled && o.cancelled();};
    try {
        if(input.robotId.empty() || input.jointNames.size()!=6 || o.collisionDetectorIds.empty() ||
            !o.sprayAxis.allFinite() || std::abs(o.sprayAxis.norm()-1)>1e-10 ||
            !std::isfinite(o.maximumRollRadians) || o.maximumRollRadians<0 || o.maximumRollRadians>3.141592653589794 ||
            !std::isfinite(o.maximumTiltRadians) || o.maximumTiltRadians<0 || o.maximumTiltRadians>3.141592653589794 ||
            !std::isfinite(o.maximumTcpAdjustment) || o.maximumTcpAdjustment<0 || o.maximumTcpAdjustment>.5 ||
            !std::isfinite(o.validationJointStep) || o.validationJointStep<1e-5 || o.validationJointStep>.01 ||
            !std::isfinite(o.clearance) || o.clearance<0 || o.workers<1 || o.workers>16)
            throw std::runtime_error("Invalid spray-axis continuity options.");
        if(std::string(detail::sprayPoseConstraintName(o.poseConstraint))=="invalid")
            throw std::runtime_error("Invalid spray pose constraint.");
        const bool fixedTcp=o.poseConstraint==SprayPoseConstraint::FixedTcp;
        const bool flexibleDistance=o.poseConstraint==SprayPoseConstraint::SurfacePivotAdjustableDistance;
        const double positionBudget=fixedTcp?0.:o.maximumTcpAdjustment;
        stats["pose_constraint"]=detail::sprayPoseConstraintName(o.poseConstraint);
        stats["position_tolerance_m"]=1e-9;
        stats["spray_axis_rule"]=fixedTcp?"first hit and TCP fixed: axial roll only; transfer orientation may change":"surface pivot; TCP may move";
        ProjectPlanningRequest request;request.robotId=input.robotId;request.jointNames=input.jointNames;
        request.collisionDetectorIds=o.collisionDetectorIds;request.enforceJointPositionLimits=o.enforceJointPositionLimits;
        std::string error,tcp;Eigen::Isometry3d linkFromTcp;
        if(!resolvePlanningTcp(document,input.robotId,tcp,linkFromTcp,error))throw std::runtime_error(error);
        auto scene=ProjectPlanningSceneBuilder::build(document,base,request,&error);if(!scene)throw std::runtime_error(error);
        const auto sampler=CartesianMotionSampler::create(*scene,tcp,linkFromTcp,error);if(!sampler)throw std::runtime_error(error);
        StoredMotionPlan source=input;
        if(source.trajectory.empty()) {
            progress("Solving source Cartesian controls on actual robot");
            CartesianIkOptions ik;ik.robotId=input.robotId;ik.jointNames=input.jointNames;
            const auto* runtime=scene->simulationRuntime().robot(input.robotId);
            const auto initial=runtime->instance->getState().q;
            for(const auto& name:input.jointNames)ik.seedJoints.push_back(initial.at(runtime->model.joints.at(runtime->model.jointNameToIndex.at(name)).dofIndex));
            ik.worldForwardKinematics=[&](const auto& q){return sampler->forward(q);};
            ik.worldJacobian=[&](const auto& q){return sampler->jacobian(q);};
            ik.maxIterations=1000;ik.tolerance=1e-10;ik.stepSize=1;ik.damping=.001;
            if(o.enforceJointPositionLimits)for(const auto& b:scene->jointBounds()){ik.lower.push_back(b.lower);ik.upper.push_back(b.upper);}
            const auto result=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,source,ik);
            if(!result.success)throw std::runtime_error("Source inverse kinematics failed: "+(result.diagnostics.empty()?std::string():result.diagnostics.back().message));
            source=result.plan;
        }
        const std::size_t count=source.trajectory.points.size();if(count<2 || count>20000 || count>o.maximumOutputPoints)throw std::runtime_error("Expected 2..20000 source controls.");
        stats["input_points"]=count;stats["enforce_joint_limits"]=o.enforceJointPositionLimits;
        std::vector<Eigen::Isometry3d> poses(count);
        for(std::size_t i=0;i<count;++i) {
            const auto& p=source.trajectory.points[i];
            if(p.q.size()!=6 || !std::isfinite(p.time) || (i && p.time<=source.trajectory.points[i-1].time))throw std::runtime_error("Invalid source joints/timestamps.");
            for(double q:p.q)if(!std::isfinite(q))throw std::runtime_error("Nonfinite source joint.");
            poses[i]=sampler->forward(p.q);
            if(input.cartesianControlPoints.points.size()==count) {
                const auto& original=input.cartesianControlPoints.points[i].tcpPose;
                if(!original.matrix().allFinite() || (original.translation()-poses[i].translation()).norm()>1e-8 ||
                    Eigen::Quaterniond(original.linear()).angularDistance(Eigen::Quaterniond(poses[i].linear()))>1e-8)
                    throw std::runtime_error("Source Cartesian controls and solved joints disagree.");
                poses[i]=original;
            }
        }
        // A valid bound for the distance of any TCP from an ancestor joint:
        // triangle inequality over every fixed translation in the model tree.
        const auto& model=scene->simulationRuntime().robot(input.robotId)->model;
        double reach=linkFromTcp.translation().norm();
        for(const auto& j:model.joints) {
            if(j.type==robot::JointType::Prismatic)throw std::runtime_error("Prismatic joints require a separate chord bound.");
            reach+=j.T_parent_joint.translation().norm()+j.T_joint_child.translation().norm();
        }
        for(const auto& link:model.links)reach+=link.second.T_local.translation().norm();
        reach=std::max(1.,reach);stats["tcp_reach_bound_m"]=reach;
        MotionValidationOptions validation;validation.preserveJointTurns=true;validation.cancelled=o.cancelled;
        const auto reference=[&](double station) {
            const auto i=std::min(count-2,std::size_t(std::floor(std::max(0.,station))));
            return interpolate(poses[i],poses[i+1],station-i);
        };
        detail::SpraySurfaceReference surface(scene->simulationRuntime(),o.workpieceId);
        if(!surface.triangleCount())throw std::runtime_error("No triangulated spray target in the selected scene.");
        stats["virtual_baffles"]=Json::parse(scene->virtualBaffleStatus());
        stats["surface_triangles"]=surface.triangleCount();stats["collision_detector_ids"]=o.collisionDetectorIds;
        stats["continuous_collision_method"]="mesh_separating_bounds_and_validated_MoveL_inverse_function_balls_with_virtual_baffle_enclosures";
        stats["surface_sources"]=Json::array();
        for(const auto& object:scene->simulationRuntime().objects())
            if(o.workpieceId.empty()?object.objectType=="workpiece":object.documentId==o.workpieceId)stats["surface_sources"].push_back(object.sourcePath.u8string());
        std::map<double,double> originalDistances;
        const auto originalDistance=[&](double station) {
            const auto found=originalDistances.find(station);if(found!=originalDistances.end())return found->second;
            const auto r=reference(station);double d;
            if(!surface.hit(r.translation(),r.linear()*o.sprayAxis,d))d=std::numeric_limits<double>::quiet_NaN();
            originalDistances.emplace(station,d);return d;
        };
        const auto target=[&](double station,const Eigen::Vector3d& delta,double shift) {
            const auto original=reference(station);Eigen::Isometry3d pose=original;
            const double d=originalDistance(station);
            if(!detail::constrainedSprayPose(original,o.sprayAxis,d,delta,shift,o,pose)) {
                pose.translation().setConstant(std::numeric_limits<double>::quiet_NaN());return pose;
            }
            if(std::isfinite(d)) {
                double first;
                const double intended=fixedTcp?d:d+shift;
                if(!surface.hit(pose.translation(),pose.linear()*o.sprayAxis,first) || std::abs(first-intended)>1e-7)
                    pose.translation().setConstant(std::numeric_limits<double>::quiet_NaN());
            }
            return pose;
        };
        std::vector<Knot> controls(count);std::vector<Eigen::Vector3d> controlAngles(count,Eigen::Vector3d::Zero());
        std::vector<Eigen::Vector3d> controlOffsets(count,Eigen::Vector3d::Zero());
        for(std::size_t i=0;i<count;++i)controls[i]={double(i),source.trajectory.points[i].q};
        std::size_t sampledQueries=0,sampledStates=0;double witness=.5;std::string rejection;
        bool certifyCandidates=false;
        std::unique_ptr<MoveLCertificate> localCertificate;
        using EdgeKey=std::pair<std::vector<double>,std::vector<double>>;
        std::map<EdgeKey,bool> localCertificates;
        const auto connection=[&](const Knot& a,const Knot& b,double step) {
            // Search resolution is a filter, not a delivery certificate. The
            // independent replay below retains one-quarter joint step and the
            // complete conservative continuous audit, feeding failures back.
            ++sampledQueries;if(stop()){rejection="cancelled";return false;}
            // Reject the interior first, and retain its actual Cartesian station.
            for(double u:{.5,.25,.75,.125,.375,.625,.875}) {
                std::vector<double> q;std::string e;
                if(!sampler->sample(a.q,b.q,u,q,e)){witness=u;rejection="MoveL IK: "+e;return false;}
                ++sampledStates;const auto valid=scene->validateState(q);
                if(!valid.valid){witness=u;rejection=valid.diagnosticCode;return false;}
            }
            std::vector<std::vector<double>> samples;std::string e;
            if(!sampler->connection(a.q,b.q,step,100000,samples,e,o.cancelled)){rejection="MoveL IK: "+e;return false;}
            for(std::size_t k=0;k<samples.size();++k) {
                ++sampledStates;const auto valid=scene->validateState(samples[k]);
                if(!valid.valid){witness=double(k)/(samples.size()-1);rejection=valid.diagnosticCode;return false;}
            }
            if(certifyCandidates && step<=o.validationJointStep) {
                const EdgeKey key{a.q,b.q};const auto cached=localCertificates.find(key);
                if(cached!=localCertificates.end())return cached->second;
                const auto result=localCertificate->check(a.q,b.q);localCertificates[key]=result.valid;
                if(!result.valid){witness=localCertificate->collisionParameter();rejection=result.diagnosticCode;return false;}
            }
            return true;
        };
        const auto knotAt=[&](double station,const Eigen::Vector3d& angles,double shift,Knot& knot) {
            const auto i=std::min(count-2,std::size_t(std::floor(station)));const double f=station-i;
            knot={station,controls[i].q};
            for(int j=0;j<6;++j)knot.q[j]=(1-f)*controls[i].q[j]+f*controls[i+1].q[j];
            auto pose=target(station,angles,shift);
            // Transfer source repairs may use a full 3D translation. Carry its
            // local endpoint offsets into support searches so those repaired
            // endpoints can be connected, while spraying knots keep their hit.
            if(!fixedTcp && !std::isfinite(originalDistance(station))) {
                pose.translation()+=(1-f)*controlOffsets[i]+f*controlOffsets[i+1];
                if((pose.translation()-reference(station).translation()).norm()>o.maximumTcpAdjustment+1e-9)return false;
            }
            if(!solve(*sampler,pose,scene->jointBounds(),o.enforceJointPositionLimits,knot.q,o.cancelled))return false;
            return scene->validateState(knot.q).valid;
        };
        std::vector<Eigen::Vector3d> proposals{Eigen::Vector3d::Zero()};
        const double rad=std::acos(-1.)/180.;
        for(double angle:{.5,1.,2.,4.,7.,10.,15.,22.,30.,45.,60.,75.,90.,120.,150.,180.}) {
            if(angle*rad<=o.maximumRollRadians+1e-12)for(double sign:{1.,-1.})proposals.emplace_back(0,0,sign*angle*rad);
            if(angle*rad<=o.maximumTiltRadians+1e-12)for(int azimuth=0;azimuth<16;++azimuth) {
                const double phi=azimuth*std::acos(-1.)/8;proposals.emplace_back(angle*rad*std::cos(phi),angle*rad*std::sin(phi),0);
            }
        }
        // Combined tilt/roll and standoff add freedoms that the former single
        // bump templates never explored. Every candidate keeps the same hit.
        for(double angle:{2.,7.,15.,30.,45.,60.,75.,90.,120.,150.,180.})if(angle*rad<=o.maximumTiltRadians+1e-12)
            for(double roll:{15.,-15.,45.,-45.,90.,-90.})if(std::abs(roll)*rad<=o.maximumRollRadians+1e-12)
                for(int azimuth=0;azimuth<8;++azimuth) {
                    const double phi=azimuth*std::acos(-1.)/4;proposals.emplace_back(angle*rad*std::cos(phi),angle*rad*std::sin(phi),roll*rad);
                }
        Json sourceBadNodes=Json::array(),sourceRepairs=Json::array();
        std::vector<double> sourceShifts{0.};
        for(double distance:{.002,.005,.01,.02,.03,.05,.075,.1,.15,.2,.3,.5})if(!fixedTcp && distance<=o.maximumTcpAdjustment+1e-12) {
            sourceShifts.push_back(distance);sourceShifts.push_back(-distance);
        }
        for(std::size_t i=0;i<count;++i) {
            const auto originalState=scene->validateState(controls[i].q);if(originalState.valid)continue;
            sourceBadNodes.push_back(i);bool fixed=false;
            stats["source_colliding_nodes"]=sourceBadNodes;
            stats["source_node_rejection"]={{"index",i},{"code",originalState.diagnosticCode},{"message",originalState.message}};
            progress("Repairing colliding source control "+std::to_string(i+1)+": "+originalState.message);
            // First reuse the neighboring feasible posture and its small local
            // variations. This discourages independent opposite-side repairs.
            if(i && controlAngles[i-1].norm()>1e-10) {
                std::vector<Eigen::Vector3d> coherent{controlAngles[i-1]};
                for(double degrees:{.5,1.,2.,4.,7.})for(int axis=0;axis<3;++axis)for(double sign:{1.,-1.}) {
                    auto a=controlAngles[i-1];a[axis]+=sign*degrees*std::acos(-1.)/180.;coherent.push_back(a);
                }
                for(const auto& angles:coherent) {
                    Knot trial;if(!knotAt(double(i),angles,0.,trial))continue;
                    controls[i]=std::move(trial);controlAngles[i]=angles;fixed=true;
                    sourceRepairs.push_back({{"source_index",i},{"angles_rad",{angles.x(),angles.y(),angles.z()}},
                        {"spray_distance_shift_m",0.},{"neighbor_posture_reused",true}});break;
                }
            }
            for(double shift:sourceShifts) {
                if(!flexibleDistance && std::isfinite(originalDistance(double(i))) && shift!=0.)continue;
                if(fixed)break;
                for(const auto& angles:proposals) {
                    Knot trial;if(!knotAt(double(i),angles,shift,trial))continue;
                    controls[i]=std::move(trial);controlAngles[i]=angles;fixed=true;
                    sourceRepairs.push_back({{"source_index",i},{"angles_rad",{angles.x(),angles.y(),angles.z()}},{"spray_distance_shift_m",shift}});
                    break;
                }
                if(fixed)break;
            }
            if(!fixed) {
                // The regular angular grid can miss a thin feasible window near
                // the tool reach/baffle boundary. Deterministic local samples
                // retain the same surface anchor; transfers also permit a small
                // translation ball instead of only the original spray axis.
                std::mt19937 generator(1701u+unsigned(i));
                std::uniform_real_distribution<double> unit(0.,1.);
                const bool spraying=std::isfinite(originalDistance(double(i)));
                std::size_t poseRejected=0,ikRejected=0,collisionRejected=0;
                for(int attempt=0;attempt<(fixedTcp && spraying?0:16384) && !fixed;++attempt) {
                    if(stop())throw std::runtime_error("Cancelled.");
                    const double tilt=fixedTcp && spraying?0.:o.maximumTiltRadians*std::sqrt(unit(generator));
                    const double azimuth=2*std::acos(-1.)*unit(generator);
                    const Eigen::Vector3d angles(tilt*std::cos(azimuth),tilt*std::sin(azimuth),
                        (2*unit(generator)-1)*o.maximumRollRadians);
                    const double shift=flexibleDistance && spraying?(2*unit(generator)-1)*o.maximumTcpAdjustment:0.;
                    auto pose=target(double(i),angles,shift);
                    Eigen::Vector3d offset=Eigen::Vector3d::Zero();
                    if(!fixedTcp && !spraying && attempt>=8192) {
                        offset=Eigen::Vector3d(2*unit(generator)-1,2*unit(generator)-1,2*unit(generator)-1);
                        if(offset.norm()>1e-12)offset*=o.maximumTcpAdjustment*unit(generator)*unit(generator)/offset.norm();
                        pose.translation()+=offset;
                    }
                    if(!pose.matrix().allFinite()){++poseRejected;continue;}
                    Knot trial{double(i),controls[i].q};
                    if(!solve(*sampler,pose,scene->jointBounds(),o.enforceJointPositionLimits,trial.q,o.cancelled)){++ikRejected;continue;}
                    const auto validity=scene->validateState(trial.q);
                    if(!validity.valid){++collisionRejected;auto& rejected=stats["refined_rejections"][validity.message];
                        rejected=rejected.is_number_unsigned()?rejected.get<std::size_t>()+1:1;continue;}
                    controls[i]=std::move(trial);controlAngles[i]=angles;fixed=true;
                    sourceRepairs.push_back({{"source_index",i},{"angles_rad",{angles.x(),angles.y(),angles.z()}},
                        {"spray_distance_shift_m",shift},{"transfer_offset_m",{offset.x(),offset.y(),offset.z()}},{"refined_candidate",attempt}});
                }
                stats["source_refined_search"]={{"index",i},{"pose_rejected",poseRejected},{"ik_rejected",ikRejected},{"collision_rejected",collisionRejected},{"repaired",fixed}};
            }
            if(!fixed && fixedTcp && std::isfinite(originalDistance(double(i)))) {
                std::size_t tested=0,ikSolutions=0;
                const double halfDegree=std::acos(-1.)/360.;
                std::vector<Eigen::Vector3d> rolls;
                for(int step=1;step<=360;++step)for(double sign:{1.,-1.}) {
                    const double roll=sign*step*halfDegree;
                    if(std::abs(roll)<=o.maximumRollRadians+1e-12)rolls.emplace_back(0,0,roll);
                }
                const auto accept=[&](const Eigen::Vector3d& angles,const std::vector<double>& seed) {
                    ++tested;const auto pose=target(double(i),angles,0.);auto q=seed;
                    if(!solve(*sampler,pose,scene->jointBounds(),o.enforceJointPositionLimits,q,o.cancelled))return false;
                    ++ikSolutions;if(!scene->validateState(q).valid)return false;
                    controls[i]={double(i),std::move(q)};controlAngles[i]=angles;
                    sourceRepairs.push_back({{"source_index",i},{"angles_rad",{0,0,angles.z()}},{"spray_distance_shift_m",0.},
                        {"deterministic_axial_search",true}});return true;
                };
                progress("Fixed-TCP axial refinement at source control "+std::to_string(i+1));
                for(const auto& angles:rolls) {
                    if(stop())throw std::runtime_error("Cancelled.");
                    if(accept(angles,controls[i].q)){fixed=true;break;}
                }
                const auto radical=[](unsigned n,unsigned base) {
                    double value=0,factor=1.;for(;n;n/=base){factor/=base;value+=factor*(n%base);}return value;
                };
                for(unsigned seedIndex=1;seedIndex<=24 && !fixed;++seedIndex) {
                    auto seed=controls[i].q;const unsigned primes[6]={2,3,5,7,11,13};
                    for(int j=0;j<6;++j) {
                        seed[j]+=(2*radical(seedIndex,primes[j])-1)*std::acos(-1.);
                        if(o.enforceJointPositionLimits && !scene->jointBounds()[j].continuous)
                            seed[j]=std::clamp(seed[j],scene->jointBounds()[j].lower,scene->jointBounds()[j].upper);
                    }
                    for(int step=0;step<=72 && !fixed;++step) {
                        if(stop())throw std::runtime_error("Cancelled.");
                        const double roll=(int(step)-36)*std::acos(-1.)/36.;
                        if(std::abs(roll)>o.maximumRollRadians+1e-12)continue;
                        if(accept(Eigen::Vector3d(0,0,roll),seed))fixed=true;
                    }
                }
                stats["fixed_tcp_refinement"]={{"source_index",i},{"tested_candidates",tested},{"converged_ik",ikSolutions},
                    {"roll_step_degrees",.5},{"alternate_seed_count",24},{"repaired",fixed}};
            }
            if(fixed)controlOffsets[i]=sampler->forward(controls[i].q).translation()-poses[i].translation();
            stats["source_control_repairs"]=sourceRepairs;
            if(fixed && std::none_of(proposals.begin(),proposals.end(),[&](const auto& a){return (a-controlAngles[i]).norm()<1e-8;}))
                proposals.insert(proposals.begin(),controlAngles[i]);
            if(!fixed) {
                stats["failed_source_index"]=i;
                stats["failure_kind"]="search_budget_exhausted";
                stats["failed_tcp_m"]={poses[i].translation().x(),poses[i].translation().y(),poses[i].translation().z()};
                stats["failed_surface_distance_m"]=std::isfinite(originalDistance(double(i)))?Json(originalDistance(double(i))):Json(nullptr);
                throw std::runtime_error("Source node "+std::to_string(i+1)+" collision repair exhausted under "+
                    detail::sprayPoseConstraintName(o.poseConstraint)+" constraints. No compliant trajectory was produced; this finite search is not an infeasibility proof.");
            }
        }
        stats["source_colliding_nodes"]=sourceBadNodes;
        // Smooth the deviation field relative to the original path, rather
        // than flattening the path itself. All trials keep their own original
        // first-hit anchor/range and must pass actual IK and scene collisions.
        std::size_t smoothed=0;
        const auto deviationCost=[&](std::size_t node) {
            double cost=.05*(controlAngles[node].squaredNorm()+100*controlOffsets[node].squaredNorm());
            const auto begin=node>1?node-1:1,end=std::min(count-2,node+1);
            for(auto k=begin;k<=end;++k) {
                const Eigen::Vector3d angle=controlAngles[k-1]-2*controlAngles[k]+controlAngles[k+1];
                const Eigen::Vector3d shift=controlOffsets[k-1]-2*controlOffsets[k]+controlOffsets[k+1];
                cost+=angle.squaredNorm()+100*shift.squaredNorm();
            }
            return cost;
        };
        for(int sweep=0;sweep<4 && count>2;++sweep)for(std::size_t k=1;k+1<count;++k) {
            if(stop())throw std::runtime_error("Cancelled.");
            const auto node=sweep%2?count-1-k:k;
            const Eigen::Vector3d previousAngle=controlAngles[node],previousOffset=controlOffsets[node];
            const Eigen::Vector3d desiredAngle=.5*(controlAngles[node-1]+controlAngles[node+1]);
            const Eigen::Vector3d desiredOffset=.5*(controlOffsets[node-1]+controlOffsets[node+1]);
            if((desiredAngle-previousAngle).norm()<1e-7 && (desiredOffset-previousOffset).norm()<1e-8)continue;
            const double oldCost=deviationCost(node);
            for(double alpha:{1.,.5,.25,.125}) {
                const Eigen::Vector3d angles=(1-alpha)*previousAngle+alpha*desiredAngle;
                const bool transfer=!std::isfinite(originalDistance(double(node)));
                controlOffsets[node]=!fixedTcp && transfer?(1-alpha)*previousOffset+alpha*desiredOffset:previousOffset;
                Knot candidate;
                if(!knotAt(double(node),angles,0.,candidate)){controlOffsets[node]=previousOffset;continue;}
                controlAngles[node]=angles;
                controlOffsets[node]=sampler->forward(candidate.q).translation()-poses[node].translation();
                if(deviationCost(node)<oldCost-1e-10) {controls[node]=std::move(candidate);++smoothed;break;}
                controlAngles[node]=previousAngle;controlOffsets[node]=previousOffset;
            }
        }
        stats["coherent_control_updates"]=smoothed;
        progress("Coherent deviation smoothing accepted "+std::to_string(smoothed)+" collision-free control updates");
        progress("Checking source MoveL connections; validation samples are not exported");
        std::vector<std::vector<double>> sourceJoints;for(const auto& control:controls)sourceJoints.push_back(control.q);
        std::vector<StateValidationResult> located;
        {
            auto sampledScene=ProjectPlanningSceneBuilder::build(document,base,request,&error);if(!sampledScene)throw std::runtime_error(error);
            detail::CdfQueryBatch queries(*sampledScene,document,base,request,o.workers,true,o.cancelled);queries.setCartesianMotion(sampler);
            auto sampleOptions=validation;sampleOptions.maxJointStep=o.validationJointStep;sampleOptions.maxSamples=100000;
            located=queries.motions(sourceJoints,sampleOptions,false,[&](auto done,auto total){progress("MoveL connections "+std::to_string(done)+" / "+std::to_string(total));});
        }
        std::vector<std::vector<Knot>> edges(count-1);std::vector<std::size_t> failed;
        for(std::size_t i=0;i+1<count;++i){edges[i]={controls[i],controls[i+1]};if(!located[i].valid)failed.push_back(i);}
        stats["source_failed_edges"]=failed;progress("Source rejected MoveL intervals: "+std::to_string(failed.size()));
        Json repairs=Json::array();
        const auto pathValid=[&](const std::vector<Knot>& path,double step) {
            for(std::size_t i=1;i<path.size();++i)if(!connection(path[i-1],path[i],step))return false;
            return true;
        };
        struct SupportHint {Eigen::Vector3d angles;double shift;};
        std::vector<SupportHint> supportHints;
        const auto rememberSupport=[&](const Eigen::Vector3d& angles,double shift) {
            const Eigen::Vector3d saved=angles;
            supportHints.erase(std::remove_if(supportHints.begin(),supportHints.end(),[&](const auto& h){
                return (h.angles-saved).norm()<1e-8 && std::abs(h.shift-shift)<1e-10;
            }),supportHints.end());
            supportHints.push_back({saved,shift});if(supportHints.size()>24)supportHints.erase(supportHints.begin());
        };
        // Only local support controls are exported. A validation grid is never
        // part of the robot program. Try one support before a small beam graph.
        const auto repairEdge=[&](std::size_t edge) {
            const Knot first=controls[edge],last=controls[edge+1];
            if(!certifyCandidates && connection(first,last,o.validationJointStep)){edges[edge]={first,last};return true;}
            const double initialWitness=witness;
            std::vector<double> locations;
            for(double f:{std::clamp(initialWitness,.05,.95),.5,.25,.75,.125,.875,.0625,.9375})
                if(std::none_of(locations.begin(),locations.end(),[&](double old){return std::abs(old-f)<1e-10;}))locations.push_back(f);
            std::vector<SupportHint> hints(supportHints.rbegin(),supportHints.rend());
            hints.push_back({.5*(controlAngles[edge]+controlAngles[edge+1]),0.});
            hints.push_back({controlAngles[edge],0.});hints.push_back({controlAngles[edge+1],0.});
            for(double f:locations)for(const auto& hint:hints) {
                if(stop())return false;
                Knot middle;if(!knotAt(edge+f,hint.angles,hint.shift,middle))continue;
                if(!connection(first,middle,.01) || !connection(middle,last,.01))continue;
                const std::vector<Knot> trial{first,middle,last};if(!pathValid(trial,o.validationJointStep))continue;
                edges[edge]=trial;rememberSupport(hint.angles,hint.shift);
                repairs.push_back({{"source_edge",edge},{"supports",1},{"station",middle.station},
                    {"angles_rad",{hint.angles.x(),hint.angles.y(),hint.angles.z()}},{"spray_distance_shift_m",hint.shift},{"hint_reused",true}});
                progress("Repaired MoveL interval "+std::to_string(edge)+" with one reused support posture");return true;
            }
            for(double f:locations) {
                if(stop())return false;
                std::size_t safeCandidates=0,leftCandidates=0;
                std::vector<double> shifts{0.};
                for(double shift:{.002,.005,.01,.02,.03,.05,.075,.1,.15,.2,.3,.5})if(!fixedTcp && shift<=o.maximumTcpAdjustment+1e-12){shifts.push_back(shift);shifts.push_back(-shift);}
                for(double shift:shifts)for(const auto& angles:proposals) {
                    Knot middle;if(!knotAt(edge+f,angles,shift,middle))continue;
                    ++safeCandidates;if(!connection(first,middle,.01))continue;
                    ++leftCandidates;if(!connection(middle,last,.01))continue;
                    const std::vector<Knot> trial{first,middle,last};if(!pathValid(trial,o.validationJointStep))continue;
                    edges[edge]=trial;rememberSupport(angles,shift);repairs.push_back({{"source_edge",edge},{"supports",1},{"station",middle.station},
                        {"angles_rad",{angles.x(),angles.y(),angles.z()}},{"spray_distance_shift_m",shift}});
                    progress("Repaired MoveL interval "+std::to_string(edge)+" with one support");return true;
                }
                progress("Interval "+std::to_string(edge)+" support fraction "+std::to_string(f)+": safe postures="+
                    std::to_string(safeCandidates)+", reachable from start="+std::to_string(leftCandidates));
            }
            // First try adjusting a source control: this can keep the original
            // point count, but both adjacent MoveL connections must be valid.
            for(std::size_t node:{edge,edge+1})for(const auto& angles:proposals) {
                Knot trial;if(!knotAt(double(node),angles,0,trial))continue;
                const auto begin=node?node-1:node,end=std::min(count-2,node);
                bool valid=true;
                for(auto e=begin;e<=end && valid;++e) {
                    const auto a=e==node?trial:controls[e],b=e+1==node?trial:controls[e+1];
                    valid=connection(a,b,.01);
                }
                if(!valid)continue;
                for(auto e=begin;e<=end && valid;++e) {
                    const auto a=e==node?trial:controls[e],b=e+1==node?trial:controls[e+1];
                    valid=connection(a,b,o.validationJointStep);
                }
                if(!valid)continue;controls[node]=trial;controlAngles[node]=angles;
                controlOffsets[node]=sampler->forward(trial.q).translation()-poses[node].translation();
                for(auto e=begin;e<=end;++e)edges[e]={controls[e],controls[e+1]};
                repairs.push_back({{"source_edge",edge},{"modified_control",node},{"supports",0}});
                progress("Repaired MoveL interval "+std::to_string(edge)+" by source posture adjustment");return true;
            }
            struct Beam {std::vector<Knot> path;double cost=0;};
            std::vector<double> shifts{0.};
            for(double shift:{.002,.005,.01,.02,.03,.05,.075,.1,.15,.2,.3,.5})if(!fixedTcp && shift<=o.maximumTcpAdjustment+1e-12) {
                shifts.push_back(shift);shifts.push_back(-shift);
            }
            for(int layers:{3,7,15}) {
                std::vector<Beam> beam{{{first},0.}};
                for(int layer=1;layer<=layers && !beam.empty();++layer) {
                    const double f=double(layer)/(layers+1);std::vector<Beam> nextBeam;
                    struct Candidate {Knot knot;double cost;};std::vector<Candidate> candidates;
                    for(double shift:shifts)for(const auto& angles:proposals) {
                        if(stop())return false;Knot candidate;if(!knotAt(edge+f,angles,shift,candidate))continue;
                        const double cost=angles.head<2>().squaredNorm()+.1*angles.z()*angles.z()+400*shift*shift;
                        candidates.push_back({std::move(candidate),cost});
                    }
                    std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.cost<b.cost;});
                    // Keep distinct encoder configurations rather than the first
                    // few almost identical small rotations. Position shifts are
                    // equally available at every layer, including transfer moves.
                    for(const auto& candidate:candidates) {
                        const auto& knot=candidate.knot;bool duplicate=false;
                        for(const auto& other:nextBeam) {
                            double delta=0;for(int j=0;j<6;++j)delta+=std::pow(other.path.back().q[j]-knot.q[j],2);
                            if(delta<.0025){duplicate=true;break;}
                        }
                        if(duplicate)continue;
                        for(const auto& parent:beam) {
                            if(!connection(parent.path.back(),knot,.02))continue;
                            auto trial=parent.path;trial.push_back(knot);
                            if(connection(knot,last,.02)) {
                                trial.push_back(last);if(pathValid(trial,o.validationJointStep)) {
                                    for(std::size_t i=1;i+1<trial.size();) {
                                        if(connection(trial[i-1],trial[i+1],o.validationJointStep))trial.erase(trial.begin()+i);else ++i;
                                    }
                                    edges[edge]=trial;repairs.push_back({{"source_edge",edge},{"supports",trial.size()-2}});
                                    progress("Repaired MoveL interval "+std::to_string(edge)+" with "+std::to_string(trial.size()-2)+" supports");return true;
                                }
                            }
                            nextBeam.push_back({std::move(trial),parent.cost+candidate.cost});break;
                        }
                        if(nextBeam.size()>=32)break;
                    }
                    progress("Interval "+std::to_string(edge)+" graph "+std::to_string(layers)+" layer "+std::to_string(layer)+
                        ": safe="+std::to_string(candidates.size())+", reachable="+std::to_string(nextBeam.size()));
                    std::sort(nextBeam.begin(),nextBeam.end(),[](const auto& a,const auto& b){return a.cost<b.cost;});
                    beam=std::move(nextBeam);
                }
            }
            return false;
        };
        for(auto edge:failed) {
            if(stop())throw std::runtime_error("Cancelled.");
            if(!repairEdge(edge))throw std::runtime_error("MoveL posture search exhausted at source interval "+std::to_string(edge)+
                " (points "+std::to_string(edge+1)+" -> "+std::to_string(edge+2)+
                "); TCP/tilt/roll bounds may exclude a feasible path. Last check: "+rejection);
        }
        stats["repairs"]=repairs;
        std::vector<Knot> path;
        const auto collect=[&] {
            path.clear();path.push_back(edges.front().front());
            for(const auto& edge:edges) {
                if(path.back().q!=edge.front().q)throw std::runtime_error("Discontinuous MoveL control junction.");
                for(std::size_t k=1;k<edge.size();++k)path.push_back(edge[k]);
            }
            if(path.size()>o.maximumOutputPoints)throw std::runtime_error("Sparse MoveL control budget exceeded.");
        };
        std::set<EdgeKey> independentSamples,continuousCertificates;
        Json feedback=Json::array();
        for(;;) {
            collect();progress("Independent MoveL replay of "+std::to_string(path.size())+" output controls");
            std::size_t failedIndex=path.size();double failedParameter=.5;std::string finalError;
            {
                auto independent=ProjectPlanningSceneBuilder::build(document,base,request,&error);if(!independent)throw std::runtime_error(error);
                detail::CdfQueryBatch queries(*independent,document,base,request,o.workers,false,o.cancelled);queries.setCartesianMotion(sampler);
                std::vector<EdgeKey> remaining;std::vector<std::size_t> indices;
                for(std::size_t i=1;i<path.size();++i) {
                    EdgeKey key{path[i-1].q,path[i].q};
                    if(!independentSamples.count(key)){remaining.push_back(std::move(key));indices.push_back(i-1);}
                }
                auto precise=validation;precise.maxJointStep=o.validationJointStep*.25;precise.maxSamples=100000;
                progress("Independent MoveL segments remaining: "+std::to_string(remaining.size()));
                for(std::size_t begin=0;begin<remaining.size();begin+=128) {
                    const auto end=std::min(remaining.size(),begin+128);
                    const std::vector<EdgeKey> batch(remaining.begin()+begin,remaining.begin()+end);
                    const auto checked=queries.independentMotions(batch,precise,false);
                    for(std::size_t j=0;j<checked.size();++j) {
                        const auto i=begin+j;
                        if(checked[j].valid)independentSamples.insert(remaining[i]);
                        else if(failedIndex==path.size()){failedIndex=indices[i];finalError="Independent MoveL replay: "+checked[j].message;}
                    }
                    progress("Independent fine MoveL segments checked "+std::to_string(end)+" / "+std::to_string(remaining.size()));
                }
            }
            if(failedIndex==path.size()) {
                progress("Conservative continuous MoveL interval audit (internal subdivisions only)");
                std::atomic<std::size_t> next{0},done{0};std::atomic_bool auditFailed{false};std::mutex auditMutex;
                const auto audit=[&] {
                    try {
                        std::string e;auto independent=ProjectPlanningSceneBuilder::build(document,base,request,&e);if(!independent)throw std::runtime_error(e);
                        MoveLCertificate certificate(*independent,*sampler,reach,o);
                        for(;;) {
                            const auto i=next.fetch_add(1);if(i+1>=path.size() || auditFailed || stop())break;
                            const EdgeKey key{path[i].q,path[i+1].q};bool cached=false;
                            {std::lock_guard<std::mutex> lock(auditMutex);cached=continuousCertificates.count(key)!=0;}
                            if(!cached) {
                                const auto valid=certificate.check(key.first,key.second);
                                if(!valid.valid) {
                                    std::lock_guard<std::mutex> lock(auditMutex);
                                    if(!auditFailed){failedIndex=i;failedParameter=certificate.collisionParameter();finalError=valid.diagnosticCode+": "+valid.message;}
                                    auditFailed=true;break;
                                }
                                std::lock_guard<std::mutex> lock(auditMutex);continuousCertificates.insert(key);
                            }
                            const auto completed=++done;if(completed%100==0){std::lock_guard<std::mutex> lock(auditMutex);progress("Continuously certified MoveL segments "+std::to_string(completed)+" / "+std::to_string(path.size()-1));}
                        }
                    }catch(const std::exception& e){std::lock_guard<std::mutex> lock(auditMutex);auditFailed=true;finalError=e.what();}
                };
                std::vector<std::thread> workers;for(int i=1;i<o.workers;++i)workers.emplace_back(audit);audit();for(auto& worker:workers)worker.join();
                if(stop())throw std::runtime_error("Cancelled.");
                if(!auditFailed && done==path.size()-1)break;
                if(failedIndex==path.size())throw std::runtime_error(finalError.empty()?"Incomplete MoveL continuous audit.":finalError);
            }
            if(stop())throw std::runtime_error("Cancelled.");
            const auto edge=std::min(count-2,std::size_t(std::floor(path[failedIndex].station+1e-10)));
            feedback.push_back({{"source_edge",edge},{"output_segment",failedIndex},{"parameter",failedParameter},{"reason",finalError}});
            stats["audit_feedback"]=feedback;
            if(feedback.size()>16)throw std::runtime_error("Continuous MoveL repair feedback budget exceeded: "+finalError);
            progress("Continuous audit feeds source interval "+std::to_string(edge)+" back to posture search: "+finalError);
            if(!localCertificate){auto localOptions=o;localOptions.progress={};localCertificate=std::make_unique<MoveLCertificate>(*scene,*sampler,reach,localOptions);}
            certifyCandidates=true;const bool repaired=repairEdge(edge);certifyCandidates=false;
            if(!repaired)throw std::runtime_error("Continuous MoveL feedback could not repair source interval "+std::to_string(edge)+": "+rejection);
            stats["repairs"]=repairs;
        }
        stats["repairs"]=repairs;
        stats["sampled_moveL_queries"]=sampledQueries;stats["search_collision_samples"]=sampledStates;
        stats["independent_joint_step_rad"]=o.validationJointStep*.25;
        const std::size_t certifiedChordCount=path.size()-1;const std::size_t chordQueries=sampledQueries;
        out.plan=source;out.plan.id=input.id+"_spray_axis_continuous";out.plan.name="Surface-preserving sparse MoveL";
        out.plan.trajectory.interpolation=robottrajectory::TrajectoryInterpolation::Linear;
        out.plan.trajectory.points.clear();out.plan.cartesianControlPoints.points.clear();
        double maxPosition=0,maxAxis=0,maxHitError=0,maxOrientation=0,maxDistanceError=0;std::size_t surfaceKnots=0,sprayingKnots=0;
        Json sprayEnabled=Json::array(),stations=Json::array();
        Json process=Json::parse(source.processMetadata,nullptr,false);
        const bool hasProcess=process.is_object() && process.value("schema",std::string())=="smrobot.planning-process.v1" &&
            process.contains("points") && process["points"].is_array() && process["points"].size()==count;
        if(!source.processMetadata.empty() && !hasProcess)throw std::runtime_error("Unsupported source process metadata; refusing to lose process semantics.");
        Json processPoints=Json::array();
        for(const auto& knot:path) {
            const auto i=std::min(count-2,std::size_t(std::floor(knot.station)));const double f=knot.station-i;
            auto p=source.trajectory.points[i];p.time=(1-f)*source.trajectory.points[i].time+f*source.trajectory.points[i+1].time;p.q=knot.q;
            p.qd.clear();p.qdd.clear();
            if(o.enforceJointPositionLimits)for(int j=0;j<6;++j) {
                const auto& bound=scene->jointBounds()[j];
                if(!bound.continuous && (p.q[j]<bound.lower || p.q[j]>bound.upper))throw std::runtime_error("Output violates joint position limits.");
            }
            out.plan.trajectory.points.push_back(p);
            robottrajectory::TimedCartesianPoint tcpPoint;tcpPoint.time=p.time;tcpPoint.tcpPose=sampler->forward(p.q);
            out.plan.cartesianControlPoints.points.push_back(tcpPoint);
            const auto r=reference(knot.station);
            maxOrientation=std::max(maxOrientation,Eigen::Quaterniond(tcpPoint.tcpPose.linear()).angularDistance(Eigen::Quaterniond(r.linear())));
            maxPosition=std::max(maxPosition,(tcpPoint.tcpPose.translation()-r.translation()).norm());
            maxAxis=std::max(maxAxis,((tcpPoint.tcpPose.linear()-r.linear())*o.sprayAxis).norm());
            const double distance=originalDistance(knot.station);double outputDistance=distance;
            if(!std::isfinite(distance) && !o.allowNonSprayingTransitions)throw std::runtime_error("Source ray misses the spray surface.");
            const auto rowIndex=f>=1-1e-12?i+1:i;
            const bool spraying=std::isfinite(distance) && (!hasProcess || process["points"][rowIndex].value("spray",true));
            sprayEnabled.push_back(spraying);stations.push_back(knot.station);if(spraying)++sprayingKnots;
            if(std::isfinite(distance)) {
                double hitDistance;if(!surface.hit(tcpPoint.tcpPose.translation(),tcpPoint.tcpPose.linear()*o.sprayAxis,hitDistance))throw std::runtime_error("Output spray ray lost its original surface hit.");
                outputDistance=hitDistance;maxDistanceError=std::max(maxDistanceError,std::abs(hitDistance-distance));
                maxHitError=std::max(maxHitError,(tcpPoint.tcpPose.translation()+hitDistance*tcpPoint.tcpPose.linear()*o.sprayAxis-
                    r.translation()-distance*r.linear()*o.sprayAxis).norm());++surfaceKnots;
            }
            if(hasProcess) {
                Json row=process["points"][rowIndex];row["time"]=p.time;row["joints"]=p.q;row["spray"]=spraying;
                Json matrix=Json::array();for(int a=0;a<4;++a)for(int b=0;b<4;++b)matrix.push_back(tcpPoint.tcpPose.matrix()(a,b));row["pose"]=matrix;
                if(std::isfinite(outputDistance))row["distance"]=outputDistance;else row.erase("distance");
                processPoints.push_back(row);
                if(!std::isfinite(distance))processPoints.back()["spray"]=false;
            }
        }
        stats["output_points"]=path.size();stats["certified_moveL_segments"]=certifiedChordCount;stats["search_chord_queries"]=chordQueries;
        stats["maximum_knot_position_error_m"]=maxPosition;stats["maximum_knot_spray_axis_error"]=maxAxis;
        stats["continuous_position_error_bound_m"]=maxPosition;
        stats["validation_joint_step_rad"]=o.validationJointStep;stats["added_controls"]=path.size()-count;
        stats["clearance_m"]=o.clearance;stats["surface_knots"]=surfaceKnots;stats["maximum_surface_hit_error_m"]=maxHitError;
        stats["maximum_knot_orientation_change_rad"]=maxOrientation;
        stats["maximum_spray_distance_error_m"]=maxDistanceError;
        if(!flexibleDistance && maxDistanceError>1e-8)throw std::runtime_error("Original spray distance invariant failed.");
        stats["surface_constraint_scope"]="first surface hit at every output knot; interpolated hit is not certified identical";
        stats["execution_motion"]="cartesian_linear";
        stats["non_spraying_knots"]=path.size()-sprayingKnots;
        stats["roll_radians"]=Json::array();stats["tilt_radians"]=Json::array();
        for(const auto& delta:controlAngles){stats["roll_radians"].push_back(delta.z());stats["tilt_radians"].push_back(delta.head<2>().norm());}
        if(maxHitError>1e-6 || maxPosition>positionBudget+1e-9)throw std::runtime_error("Spray ray invariant failed.");
        if(hasProcess){process["points"]=std::move(processPoints);out.plan.processMetadata=process.dump();}
        auto metadata=Json::parse(source.optimizationMetadata,nullptr,false);if(!metadata.is_object())metadata=Json::object();
        metadata.erase("ik_solver");metadata.erase("points");
        metadata["motion"]="cartesian_linear";metadata["joint_convention"]="runtime";
        metadata["spray_enabled"]=std::move(sprayEnabled);metadata["source_stations"]=std::move(stations);
        metadata["spray_axis"]={o.sprayAxis.x(),o.sprayAxis.y(),o.sprayAxis.z()};
        stats["success"]=true;
        stats["seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
        metadata["spray_axis_continuity"]=stats;
        out.plan.optimizationMetadata=metadata.dump();out.success=true;out.message="Sparse MoveL trajectory passed independent replay and conservative continuous collision audit; surface hits preserved at output controls.";
    } catch(const std::exception& e) {out.plan={};out.message=e.what();stats["error"]=out.message;}
    stats["seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();out.statistics=stats.dump(2);return out;
}
}
