#include <ProjectMotionPlanning/VirtualBaffleEnvironment.h>
#include "GpuBaffleSearch.h"
#include "BaffleRimIndex.h"
#include <AssetCore/AssetManager.h>
#include <SimulationProject/AssetResolver.h>
#include <filesystem>
#include <stdexcept>
#include <mutex>
#include <map>
#include <sstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <functional>
namespace motion_planning {
namespace {
using Triangle=ModelTrajPlan::VirtualBaffle::Triangle;
using V3=Eigen::Vector3d;
std::vector<Triangle> mesh(const std::string& path,const V3& scale){
    auto model=assetcore::AssetManager::instance().loadModel(path,1.f);
    if(!model)throw std::runtime_error("baffle geometry load failed: "+path);
    std::vector<Triangle> out;
    for(const auto& part:model->subMeshes()){
        const auto& g=part.geometry;
        const auto vertex=[&](size_t i)->V3{const auto& v=g.positions.at(i);return V3(v.x(),v.y(),v.z()).cwiseProduct(scale);};
        size_t n=g.indices.empty()?g.positions.size():g.indices.size();
        if(n%3)throw std::runtime_error("baffle geometry is not triangulated: "+path);
        for(size_t i=0;i<n;i+=3)out.push_back({vertex(g.indices.empty()?i:g.indices[i]),vertex(g.indices.empty()?i+1:g.indices[i+1]),vertex(g.indices.empty()?i+2:g.indices[i+2])});
    }
    if(out.empty())throw std::runtime_error("empty baffle collision geometry: "+path);
    return out;
}
std::vector<Triangle> box(const V3& size){
    if(!size.allFinite() || size.minCoeff()<=0)throw std::runtime_error("invalid baffle primitive size");
    std::vector<Triangle> out;V3 p[8];for(int i=0;i<8;++i)p[i]=V3(i&1?1:-1,i&2?1:-1,i&4?1:-1).cwiseProduct(size)/2;
    const int faces[12][3]={{0,1,3},{0,3,2},{4,6,7},{4,7,5},{0,4,5},{0,5,1},{2,3,7},{2,7,6},{0,2,6},{0,6,4},{1,5,7},{1,7,3}};
    for(const auto& f:faces)out.push_back({p[f[0]],p[f[1]],p[f[2]]});return out;
}
// Odd-ray solid containment detects a baffle aperture boundary enclosed by a closed body.
bool contains(const std::vector<Triangle>& triangles,const V3& p){
    const V3 ray=V3(1,0.3713906763541037,0.6947465906068658).normalized();std::vector<double> hits;
    for(const auto& t:triangles){const V3 e=t[1]-t[0],f=t[2]-t[0],h=ray.cross(f);double det=e.dot(h);if(std::abs(det)<1e-12)continue;V3 s=p-t[0];double u=s.dot(h)/det;if(u<0||u>1)continue;V3 q=s.cross(e);double v=ray.dot(q)/det;if(v<0||u+v>1)continue;double d=f.dot(q)/det;if(d>1e-9)hits.push_back(d);}
    std::sort(hits.begin(),hits.end());hits.erase(std::unique(hits.begin(),hits.end(),[](double a,double b){return std::abs(a-b)<1e-8;}),hits.end());return hits.size()%2;
}
}
void VirtualBaffleEnvironment::build(const simulation_runtime::ProjectSimulationRuntime& runtime){
    gpu_.reset();gpuBodyOffsets_.clear();gpuEdgeOffsets_.clear();gpuFallbackBatches_=0;cpuSearchQueries_=0;
    baffles_.clear();searchIndices_.clear();bodies_.clear();error_.clear();nlohmann::json settings=nlohmann::json::object();
    try {
        for(const auto& e:runtime.document().extensions)if(e.key==extensionKey)settings=nlohmann::json::parse(e.serializedPayload);
        if(!settings.value("enabled",true))return;
        double thickness=settings.value("thickness_mm",1.)/1000.,residual=settings.value("fit_tolerance_mm",5.)/1000.;
        for(const auto& object:runtime.objects()){
            if(object.objectType!="workpiece" || !object.collisionEnabled)continue;
            WorkpieceBaffle b;b.objectId=object.documentId;b.visible=settings.value("visible",true);
            try {
                static std::mutex cacheMutex;static std::map<std::string,ModelTrajPlan::VirtualBaffle> cache;
                std::ostringstream key;key<<object.sourcePath.u8string()<<'|'<<std::filesystem::last_write_time(object.sourcePath).time_since_epoch().count()<<'|'<<std::filesystem::file_size(object.sourcePath)<<'|'<<std::setprecision(17)<<object.collisionScale<<'|'<<thickness<<'|'<<residual;
                std::lock_guard<std::mutex> lock(cacheMutex);
                const auto cached=cache.find(key.str());
                if(cached!=cache.end()){b.geometry=cached->second;baffles_.push_back(std::move(b));continue;}
                ModelTrajPlan::ModelData data;auto tris=mesh(object.sourcePath.u8string(),V3::Constant(object.collisionScale));
                for(const auto& t:tris)for(const auto& p:t){data.mainModelData_.m_indices.push_back(static_cast<unsigned>(data.mainModelData_.m_vertices.size()/3));for(int j=0;j<3;++j)data.mainModelData_.m_vertices.push_back(float(p[j]));}
                b.geometry=ModelTrajPlan::buildVirtualBaffle(data,thickness,residual);
                if(cache.size()>=16)cache.clear();cache.emplace(key.str(),b.geometry);
            }catch(const std::exception& e){b.geometry.diagnostic=e.what();}
            baffles_.push_back(std::move(b));
        }
        for(const auto& b:baffles_) {
            detail::BaffleRimIndex::Rings rings;
            for(const auto& ring:b.geometry.contours) {
                rings.emplace_back();
                for(const auto& uv:ring)rings.back().push_back({uv.x(),uv.y()});
            }
            searchIndices_.push_back(std::make_shared<detail::BaffleRimIndex>(rings));
        }
        if(std::none_of(baffles_.begin(),baffles_.end(),[](const auto& b){return b.geometry.enabled;}))return;
        for(const auto& robot:runtime.robots())for(const auto& entry:robot.model.links){const auto& link=entry.second;
            for(const auto& g:link.collisions){if(!g.enabled)continue;Body body;body.robotId=robot.documentId;body.link=link.name;body.local=g.T_part;body.margin=g.inflationMargin;if(!std::isfinite(body.margin)||body.margin<0)throw std::runtime_error("invalid collision inflation margin");
                switch(g.type){
                    case robot::RobotGeometryType::Mesh:body.triangles=mesh(g.meshPath,g.meshScale);break;
                    case robot::RobotGeometryType::Box:body.triangles=box(g.boxSize);break;
                    // Circumscribed boxes are conservative for analytic round primitives; never an inscribed mesh.
                    case robot::RobotGeometryType::Sphere:body.triangles=box(V3::Constant(2*g.radius));break;
                    case robot::RobotGeometryType::Cylinder:body.triangles=box(V3(2*g.radius,2*g.radius,g.length));break;
                    default:throw std::runtime_error("unsupported baffle collision element: "+link.name);
                }bodies_.push_back(std::move(body));
            }
            if(link.collisions.empty() && !link.visuals.empty())throw std::runtime_error("missing link collision geometry: "+link.name);
        }
        for(const auto& a:runtime.mountedAttachments())if(a.owner.valid && (a.kind=="tool" || a.kind=="gun" || a.type=="tool" || a.kind=="toolAttachment" || !a.visual.visualPath.empty())){
            Body body;body.attachmentId=a.documentId;body.robotId=a.owner.robotId;
            body.triangles=mesh(a.visual.visualPath.u8string(),V3::Constant(a.visual.visualScale));bodies_.push_back(std::move(body));
        }
        for(auto& body:bodies_)for(const auto& t:body.triangles)for(const auto& p:t){body.lower=body.lower.cwiseMin(p);body.upper=body.upper.cwiseMax(p);}
        for(auto& body:bodies_) {
            body.triangleOrder.resize(body.triangles.size());std::iota(body.triangleOrder.begin(),body.triangleOrder.end(),0);
            std::function<int(std::size_t,std::size_t)> buildNode;
            buildNode=[&](std::size_t first,std::size_t count){
                const auto index=int(body.nodes.size());body.nodes.emplace_back();Body::Node node;
                node.first=first;node.count=count;node.lower=V3::Constant(1e300);node.upper=V3::Constant(-1e300);
                for(std::size_t i=first;i<first+count;++i)for(const auto& v:body.triangles[body.triangleOrder[i]]){node.lower=node.lower.cwiseMin(v);node.upper=node.upper.cwiseMax(v);}
                if(count>16){Eigen::Index axis; (node.upper-node.lower).maxCoeff(&axis);const auto middle=first+count/2;
                    std::nth_element(body.triangleOrder.begin()+first,body.triangleOrder.begin()+middle,body.triangleOrder.begin()+first+count,[&](std::size_t a,std::size_t b){const auto& x=body.triangles[a];const auto& y=body.triangles[b];return x[0][axis]+x[1][axis]+x[2][axis]<y[0][axis]+y[1][axis]+y[2][axis];});
                    node.left=buildNode(first,count/2);node.right=buildNode(middle,count-count/2);}
                body.nodes[index]=node;return index;
            };
            if(!body.triangles.empty())buildNode(0,body.triangles.size());
        }
        if(bodies_.empty() && !runtime.robots().empty())throw std::runtime_error("no full-robot geometry available for baffle query");
    }catch(const std::exception& e){error_=e.what();}
}
bool VirtualBaffleEnvironment::validState(const simulation_runtime::ProjectSimulationRuntime& runtime,double clearance,std::string& reason)const{
    if(!error_.empty()){reason="virtual_baffle_unavailable: "+error_;return false;}
    for(const auto& b:baffles_){if(!b.geometry.enabled)continue;const auto* object=runtime.object(b.objectId);if(!object){reason="virtual_baffle_workpiece_missing";return false;}
        const auto fromWorld=object->transform.inverse();
        for(const auto& body:bodies_){Eigen::Isometry3d world;
            if(body.attachmentId.empty()){const auto* robot=runtime.robot(body.robotId);if(!robot||!robot->instance){reason="baffle robot missing";return false;}world=robot->instance->getLinkTransform(body.link)*body.local;}
            else {const auto* a=runtime.mountedAttachment(body.attachmentId);if(!a){reason="baffle tool missing";return false;}world=a->transform.worldVisual;}
            const Eigen::Isometry3d tf=fromWorld*world;
            double minBound=1e300,maxBound=-1e300;
            for(int i=0;i<8;++i){V3 p;for(int j=0;j<3;++j)p[j]=i&(1<<j)?body.upper[j]:body.lower[j];double z=b.geometry.normal.dot(tf*p-b.geometry.center);minBound=std::min(minBound,z);maxBound=std::max(maxBound,z);}
            const double margin=clearance+body.margin;
            if(minBound>b.geometry.thickness/2+margin || maxBound<-b.geometry.thickness/2-margin)continue;
            const V3 normalLocal=tf.linear().transpose()*b.geometry.normal;
            const double offset=b.geometry.normal.dot(tf.translation()-b.geometry.center);
            const double half=b.geometry.thickness/2+margin;
            bool hit=false;
            std::vector<int> stack{0};
            while(!stack.empty() && !hit){const auto index=stack.back();stack.pop_back();const auto& node=body.nodes[index];
                const V3 center=(node.lower+node.upper)/2,extent=(node.upper-node.lower)/2;
                const double middle=normalLocal.dot(center)+offset,radius=normalLocal.cwiseAbs().dot(extent);
                if(middle-radius>half+1e-12 || middle+radius<-half-1e-12)continue;
                if(node.left>=0){stack.push_back(node.right);stack.push_back(node.left);continue;}
                for(std::size_t k=node.first;k<node.first+node.count;++k){const auto& triangle=body.triangles[body.triangleOrder[k]];
                    double low=1e300,high=-1e300;
                    for(const auto& p:triangle){const double z=normalLocal.dot(p)+offset;low=std::min(low,z);high=std::max(high,z);}
                    if(low>half+1e-12 || high<-half-1e-12)continue;
                    Triangle transformed;for(int j=0;j<3;++j)transformed[j]=tf*triangle[j];
                    if(b.geometry.intersectsTriangle(transformed,margin)){hit=true;break;}
                }
            }
            if(!hit) {
                // A closed body can enclose a rim without intersecting any triangle.
                // Build the full transformed mesh only for potentially enclosed rim points.
                V3 boundLo=V3::Constant(1e300),boundHi=V3::Constant(-1e300);
                for(int i=0;i<8;++i){V3 p;for(int j=0;j<3;++j)p[j]=i&(1<<j)?body.upper[j]:body.lower[j];p=tf*p;boundLo=boundLo.cwiseMin(p);boundHi=boundHi.cwiseMax(p);}
                std::vector<V3> probes;
                for(const auto& ring:b.geometry.contours)for(const auto& uv:ring){const V3 p=b.geometry.center+b.geometry.axisX*uv.x()+b.geometry.axisY*uv.y();
                    if((p.array()>=boundLo.array()).all() && (p.array()<=boundHi.array()).all())probes.push_back(p);}
                if(!probes.empty()) {
                    const V3 worldRay=V3(1,0.3713906763541037,0.6947465906068658).normalized();
                    const V3 rayLocal=tf.linear().transpose()*worldRay;
                    for(const auto& p:probes){
                        const V3 origin=tf.inverse()*p;std::vector<Triangle> candidates;stack={0};
                        while(!stack.empty()){const auto index=stack.back();stack.pop_back();const auto& node=body.nodes[index];
                            double low=0,high=1e300;bool separated=false;
                            for(int j=0;j<3;++j){if(std::abs(rayLocal[j])<1e-15){if(origin[j]<node.lower[j]-1e-10 || origin[j]>node.upper[j]+1e-10)separated=true;}
                                else {double a=(node.lower[j]-1e-10-origin[j])/rayLocal[j],c=(node.upper[j]+1e-10-origin[j])/rayLocal[j];if(a>c)std::swap(a,c);low=std::max(low,a);high=std::min(high,c);}}
                            if(separated || low>high)continue;
                            if(node.left>=0){stack.push_back(node.right);stack.push_back(node.left);continue;}
                            for(std::size_t k=node.first;k<node.first+node.count;++k){Triangle triangle;for(int j=0;j<3;++j)triangle[j]=tf*body.triangles[body.triangleOrder[k]][j];candidates.push_back(triangle);}
                        }
                        if(!contains(candidates,p))continue;
                        // Retain the original tight world-AABB guard for open meshes.
                        V3 lo=V3::Constant(1e300),hi=V3::Constant(-1e300);
                        for(const auto& triangle:body.triangles)for(const auto& v:triangle){const V3 x=tf*v;lo=lo.cwiseMin(x);hi=hi.cwiseMax(x);}
                        if((p.array()>=lo.array()).all() && (p.array()<=hi.array()).all()){hit=true;break;}
                    }
                }
            }
            if(hit){reason="virtual_baffle_collision: "+b.objectId+" / "+body.robotId+" / "+(body.attachmentId.empty()?body.link:body.attachmentId);return false;}
        }
    }return true;
}
VirtualBaffleEnvironment::SearchPose VirtualBaffleEnvironment::captureSearchPose(const simulation_runtime::ProjectSimulationRuntime& runtime)const{
    SearchPose pose;
    if(!error_.empty())return pose;
    for(const auto& b:baffles_){const auto* object=runtime.object(b.objectId);if(!object && b.geometry.enabled)return pose;pose.objects.push_back(object?object->transform:Eigen::Isometry3d::Identity());}
    for(const auto& body:bodies_){Eigen::Isometry3d world;
        if(body.attachmentId.empty()){const auto* robot=runtime.robot(body.robotId);if(!robot||!robot->instance)return pose;world=robot->instance->getLinkTransform(body.link)*body.local;}
        else {const auto* a=runtime.mountedAttachment(body.attachmentId);if(!a)return pose;world=a->transform.worldVisual;}
        if(!world.matrix().allFinite())return pose;pose.bodies.push_back(world);
    }
    pose.valid=true;return pose;
}
double VirtualBaffleEnvironment::searchViolation(const simulation_runtime::ProjectSimulationRuntime& runtime,double clearance)const{
    return searchViolation(captureSearchPose(runtime),clearance);
}
double VirtualBaffleEnvironment::searchViolation(const SearchPose& pose,double clearance)const{
    ++cpuSearchQueries_;
    if(!pose.valid || pose.bodies.size()!=bodies_.size() || pose.objects.size()!=baffles_.size())return std::numeric_limits<double>::infinity();
    if(!error_.empty() || !std::isfinite(clearance) || clearance<0)return std::numeric_limits<double>::infinity();
    double worst=0.;
    for(std::size_t baffleIndex=0;baffleIndex<baffles_.size();++baffleIndex){const auto& b=baffles_[baffleIndex];if(!b.geometry.enabled)continue;
        const auto& g=b.geometry;const auto& rim=*searchIndices_.at(baffleIndex);
        for(std::size_t bodyIndex=0;bodyIndex<bodies_.size();++bodyIndex){const auto& body=bodies_[bodyIndex];
            const Eigen::Isometry3d tf=pose.objects[baffleIndex].inverse()*pose.bodies[bodyIndex];const double half=g.thickness/2+clearance+body.margin;
            double low=1e300,high=-1e300;
            for(int k=0;k<8;++k){V3 p;for(int j=0;j<3;++j)p[j]=k&(1<<j)?body.upper[j]:body.lower[j];const double z=g.normal.dot(tf*p-g.center);low=std::min(low,z);high=std::max(high,z);}
            if(low>half || high<-half)continue;
            int hits=0;double outside=0.;
            const V3 normalLocal=tf.linear().transpose()*g.normal;
            const double planeOffset=g.normal.dot(tf.translation()-g.center);
            for(const auto& triangle:body.triangles){Triangle t;double triangleLow=1e300,triangleHigh=-1e300;
                for(int j=0;j<3;++j){const double z=normalLocal.dot(triangle[j])+planeOffset;triangleLow=std::min(triangleLow,z);triangleHigh=std::max(triangleHigh,z);}
                // Exact slab separation: these triangles would clip to empty.
                // Avoid repeated temporary polygon allocations in search feedback.
                // The independent validState collision test is unchanged.
                if(triangleLow>half+1e-12 || triangleHigh<-half-1e-12)continue;
                for(int j=0;j<3;++j)t[j]=tf*triangle[j];
                // Clip the actual triangle to the actual baffle slab. Distances
                // below are to its aperture rims, not to unrelated STL detectors.
                std::vector<V3> poly(t.begin(),t.end());
                for(double sign:{1.,-1.}){std::vector<V3> next;if(poly.empty())break;V3 a=poly.back();double da=sign*g.normal.dot(a-g.center)-half;
                    for(const auto& p:poly){double d=sign*g.normal.dot(p-g.center)-half;if((d<=0)!=(da<=0))next.push_back(a+da/(da-d)*(p-a));if(d<=0)next.push_back(p);a=p;da=d;}poly=std::move(next);}
                if(poly.empty())continue;
                bool hasOutsideVertex=false;
                for(const auto& p:poly) {
                    const V3 delta=p-g.center;
                    const detail::BaffleRimIndex::Point uv={delta.dot(g.axisX),delta.dot(g.axisY)};
                    if(rim.contains(uv))continue;
                    hasOutsideVertex=true;
                    const double nearest=rim.distance(uv);
                    if(nearest<1e299)outside=std::max(outside,nearest);
                }
                // Keep original edge/contour checks when all clipped vertices
                // lie in the aperture. Outside vertices already imply a hit.
                if(hasOutsideVertex || g.intersectsTriangle(t,clearance+body.margin))++hits;
            }
            // Mesh-intersection fraction resolves equal rim distances. Solid
            // containment remains owned by validState; zero merit is not safety.
            worst=std::max(worst,outside+.001*hits/std::max<std::size_t>(1,body.triangles.size()));
        }
    }return worst;
}
std::vector<double> VirtualBaffleEnvironment::searchViolationBatch(const std::vector<SearchPose>& poses,double clearance,bool gpu)const{
    std::vector<double> result(poses.size(),0.);
    const auto cpu=[&](){for(std::size_t i=0;i<poses.size();++i)result[i]=searchViolation(poses[i],clearance);return result;};
    if(!gpu || !error_.empty() || !std::isfinite(clearance) || clearance<0)return cpu();
    try {
        if(!gpu_){
            std::vector<double> triangles,edges;gpuBodyOffsets_.clear();gpuEdgeOffsets_.clear();
            for(const auto& body:bodies_){gpuBodyOffsets_.push_back(int(triangles.size()/9));for(const auto& t:body.triangles)for(const auto& p:t)for(int j=0;j<3;++j)triangles.push_back(p[j]);}
            for(const auto& b:baffles_){gpuEdgeOffsets_.push_back(int(edges.size()/4));for(const auto& ring:b.geometry.contours)for(std::size_t j=0;j<ring.size();++j){const auto& a=ring[j];const auto& c=ring[(j+1)%ring.size()];edges.insert(edges.end(),{a.x(),a.y(),c.x(),c.y()});}}
            gpuEdgeOffsets_.push_back(int(edges.size()/4));auto backend=std::make_shared<detail::GpuBaffleSearch>();backend->configure(std::move(triangles),std::move(edges));gpu_=std::move(backend);
        }
        std::vector<detail::GpuBaffleTask> tasks;std::vector<std::size_t> owners;
        for(std::size_t i=0;i<poses.size();++i){const auto& pose=poses[i];
            if(!pose.valid || pose.bodies.size()!=bodies_.size() || pose.objects.size()!=baffles_.size())return cpu();
            for(std::size_t j=0;j<baffles_.size();++j){const auto& g=baffles_[j].geometry;if(!g.enabled)continue;
                for(std::size_t k=0;k<bodies_.size();++k){const auto& body=bodies_[k];if(body.triangles.empty())continue;
                    const Eigen::Isometry3d tf=pose.objects[j].inverse()*pose.bodies[k];const double margin=clearance+body.margin,half=g.thickness/2+margin;
                    double low=1e300,high=-1e300;for(int corner=0;corner<8;++corner){V3 p;for(int a=0;a<3;++a)p[a]=corner&(1<<a)?body.upper[a]:body.lower[a];double z=g.normal.dot(tf*p-g.center);low=std::min(low,z);high=std::max(high,z);}
                    if(low>half || high<-half)continue;
                    detail::GpuBaffleTask t;t.triangleBegin=gpuBodyOffsets_[k];t.triangleCount=int(body.triangles.size());t.edgeBegin=gpuEdgeOffsets_[j];t.edgeCount=gpuEdgeOffsets_[j+1]-gpuEdgeOffsets_[j];
                    for(int a=0;a<3;++a)for(int b=0;b<4;++b)t.geometry[4*a+b]=tf.matrix()(a,b);
                    for(int a=0;a<3;++a){t.geometry[12+a]=g.center[a];t.geometry[15+a]=g.normal[a];t.geometry[18+a]=g.axisX[a];t.geometry[21+a]=g.axisY[a];}
                    t.geometry[24]=half;t.geometry[25]=margin;
                    // Conservative CPU hierarchy selects candidates; CUDA retains
                    // exact full-mesh clipping and original hit normalization.
                    const V3 localNormal=tf.linear().transpose()*g.normal;
                    const double offset=g.normal.dot(tf.translation()-g.center);
                    std::vector<int> pending{0};
                    while(!pending.empty()){const auto n=pending.back();pending.pop_back();const auto& node=body.nodes[n];
                        const double middle=localNormal.dot((node.lower+node.upper)/2)+offset;
                        const double radius=localNormal.cwiseAbs().dot((node.upper-node.lower)/2);
                        if(middle-radius>half+1e-10 || middle+radius<-half-1e-10)continue;
                        if(node.left>=0){pending.push_back(node.right);pending.push_back(node.left);continue;}
                        for(std::size_t index=node.first;index<node.first+node.count;++index){const auto triangle=body.triangleOrder[index];double low=1e300,high=-1e300;
                            for(const auto& point:body.triangles[triangle]){const double z=localNormal.dot(point)+offset;low=std::min(low,z);high=std::max(high,z);}
                            if(low<=half+1e-10 && high>=-half-1e-10)t.triangleIndices.push_back(int(triangle));}
                    }
                    if(t.triangleIndices.empty())continue;
                    if(t.triangleIndices.size()==body.triangles.size())t.triangleIndices.clear();
                    tasks.push_back(std::move(t));owners.push_back(i);
                }
            }
        }
        std::vector<double> merits;
        if(!gpu_->evaluate(tasks,merits)){++gpuFallbackBatches_;return cpu();}
        for(std::size_t i=0;i<merits.size();++i)result[owners[i]]=std::max(result[owners[i]],merits[i]);return result;
    }catch(const std::exception&){++gpuFallbackBatches_;return cpu();}
}
nlohmann::json VirtualBaffleEnvironment::gpuSearchStatus()const{
    auto out=gpu_?gpu_->status():nlohmann::json{{"attempted",false},{"available",false},{"batch_queries",0}};
    out["fallback_batches"]=gpuFallbackBatches_;out["cpu_merit_queries"]=cpuSearchQueries_;return out;
}
nlohmann::json VirtualBaffleEnvironment::status()const{
    nlohmann::json out={{"enabled",true},{"thickness_mm",1.},{"infinite",true},{"primitive_policy","round primitives use conservative circumscribed boxes"},{"error",error_},{"objects",nlohmann::json::array()}};
    for(const auto& b:baffles_){
        const auto v=[](const V3& p){return nlohmann::json{p.x(),p.y(),p.z()};};
        nlohmann::json rings=nlohmann::json::array();for(const auto& ring:b.geometry.contours){nlohmann::json r=nlohmann::json::array();for(const auto& p:ring)r.push_back({p.x(),p.y()});rings.push_back(r);}
        out["objects"].push_back({{"object_id",b.objectId},{"enabled",b.geometry.enabled},{"visible",b.visible},{"thickness_mm",b.geometry.thickness*1000},{"diagnostic",b.geometry.diagnostic},{"boundary_count",b.geometry.topology.loopCount},{"branches",b.geometry.topology.branchVertices},{"fit_rms_mm",b.geometry.rmsResidual*1000},{"fit_max_mm",b.geometry.maxResidual*1000},{"section_count",b.geometry.contours.size()},
            {"center_local_m",v(b.geometry.center)},{"normal_local",v(b.geometry.normal)},
            {"axis_x_local",v(b.geometry.axisX)},{"axis_y_local",v(b.geometry.axisY)},{"aperture_contours_m",rings}});
    }return out;
}
}
