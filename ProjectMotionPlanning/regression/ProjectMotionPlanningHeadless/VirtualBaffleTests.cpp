#include <SlicingTrajPlan/VirtualBaffle.h>
#include <Eigen/Geometry>
#include <iostream>
#include <stdexcept>
#include <ProjectMotionPlanning/VirtualBaffleEnvironment.h>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <random>
#include <chrono>
#include <iomanip>
#include <limits>

namespace {
using V3=Eigen::Vector3d;
using ModelTrajPlan::VirtualBaffle;
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
ModelTrajPlan::ModelData tube(const Eigen::Isometry3d& pose=Eigen::Isometry3d::Identity()){
    ModelTrajPlan::ModelData model;
    V3 p[8];
    const double x[4]={-.1,.1,.1,-.1},y[4]={-.05,-.05,.05,.05};
    for(int i=0;i<8;++i)p[i]=pose*V3(x[i%4],y[i%4],i<4?-.02:.02);
    const auto tri=[&](int a,int b,int c){for(int k:{a,b,c}){
        model.mainModelData_.m_indices.push_back(static_cast<unsigned>(model.mainModelData_.m_vertices.size()/3));
        for(int j=0;j<3;++j)model.mainModelData_.m_vertices.push_back(float(p[k][j]));}};
    for(int i=0;i<4;++i){int j=(i+1)%4;tri(i,j,j+4);tri(i,j+4,i+4);}
    return model;
}

bool referenceContains(const std::vector<VirtualBaffle::Triangle>& triangles,const V3& p){
    const V3 ray=V3(1,.3713906763541037,.6947465906068658).normalized();std::vector<double> hits;
    for(const auto& t:triangles){const V3 e=t[1]-t[0],f=t[2]-t[0],h=ray.cross(f);const double d=e.dot(h);if(std::abs(d)<1e-12)continue;
        const V3 s=p-t[0];const double u=s.dot(h)/d;if(u<0||u>1)continue;const V3 q=s.cross(e);const double v=ray.dot(q)/d;if(v<0||u+v>1)continue;const double distance=f.dot(q)/d;if(distance>1e-9)hits.push_back(distance);}
    std::sort(hits.begin(),hits.end());hits.erase(std::unique(hits.begin(),hits.end(),[](double a,double b){return std::abs(a-b)<1e-8;}),hits.end());return hits.size()%2;
}
void environmentRegression(){
    const auto dir=std::filesystem::temp_directory_path()/("rs2026-baffle-bvh-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));std::filesystem::create_directories(dir);
    std::vector<VirtualBaffle::Triangle> body;
    const int faces[][3]={{0,1,3},{0,3,2},{4,6,7},{4,7,5},{0,4,5},{0,5,1},{2,3,7},{2,7,6},{0,2,6},{0,6,4},{1,5,7},{1,7,3}};
    for(int box=0;box<32;++box){V3 vertices[8];const V3 center(.025*(box%4),.025*((box/4)%4),.025*(box/16));
        for(int i=0;i<8;++i)vertices[i]=center+V3(i&1?.008:-.008,i&2?.008:-.008,i&4?.008:-.008);
        for(const auto& face:faces)body.push_back({vertices[face[0]],vertices[face[1]],vertices[face[2]]});}
    const auto save=[&](const std::filesystem::path& path,const std::vector<VirtualBaffle::Triangle>& mesh){std::ofstream file(path);file<<std::setprecision(17)<<"solid test\n";
        for(const auto& triangle:mesh){file<<"facet normal 0 0 0\nouter loop\n";for(const auto& v:triangle)file<<"vertex "<<v.x()<<' '<<v.y()<<' '<<v.z()<<'\n';file<<"endloop\nendfacet\n";}file<<"endsolid test\n";};
    save(dir/"body.stl",body);std::vector<VirtualBaffle::Triangle> opening;const auto model=tube();const auto& data=model.mainModelData_;
    for(std::size_t i=0;i<data.m_indices.size();i+=3){VirtualBaffle::Triangle triangle;for(int j=0;j<3;++j){const auto k=3*data.m_indices[i+j];triangle[j]=V3(data.m_vertices[k],data.m_vertices[k+1],data.m_vertices[k+2]);}opening.push_back(triangle);}save(dir/"opening.stl",opening);
    {std::ofstream file(dir/"robot.urdf");file<<"<robot name='test'><link name='base'><collision><geometry><mesh filename='body.stl'/></geometry></collision></link></robot>";}
    simulation_project::ProjectDocument document;simulation_project::RobotDesc robot;robot.id="robot";robot.sourceType="urdf";robot.sourcePath=(dir/"robot.urdf").generic_u8string();document.robots.push_back(robot);
    simulation_project::SceneObjectDesc object;object.id="opening";object.objectType="workpiece";object.sourcePath=(dir/"opening.stl").generic_u8string();document.objects.push_back(object);
    simulation_runtime::ProjectSimulationRuntime runtime;const auto loaded=runtime.loadProject(document,dir);require(loaded.success,loaded.message.c_str());
    motion_planning::VirtualBaffleEnvironment environment;environment.build(runtime);require(environment.baffles().size()==1 && environment.baffles().front().geometry.enabled,"Baffle BVH fixture unavailable");
    const auto& baffle=environment.baffles().front().geometry;
    std::vector<motion_planning::VirtualBaffleEnvironment::SearchPose> searchPoses;
    std::mt19937 rng(1701);std::uniform_real_distribution<double> distribution(-1.,1.);int validCount=0,hitCount=0;
    for(int sample=0;sample<512;++sample){Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=V3(.2*distribution(rng),.15*distribution(rng),.06*distribution(rng));pose.linear()=Eigen::AngleAxisd(3*distribution(rng),V3(1,2,3).normalized()).toRotationMatrix();
        simulation_project::TransformDesc transform;transform.x=pose.translation().x();transform.y=pose.translation().y();transform.z=pose.translation().z();
        const auto angles=pose.linear().eulerAngles(2,1,0);transform.yaw=angles[0];transform.pitch=angles[1];transform.roll=angles[2];
        const auto moved=runtime.setRobotBaseTransform("robot",transform);require(moved.success,moved.message.c_str());const auto captured=environment.captureSearchPose(runtime);searchPoses.push_back(captured);pose=captured.bodies.front();
        const double clearance=(sample%3)*.001;std::vector<VirtualBaffle::Triangle> transformed;V3 lower=V3::Constant(1e300),upper=V3::Constant(-1e300);bool hit=false;
        for(const auto& triangle:body){VirtualBaffle::Triangle world;for(int j=0;j<3;++j){world[j]=pose*triangle[j];lower=lower.cwiseMin(world[j]);upper=upper.cwiseMax(world[j]);}hit=hit||baffle.intersectsTriangle(world,clearance);transformed.push_back(world);}
        if(!hit)for(const auto& ring:baffle.contours)for(const auto& uv:ring){const V3 p=baffle.center+baffle.axisX*uv.x()+baffle.axisY*uv.y();if((p.array()>=lower.array()).all() && (p.array()<=upper.array()).all() && referenceContains(transformed,p))hit=true;}
        std::string reason;const bool valid=environment.validState(runtime,clearance,reason);if(valid)++validCount;else ++hitCount;require(valid==!hit,"BVH/slab/containment differs from brute-force full mesh");
    }
    require(validCount>0 && hitCount>0,"BVH regression must include passing and rejected states");
    const auto batch=environment.searchViolationBatch(searchPoses,.001,true);
    const auto cpu=environment.searchViolationBatch(searchPoses,.001,false);
    for(std::size_t first=0;first<searchPoses.size();first+=32) {
        const auto last=std::min(searchPoses.size(),first+32);
        const std::vector<motion_planning::VirtualBaffleEnvironment::SearchPose> small(searchPoses.begin()+first,searchPoses.begin()+last);
        const auto serial=environment.searchViolationBatch(small,.001,true);
        for(std::size_t i=0;i<serial.size();++i) {
            require(std::abs(serial[i]-batch[first+i])<1e-12,"Parallel/serial GPU staging changed pose order or merit");
            require(std::abs(cpu[first+i]-batch[first+i])<1e-10,"GPU staging differs from independent CPU baffle score");
        }
    }
    std::cout<<"512 parallel/serial/CPU baffle merit comparisons passed "<<environment.gpuSearchStatus().dump()<<'\n';
    for(const char* file:{"robot.urdf","body.stl","opening.stl"})std::filesystem::remove(dir/file);std::filesystem::remove(dir);
    std::cout<<"512 independent full-mesh/BVH comparisons passed\n";
}
}
int main(){try{
    const auto b=ModelTrajPlan::buildVirtualBaffle(tube());
    require(b.enabled,b.diagnostic.c_str());require(b.topology.loopCount==2,"two rims");
    require(b.center.norm()<1e-6,"center");require(std::abs(b.normal.z())>.999,"plane fit");
    require(!b.intersectsTriangle({V3(0,0,0),V3(.01,0,0),V3(0,.01,0)}),"aperture permits body");
    require(b.intersectsTriangle({V3(.2,0,0),V3(.21,0,0),V3(.2,.01,0)}),"infinite forbidden slab");
    require(!b.intersectsTriangle({V3(.2,0,.01),V3(.21,0,.01),V3(.2,.01,.01)}),"outside slab");
    require(b.intersectsTriangle({V3(.095,0,0),V3(.096,0,0),V3(.095,.001,0)},.01),"clearance shrinks aperture");
    Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=V3(.3,-.4,.2);
    pose.linear()=Eigen::AngleAxisd(.7,V3(1,2,3).normalized()).toRotationMatrix();
    const auto moved=ModelTrajPlan::buildVirtualBaffle(tube(pose));require(moved.enabled,moved.diagnostic.c_str());
    require((moved.center-pose.translation()).norm()<1e-5,"rigid transform fit");
    require(std::abs(moved.normal.dot(pose.linear().col(2)))>.999,"rotated plane");
    VirtualBaffle hole=b;hole.contours.push_back({{-.01,-.01},{.01,-.01},{.01,.01},{-.01,.01}});
    require(hole.intersectsTriangle({V3(0,0,0),V3(.001,0,0),V3(0,.001,0)}),"nested hole is forbidden");
    require(!ModelTrajPlan::buildVirtualBaffle(ModelTrajPlan::ModelData()).enabled,"empty geometry disabled");
    const auto display=b.displayTriangles(.3);require(!display.empty(),"display geometry");
    for(const auto& t:display){const V3 p=(t[0]+t[1]+t[2])/3;
        require(!(std::abs(p.x())<.09999&&std::abs(p.y())<.04999),"display preserves aperture");}
    // Symmetric circular rims remain valid under correspondence selection.
    ModelTrajPlan::ModelData circle;
    constexpr int count=64;
    auto& mesh=circle.mainModelData_;
    for(int layer=0;layer<2;++layer)for(int i=0;i<count;++i){
        const double angle=2*std::acos(-1.)*i/count;
        mesh.m_vertices.insert(mesh.m_vertices.end(),{float(.1*std::cos(angle)),float(.1*std::sin(angle)),layer?.02f:-.02f});
    }
    for(unsigned i=0;i<count;++i){const unsigned j=(i+1)%count;
        mesh.m_indices.insert(mesh.m_indices.end(),{i,j,j+count,i,j+count,i+count});}
    const auto symmetric=ModelTrajPlan::buildVirtualBaffle(circle);
    require(symmetric.enabled,symmetric.diagnostic.c_str());
    require(std::abs(symmetric.normal.z())>.999 && symmetric.center.norm()<1e-6,"symmetric-rim plane remains valid");
    VirtualBaffle aperture;aperture.enabled=true;aperture.contours={{{-1,-1},{1,-1},{1,1},{-1,1}}};
    require(!aperture.apertureContains({1,0}) && !aperture.apertureContains({-1,0}),"aperture boundary contact is forbidden on both sides");
    require(aperture.apertureContains({0,0}),"boundary rule preserves the aperture interior");
    const double nan=std::numeric_limits<double>::quiet_NaN();
    require(aperture.intersectsTriangle({V3(nan,0,0),V3::Zero(),V3::UnitX()}),"invalid collision input stays rejected");
    environmentRegression();
    std::cout<<"Virtual baffle geometry and collision checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
