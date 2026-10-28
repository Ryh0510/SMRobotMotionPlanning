#include "../../src/GpuBaffleSearch.h"
#include "../../src/BaffleRimIndex.h"
#include "../../src/GpuSerialIk.h"
#include "../../src/SurfacePostureSearch.h"
#include "../../src/SurfaceFeasibilitySampling.h"
#include <SlicingTrajPlan/VirtualBaffle.h>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <random>
#include <chrono>
using Baffle=ModelTrajPlan::VirtualBaffle;
using V3=Eigen::Vector3d;
using motion_planning::detail::GpuBaffleTask;
using motion_planning::detail::GpuBaffleSearch;
nlohmann::json ikRegression(){
    using motion_planning::detail::GpuSerialIk;
    robot::RobotModel model;std::vector<std::string> names;
    for(int i=0;i<6;++i){robot::RobotJoint j;j.name="q"+std::to_string(i);j.parent="l"+std::to_string(i);j.child="l"+std::to_string(i+1);j.type=robot::JointType::Revolute;j.dofIndex=i;model.joints.push_back(j);names.push_back(j.name);}
    const V3 axes[]={V3(0,0,1),V3(0,1,0),V3(0,1,0),V3(1,0,0),V3(0,1,0),V3(1,0,0)};
    const V3 pivots[]={V3(.2,.1,0),V3(.2,.1,.4),V3(.6,.1,.4),V3(.9,.1,.4),V3(1,.1,.4),V3(1.1,.1,.4)};
    Eigen::Isometry3d home=Eigen::Isometry3d::Identity();home.translation()=V3(1.2,.1,.5);home.linear()=Eigen::AngleAxisd(.7,V3(1,2,3).normalized()).toRotationMatrix();
    motion_planning::CartesianMultiIkOptions options;options.model.jointNames=names;options.lower.assign(6,-6.28);options.upper.assign(6,6.28);options.maxIterations=100;options.maxCandidatesPerPoint=128;options.model.damping=.001;
    options.model.worldForwardKinematics=[&](const std::vector<double>& q){Eigen::Isometry3d out=Eigen::Isometry3d::Identity();for(int i=0;i<6;++i){Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.linear()=Eigen::AngleAxisd(q[i],axes[i]).toRotationMatrix();t.translation()=pivots[i]-t.linear()*pivots[i];out=out*t;}return Eigen::Isometry3d(out*home);};
    GpuSerialIk solver;if(!solver.configure(model,names,"l6",options))throw std::runtime_error(solver.status().dump());
    double serialError=0;
    for(int n=0;n<512;++n){std::vector<double> q(6);for(int j=0;j<6;++j)q[j]=6*std::sin((n+1)*(j+1)*.371);serialError=std::max(serialError,(solver.forward(q).matrix()-options.model.worldForwardKinematics(q).matrix()).norm());}
    if(serialError>1e-8)throw std::runtime_error("CPU serial FK differs from original FK");
    double jacobianError=0;
    for(int n=0;n<128;++n){std::vector<double> q(6);for(int j=0;j<6;++j)q[j]=6*std::sin((n+1)*(j+1)*.371);
        Eigen::Matrix<double,6,6> numeric;
        for(int j=0;j<6;++j){auto lo=q,hi=q;lo[j]-=1e-6;hi[j]+=1e-6;const auto a=options.model.worldForwardKinematics(lo),b=options.model.worldForwardKinematics(hi);
            numeric.block<3,1>(0,j)=(b.translation()-a.translation())/2e-6;const Eigen::AngleAxisd aa(b.linear()*a.linear().transpose());numeric.block<3,1>(3,j)=aa.axis()*aa.angle()/2e-6;}
        jacobianError=std::max(jacobianError,(solver.jacobian(q)-numeric).norm());}
    if(jacobianError>1e-7)throw std::runtime_error("Serial geometric Jacobian differs from independent FK derivative");

    std::vector<Eigen::Isometry3d> targets;std::vector<std::vector<double>> seeds;
    std::vector<double> seed{.1,-.2,.5,.2,-.4,.3};seeds.push_back(seed);
    for(int i=0;i<96;++i){auto q=seed;for(int j=0;j<6;++j)q[j]+=.12*std::sin((i+1)*(j+1)*.713);targets.push_back(options.model.worldForwardKinematics(q));}
    std::vector<std::vector<motion_planning::CartesianIkCandidate>> roots;
    if(!solver.solve(targets,seeds,roots))throw std::runtime_error(solver.status().dump());
    int solved=0;double error=0;for(std::size_t i=0;i<roots.size();++i)if(!roots[i].empty()){++solved;for(const auto& c:roots[i])error=std::max(error,(options.model.worldForwardKinematics(c.joints).matrix()-targets[i].matrix()).norm());}
    if(solved!=96 || error>2e-6)throw std::runtime_error("GPU IK reachable-target regression failed: "+std::to_string(solved));
    auto unreachable=home;unreachable.translation()=V3(100,100,100);
    if(!solver.solve({unreachable},seeds,roots) || !roots.front().empty())throw std::runtime_error("GPU IK accepted unreachable target");
    GpuSerialIk unsupported;model.joints.back().isLoop=true;if(unsupported.configure(model,names,"l6",options))throw std::runtime_error("GPU IK accepted loop chain");
    const auto genes=motion_planning::surfacePostureSeedGenes(V3(0,0,1),true,5000);
    if(genes.size()!=5000)throw std::runtime_error("Initial posture seed pool is not 5000");for(const auto& x:genes)if(!x.allFinite() || x.head<3>().norm()>1+1e-12 || std::abs(x[3])>1+1e-12)throw std::runtime_error("Invalid initial posture cone/spin seed");
    const double cone=80*std::acos(-1.)/180;
    const auto full=motion_planning::surfacePostureInitialPool(V3(0,0,1),true,5000,cone,959);
    std::set<std::array<long long,4>> distinct;double maximumTilt=0;std::array<int,10> areaBins{},spinBins{};
    for(const auto& x:full){
        const auto q=motion_planning::surfacePostureDelta(x,V3(0,0,1),true,cone-1e-5);
        if(!distinct.insert(motion_planning::surfacePostureKey(q)).second)throw std::runtime_error("Duplicate physical initial posture");
        const V3 normal=q*V3(0,0,1),surface(.3,-.4,.5),tcp=surface-.1*normal;
        const double tilt=std::acos(std::clamp(normal.z(),-1.,1.));maximumTilt=std::max(maximumTilt,tilt);
        const double fraction=(1-std::cos(tilt))/(1-std::cos(cone-1e-5));++areaBins[std::min(9,int(10*fraction))];++spinBins[std::min(9,int(5*(x[3]+1)))];
        if(std::abs((tcp-surface).norm()-.1)>1e-12 || tilt>cone+1e-12)throw std::runtime_error("Initial TCP outside 100 mm / 80 degree cap");
    }
    for(int n:areaBins)if(n<380 || n>620)throw std::runtime_error("Nonuniform spherical-area initial pool");
    for(int n:spinBins)if(n<380 || n>620)throw std::runtime_error("Nonuniform axial-spin initial pool");
    if(distinct.size()!=5000 || maximumTilt<cone-.001)throw std::runtime_error("Initial cap coverage/uniqueness failure");
    for(double radiusFraction:{0.,.3,.5,.8}){
        std::array<int,10> radiusBins{},normalBins{};
        const V3 axis=V3(1,2,3).normalized();
        for(int n=1;n<=5000;++n){const auto x=motion_planning::surfaceFeasibilitySample(n,axis,true,cone,radiusFraction);
            const auto rotation=motion_planning::surfacePostureDelta(x.head<4>().eval(),axis,true,cone-1e-5);
            const double tilt=std::acos(std::clamp(axis.dot(rotation*axis),-1.,1.)),radius=.1*(1+.8*x[4]);
            if(!x.allFinite() || std::abs(axis.dot(x.head<3>()))>1e-12 || tilt>cone || radius<.1*(1-radiusFraction)-1e-12 || radius>.1*(1+radiusFraction)+1e-12)throw std::runtime_error("Feasibility proposal violates source cap/radius bounds");
            ++normalBins[std::min(9,int(10*(1-std::cos(tilt))/(1-std::cos(cone-1e-5))))];
            if(radiusFraction>0)++radiusBins[std::min(9,int(5*(.8*x[4]/radiusFraction+1)))];
        }
        for(int count:normalBins)if(count<450 || count>550)throw std::runtime_error("Feasibility normal coverage is not equal-area");
        if(radiusFraction>0)for(int count:radiusBins)if(count<450 || count>550)throw std::runtime_error("Feasibility radius coverage is not uniform");
    }
    return {{"reachable_targets",solved},{"cpu_fk_matrix_error_max",error},{"initial_postures",genes.size()},{"uniform_area_bins",areaBins},{"uniform_spin_bins",spinBins},{"distinct_80_degree_postures",distinct.size()},{"maximum_tilt_deg",maximumTilt*180/std::acos(-1.)},{"gpu",solver.status()}};
}
double reference(const Baffle& b,const Baffle::Triangle& triangle,double clearance){
    std::vector<V3> poly(triangle.begin(),triangle.end());
    for(double sign:{1.,-1.}){std::vector<V3> next;if(poly.empty())break;V3 a=poly.back();double da=sign*b.normal.dot(a-b.center)-b.thickness/2-clearance;
        for(const auto& p:poly){const double d=sign*b.normal.dot(p-b.center)-b.thickness/2-clearance;if((d<=0)!=(da<=0))next.push_back(a+da/(da-d)*(p-a));if(d<=0)next.push_back(p);a=p;da=d;}poly=std::move(next);}
    double outside=0;bool hit=false;
    for(const auto& p:poly){const V3 delta=p-b.center;Eigen::Vector2d uv(delta.dot(b.axisX),delta.dot(b.axisY));if(b.apertureContains(uv))continue;hit=true;double near=1e300;
        for(const auto& ring:b.contours)for(std::size_t i=0;i<ring.size();++i){const auto e=(ring[(i+1)%ring.size()]-ring[i]).eval(),d=(uv-ring[i]).eval();const double f=e.squaredNorm()>1e-24?std::clamp(d.dot(e)/e.squaredNorm(),0.,1.):0.;near=std::min(near,(d-f*e).norm());}outside=std::max(outside,near);}
    return outside+.001*(hit || b.intersectsTriangle(triangle,clearance));
}
int main(int argc,char** argv){try{
    Baffle b;b.enabled=true;b.contours={{{-.1,-.1},{.1,-.1},{.1,.1},{-.1,.1}},{{-.02,-.02},{.02,-.02},{.02,.02},{-.02,.02}}};
    if(argc>1 && std::string(argv[1])!="--expect-unavailable") {std::ifstream in(argv[1]);nlohmann::json rings;in>>rings;b.contours.clear();for(const auto& ring:rings){b.contours.emplace_back();for(const auto& p:ring)b.contours.back().emplace_back(p[0].get<double>(),p[1].get<double>());}}
    std::mt19937 rng(1703);std::uniform_real_distribution<double> d(-.5,.5);
    std::vector<Baffle::Triangle> triangles;
    for(int i=0;i<2200;++i){Baffle::Triangle t;for(auto& p:t)p=V3(d(rng),d(rng),d(rng)*.03);triangles.push_back(t);}
    // Coplanar, aperture-contained, crossing, enclosing and exact rim cases.
    triangles.push_back({V3(-.4,-.4,0),V3(.4,-.4,0),V3(0,.4,0)});
    triangles.push_back({V3(.03,.03,0),V3(.04,.03,0),V3(.03,.04,0)});
    triangles.push_back({V3(.03,.03,.2),V3(.04,.03,.2),V3(.03,.04,.2)});
    for(const auto& ring:b.contours)for(const auto& p:ring)triangles.push_back({V3(p.x(),p.y(),0),V3(p.x()+.001,p.y(),.0005),V3(p.x(),p.y()+.001,-.0005)});
    std::vector<double> mesh,edges;for(const auto& t:triangles)for(const auto& p:t)for(int j=0;j<3;++j)mesh.push_back(p[j]);
    for(const auto& ring:b.contours)for(std::size_t i=0;i<ring.size();++i){const auto& a=ring[i];const auto& c=ring[(i+1)%ring.size()];edges.insert(edges.end(),{a.x(),a.y(),c.x(),c.y()});}
    GpuBaffleSearch gpu;gpu.configure(mesh,edges);GpuBaffleTask base;base.triangleCount=1;base.edgeCount=int(edges.size()/4);
    base.geometry={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,0,1,0,.0005,0};
    if(argc>1 && std::string(argv[1])=="--expect-unavailable") {std::vector<double> scores;if(gpu.evaluate({base},scores))throw std::runtime_error("Expected missing-runtime failure");std::cout<<gpu.status().dump()<<'\n';return 0;}
    double maxError=0;std::size_t cases=0;const auto began=std::chrono::steady_clock::now();
    for(double clearance:{0.,.001,.005})for(int transform=0;transform<2;++transform)for(std::size_t first=0;first<triangles.size();first+=512){
        Eigen::Isometry3d tf=Eigen::Isometry3d::Identity();if(transform){tf.linear()=Eigen::AngleAxisd(.17,V3(1,2,3).normalized()).toRotationMatrix();tf.translation()=V3(.013,-.007,.003);}
        std::vector<GpuBaffleTask> tasks;for(std::size_t i=first;i<std::min(triangles.size(),first+512);++i){auto t=base;t.triangleBegin=int(i);t.geometry[24]=b.thickness/2+clearance;t.geometry[25]=clearance;for(int a=0;a<3;++a)for(int k=0;k<4;++k)t.geometry[4*a+k]=tf.matrix()(a,k);tasks.push_back(t);}
        std::vector<double> scores;if(!gpu.evaluate(tasks,scores))throw std::runtime_error(gpu.status().dump());
        for(std::size_t i=0;i<tasks.size();++i){Baffle::Triangle t;for(int k=0;k<3;++k)t[k]=tf*triangles[first+i][k];const double expected=reference(b,t,clearance),error=std::abs(scores[i]-expected);maxError=std::max(maxError,error);++cases;
            if(error>1e-10)throw std::runtime_error("CPU/GPU mismatch at "+std::to_string(cases)+" error "+std::to_string(error));}
    }
    // A task with many triangles must reduce max distance and hit fraction correctly.
    base.triangleCount=int(triangles.size());std::vector<double> combined;
    if(!gpu.evaluate({base},combined))throw std::runtime_error(gpu.status().dump());
    double outside=0;int hits=0;for(const auto& t:triangles){const auto x=reference(b,t,0);const bool hit=x>=.001;if(hit)++hits;outside=std::max(outside,x-(hit?.001:0.));}
    if(std::abs(combined[0]-outside-.001*hits/triangles.size())>1e-10)throw std::runtime_error("Multi-triangle reduction mismatch");
    auto sparse=base;
    for(int i=int(triangles.size())-1;i>=0;--i)sparse.triangleIndices.push_back(i);
    std::vector<double> sparseScore;
    if(!gpu.evaluate({sparse},sparseScore) || std::abs(sparseScore[0]-combined[0])>1e-12)throw std::runtime_error("Sparse GPU full-mesh normalization or indexing mismatch");
    sparse.triangleIndices={0};
    if(!gpu.evaluate({sparse},sparseScore) || std::abs(sparseScore[0]-reference(b,triangles[0],0)+.001*(reference(b,triangles[0],0)>=.001?1.:0.)*(1.-1./triangles.size()))>1e-10)throw std::runtime_error("Sparse GPU merit must retain original full-mesh denominator");

    std::cout<<nlohmann::json{{"cases",cases},{"max_error",maxError},{"elapsed_s",std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count()},{"gpu",gpu.status()},{"ik_regression",ikRegression()}}.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
