#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/SafeTrajectoryDownsampling.h>
#include <nlohmann/json.hpp>
#include <ProjectMotionPlanning/VirtualBaffleEnvironment.h>
#include <ProjectMotionPlanning/ExternalTrajectoryEvaluation.h>
#include <ProjectMotionPlanning/ExternalWorkpiecePlanning.h>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include "../../src/CdfQueryBatch.h"
#include "../../src/PointSearchPopulation.h"
#include "../../src/SurfacePostureSearch.h"
#include "../../src/GaRadiusEvolution.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <set>

using namespace motion_planning;
int main()
{
    std::cout<<std::unitbuf;
    int failures=0;
    const auto check=[&](bool ok,const char* message) { std::cout<<(ok?"PASS ":"FAIL ")<<message<<'\n'; if(!ok) ++failures; };
    {
        robottrajectory::JointTrajectory source;
        for(int i=0;i<9;++i) {
            robottrajectory::TimedJointPoint point;point.time=double(i);point.q={double(i),double(i%2)};
            source.points.push_back(point);
        }
        SafeTrajectoryDownsamplingOptions o;o.targetPoints=3;
        o.edgeAdmissible=[](std::size_t a,std::size_t b){return b-a<=2;};
        const auto safe=downsampleTrajectorySafely(source,o);
        check(safe.success && safe.trajectory.points.size()>o.targetPoints && safe.sourceIndices.front()==0 && safe.sourceIndices.back()==8,
            "Unsafe shortcuts restore necessary safe points above the requested budget");
        bool exact=true;for(std::size_t i=0;i<safe.sourceIndices.size();++i)
            exact &= safe.trajectory.points[i].q==source.points[safe.sourceIndices[i]].q && safe.trajectory.points[i].time==source.points[safe.sourceIndices[i]].time;
        check(exact && source.points.size()==9,"Downsampling preserves endpoint/source encoder values and times without mutating the input");
        o.edgeAdmissible=[](std::size_t,std::size_t){return true;};o.targetPoints=4;
        const auto sparse=downsampleTrajectorySafely(source,o);
        check(sparse.success && sparse.trajectory.points.size()==4,"Clear connections reach the entered target");
        o.targetPoints=3;o.edgeAdmissible=[](std::size_t a,std::size_t b){return !(a==0 && b==4);};
        const auto balanced=downsampleTrajectorySafely(source,o);
        check(balanced.success && balanced.trajectory.points.size()==3,
            "Restored safety bends do not permanently pin redundant uniform seeds; every replacement chord is rechecked");
        o.targetPoints=4;o.edgeAdmissible=[](std::size_t,std::size_t){return true;};
        o.requiredIndices={1,7};
        const auto pinned=downsampleTrajectorySafely(source,o);
        check(pinned.success && std::find(pinned.sourceIndices.begin(),pinned.sourceIndices.end(),1)!=pinned.sourceIndices.end() &&
            std::find(pinned.sourceIndices.begin(),pinned.sourceIndices.end(),7)!=pinned.sourceIndices.end(),"Process boundary knots remain pinned");
        o.requiredIndices.clear();o.edgeAdmissible=[](std::size_t a,std::size_t b){return !(a==3 && b==4) && b-a==1;};
        const auto blocked=downsampleTrajectorySafely(source,o);
        check(!blocked.success && blocked.trajectory.empty(),"An invalid original motion cannot publish a partial or unchecked sparse trajectory");
        bool cancelled=false;o.cancelled=[&](){return cancelled;};
        o.edgeAdmissible=[&](std::size_t,std::size_t){cancelled=true;return true;};
        const auto stopped=downsampleTrajectorySafely(source,o);
        check(stopped.cancelled && !stopped.success && stopped.trajectory.empty(),"Cancellation clears all partial downsampling output");
    }
    {
        const Eigen::Vector3d axis=Eigen::Vector3d(1,2,3).normalized();
        const double angle=80*std::acos(-1.)/180, distance=.1;
        const Eigen::Vector3d original(.3,-.2,.7),anchor=original+distance*axis;
        const auto pool=surfacePostureInitialPool(axis,false,5000,angle,947);
        double meanCos=0;bool valid=pool.size()==5000;std::set<std::array<long long,4>> distinct;
        int sectors[8]={};
        const auto u=axis.unitOrthogonal().eval(),v=axis.cross(u).eval();
        for(const auto& gene:pool) {
            const auto rotation=surfacePostureDelta(gene,axis,false,angle-1e-5);
            const Eigen::Vector3d normal=rotation*axis;
            const Eigen::Vector3d tcp=anchor-distance*normal;
            const double cosine=std::clamp(normal.dot(axis),-1.,1.);
            valid=valid && (tcp+distance*normal-anchor).norm()<1e-12 &&
                std::abs((tcp-anchor).norm()-distance)<1e-12 && std::acos(cosine)<=angle;
            meanCos+=cosine;distinct.insert(surfacePostureKey(rotation));
            double azimuth=std::atan2(normal.dot(v),normal.dot(u));if(azimuth<0)azimuth+=2*std::acos(-1.);
            ++sectors[std::min(7,int(azimuth/(2*std::acos(-1.))*8))];
        }
        check(valid && distinct.size()==5000,"All 5000 distinct candidates lie on the original surface-anchored 100mm sphere within the 80 degree normal cone");
        bool azimuths=true;for(int n:sectors)azimuths=azimuths && n>500 && n<750;
        check(azimuths && std::abs(meanCos/5000-(1+std::cos(angle-1e-5))/2)<.015,
            "Initial normals are uniform in spherical area rather than uniform in polar angle");
        check(ExternalTrajectoryEvaluationRequest{}.anchorSearchSeconds==0,
            "Default GA has no three-minute single-point deadline");
    }
    {
        GaRadiusEvolution radius(true);
        check(radius.fraction()==0 && radius.beginGeneration(1,false)==0 && radius.beginGeneration(2,false)==0,"Radius initial pool and first two offspring rounds remain nominal");
        check(radius.beginGeneration(3,false)==.3 && std::abs(GaRadiusEvolution::distance(.1,radius.clampGene(-1))-.07)<1e-12 &&
            std::abs(GaRadiusEvolution::distance(.1,radius.clampGene(1))-.13)<1e-12,"Unsafe third round unlocks precisely 70-130 mm");
        check(radius.beginGeneration(4,false)==.5,"Unsafe fourth round unlocks +/-50 percent");
        check(radius.beginGeneration(5,false)==.8 && radius.beginGeneration(8,false)==.8,"Unsafe fifth through eighth round caps radius at +/-80 percent");
        GaRadiusEvolution safe(true),disabled(false);
        safe.beginGeneration(3,false);
        check(safe.beginGeneration(4,true)==.3 && safe.beginGeneration(8,true)==.3,"Finding safety freezes radius bounds while subsequent rounds continue");
        check(disabled.beginGeneration(8,false)==0,"Disabling radius evolution retains fixed spray distance");
        const double parent=GaRadiusEvolution::distance(.1,.25);radius.beginGeneration(8,false);
        check(parent==GaRadiusEvolution::distance(.1,.25) && std::abs(parent-.12)<1e-12,"Absolute radius genes retain evaluated parent geometry when bounds expand");
    }
    {
        using Search=AdaptivePostureSearch;const double pi=std::acos(-1.);
        const auto axis=Eigen::Vector3d(1,2,3).normalized().eval();
        const Search::X x(.3,.4,.91);const auto q=Search::rotation(x,axis,pi/4,true);
        check(std::abs(std::acos(std::clamp(axis.dot(q*axis),-1.,1.))-pi/8)<1e-9,"Adaptive posture arbitrary-axis normal cone is independent of spin");
        check(Search::rotation(Search::coordinates(q,axis,pi/4,true),axis,pi/4,true).angularDistance(q)<1e-9,"Adaptive posture coordinates round-trip surface swing and axial phase");
        check((Search::repair(Search::X(.2,.3,1.01),true)-Search::X(.2,.3,-.99)).norm()<1e-10,"Adaptive posture axial phase wraps continuously");
        AdaptivePostureOptions o;o.pointEvaluations=128;o.localIterations=24;
        std::vector<Search::X> first,second;
        const auto run=[&](auto& samples){return Search::search(o,{Search::X(.6,.15,.1)},true,[&](const auto& p){samples.push_back(p);PostureConstraintFeedback f;
            const double d=(p-Search::X(.42,.15,.1)).norm();f.feasible=d<.09;f.violation=d;f.cost=d;return f;},[](){return false;});};
        const auto a=run(first),b=run(second);
        check(a.safePostures>0 && a.localEvaluations>0,"Adaptive local feasibility restoration finds a constrained basin");
        bool same=first.size()==second.size();for(std::size_t i=0;same && i<first.size();++i)same=first[i].isApprox(second[i],1e-12);
        check(same,"Adaptive search is reproducible under identical budgets and oracle");
        check(a.evaluations<=128 && first.size()==std::size_t(a.evaluations),"Adaptive search accounts every oracle evaluation within budget");
        const auto cancelled=Search::search(o,{},true,[](const auto&){return PostureConstraintFeedback{};},[](){return true;});
        check(cancelled.evaluations==0 && cancelled.safePostures==0,"Adaptive cancellation never invents a feasible result");
    }
    {
        using namespace ModelTrajPlan;using V3=Eigen::Vector3d;using V2=Eigen::Vector2d;
        ModelData tube;
        const auto add=[&](V3 a,V3 b,V3 c){for(const V3& p:{a,b,c}){tube.mainModelData_.m_indices.push_back(unsigned(tube.mainModelData_.m_vertices.size()/3));for(int j=0;j<3;++j)tube.mainModelData_.m_vertices.push_back(float(p[j]));}};
        const V2 ring[4]={{-1,-1},{1,-1},{1,1},{-1,1}};
        for(int i=0;i<4;++i){auto a=ring[i],b=ring[(i+1)%4];V3 l(a.x(),a.y(),-1),r(b.x(),b.y(),-1),u(2*a.x(),2*a.y(),1),v(2*b.x(),2*b.y(),1),m=(u+v)/2;
            add(l,r,m);add(l,m,u);add(r,v,m);}
        auto b=buildVirtualBaffle(tube);
        if(!b.enabled)std::cout<<"Baffle diagnostic: "<<b.diagnostic<<" / rims="<<b.topology.loopCount<<" / residual="<<b.maxResidual<<" / match="<<b.correspondenceRms<<std::endl;
        check(b.enabled && b.topology.loopCount==2 && b.contours.size()==1,"Two unlabeled rims with unequal sampling generate a baffle");
        check(std::abs(b.center.z())<1e-7 && b.maxResidual<1e-7 && std::abs(b.thickness-.001)<1e-12,"Midpoint fit uses metres and default 1 mm thickness");
        const auto uv=[&](const V3& p){return V2(b.axisX.dot(p-b.center),b.axisY.dot(p-b.center));};
        check(b.apertureContains(uv(V3::Zero())) && !b.apertureContains(uv(V3(2,0,0))),"Only central mesh section is permitted");
        check(!b.intersectsTriangle({V3(0,0,-.1),V3(.1,0,.1),V3(0,.1,.1)}),"Tool crossing through aperture is permitted");
        check(b.intersectsTriangle({V3(1000,0,-.1),V3(1000.1,0,.1),V3(1000,.1,.1)}),"Baffle remains infinite far beyond display patch");
        check(!b.intersectsTriangle({V3(2,0,.002),V3(3,0,.002),V3(2,1,.002)}) && b.intersectsTriangle({V3(2,0,.0004),V3(3,0,.0004),V3(2,1,.0004)}),"Physical slab thickness is checked rather than a zero-thickness plane");
        check(b.intersectsTriangle({V3(2,0,.002),V3(3,0,.002),V3(2,1,.002)},.002),"Configured clearance additionally inflates the slab");
        check(!b.displayTriangles(4).empty(),"Forbidden plane has a renderable semi-transparent mesh");
        VirtualBaffle multi;multi.enabled=true;multi.contours={{{-2,-2},{2,-2},{2,2},{-2,2}},{{-.2,-.2},{.2,-.2},{.2,.2},{-.2,.2}}};
        check(!multi.apertureContains(V2::Zero()) && multi.apertureContains(V2(1,0)),"Nested section loops preserve forbidden islands");
        check(multi.intersectsTriangle({V3(-1,-1,0),V3(1,-1,0),V3(0,1,0)}),"Forbidden island enclosed in a triangle is detected");
        check(!buildVirtualBaffle(ModelData{}).enabled,"Missing boundaries default to disabled");
        auto bad=tube;for(int i=0;i<9;++i)bad.mainModelData_.m_vertices.push_back(tube.mainModelData_.m_vertices[i]);for(int i=0;i<3;++i)bad.mainModelData_.m_indices.push_back(unsigned(tube.mainModelData_.m_vertices.size()/3+i));
        check(!buildVirtualBaffle(bad).enabled,"Nonmanifold or branching boundaries default to disabled");
        auto warped=tube;for(size_t i=0;i<warped.mainModelData_.m_vertices.size();i+=3)if(warped.mainModelData_.m_vertices[i+2]>0)warped.mainModelData_.m_vertices[i+2]+=warped.mainModelData_.m_vertices[i]*warped.mainModelData_.m_vertices[i+1]*.1f;
        const auto nonplanar=buildVirtualBaffle(warped);
        check(!nonplanar.enabled && nonplanar.maxResidual>.002,"Nonplanar midpoint fit reports residual and defaults disabled");
    }
    {
        check(pointSearchRank(false,"virtual_baffle_collision",.15)==pointSearchRank(false,"virtual_baffle_collision",.9),"Unrelated detector distance cannot improve a baffle collision score");
        check(pointSearchRank(false,"near_singular_state",.9)<pointSearchRank(true,"",0),"Singular candidates cannot outrank feasible candidates");
        check(pointSearchRank(false,"not_checked",.9)<pointSearchRank(false,"state_in_collision",0),"Unchecked states never receive measured collision fitness");
        for(int algorithm=0;algorithm<=3;++algorithm) {
            PointSearchSchedule schedule(algorithm);
            bool fixed=true;
            for(int i=0;i<2048;++i){fixed=fixed && schedule.begin()==algorithm;schedule.feedback(i%7==0);}
            check(fixed,"One algorithm stays fixed through all attempts, improvements and stalls");
        }
    }
    for(int strategy=1;strategy<=3;++strategy) {
        PointSearchPopulation search(41,PointSearchPopulation::X::Zero());
        PointSearchPopulation::X target;target<<.35,-.25,.2,.4;
        double best=100;bool bounded=true;
        for(int i=0;i<2400;++i) {
            const auto x=search.ask(strategy);
            bounded=bounded && x.allFinite() && x.head<3>().norm()<=1+1e-12 && std::abs(x[3])<=1;
            const double error=(x-target).squaredNorm();best=std::min(best,error);
            search.tell({-error,0,false});
        }
        check(bounded,"population respects rotation ball and distance bounds");
        check(best<.02,"population learns measured objective (GA/DE/CMA-ES)");
    }
    check(!PointSearchPopulation::Fitness{-1,0,false}.better({-1,0,false}),"unknown penetration does not invent improvement");
    {
        const auto axis=Eigen::Vector3d(1,2,3).normalized().eval();
        PointSearchPopulation search(1187,PointSearchPopulation::X::Zero());
        search.setPeriodicAxis(3);search.useGuidedGeneticEvolution(axis);
        PointSearchPopulation::X target=PointSearchPopulation::X::Zero();
        target.head<3>()=.83*axis.unitOrthogonal();target[3]=.99;
        auto seed=target;seed.head<3>()*=.8;seed[3]=-.96;
        const auto error=[&](const auto& x){return (x.head<3>()-target.head<3>()).squaredNorm()+std::pow(std::remainder(x[3]-target[3],2.),2);};
        search.guide(seed,{-1,error(seed),true});
        double best=error(seed);bool bounded=true;bool evolved=false;
        for(int i=0;i<192;++i){const auto x=search.ask(1);
            bounded &= std::abs(axis.dot(x.head<3>()))<1e-12 && x.head<3>().norm()<=1+1e-12 && std::abs(x[3])<=1;
            if(i==0)evolved=(x-seed).norm()>1e-8;
            best=std::min(best,error(x));search.tell({-1,error(x),true});}
        check(evolved,"Guided steady-state GA immediately evolves instead of replaying its evaluated guide");
        check(bounded,"Guided GA mutations stay in an arbitrary swing plane and wrap the axial seam");
        check(best<error(seed)*.5,"Guided GA improves measured collision merit within a single point budget");
        check(PointSearchPopulation::Fitness{1000,0,false}.better({-1,0,true}),"Zero collision merit remains inferior to independently safe candidates");
    }
    {
        const auto axis=Eigen::Vector3d(1,2,3).normalized().eval();
        const auto seeds=surfacePostureSeedGenes(axis,true);
        bool bounded=true;std::set<int> spins;
        for(const auto& x:seeds){bounded &= x.head<3>().norm()<=1+1e-12 && std::abs(x.head<3>().dot(axis))<1e-12;spins.insert((int(std::lround(x[3]*4))+8)%8);}
        check(bounded && spins.size()==8,"Surface seeds cover the full axial circle and respect an arbitrary normal cone");
        const auto fixed=surfacePostureSeedGenes(axis,false);
        check(std::all_of(fixed.begin(),fixed.end(),[](const auto& x){return x[3]==0;}),"Disabled axial rotation cannot introduce spin seeds");
        PointSearchPopulation circular(71,PointSearchPopulation::X::Zero());circular.setPeriodicAxis(3);
        double best=10.;bool wrapped=true;
        for(int i=0;i<600;++i){const auto x=circular.ask(1);const double e=x.head<3>().squaredNorm()+std::pow(std::remainder(x[3]-.99,2.),2);wrapped &= std::abs(x[3])<=1.;best=std::min(best,e);circular.tell({-e,0,false});}
        check(wrapped && best<.02,"Periodic GA learns across the plus/minus pi seam without changing bounded genes");
    }
    auto dir=std::filesystem::temp_directory_path()/std::filesystem::path("rws-external-test-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(dir);
    {
        std::ofstream out(dir/"robot.urdf");
        out<<"<robot name='external'><link name='base'/>";
        const char* axes[]={"0 0 1","0 1 0","0 1 0","1 0 0","0 1 0","1 0 0"};
        const char* offsets[]={"0 0 0.4","0.1 0 0.1","0.4 0 0","0.3 0 0","0.1 0 0","0.1 0 0"};
        for(int j=0;j<6;++j) {
            out<<"<link name='l"<<j<<"'><collision><geometry><box size='0.02 0.02 0.02'/></geometry></collision></link>"
                <<"<joint name='j"<<j<<"' type='revolute'><parent link='"<<(j==0?"base":"l"+std::to_string(j-1))
                <<"'/><child link='l"<<j<<"'/><origin xyz='"<<offsets[j]<<"'/><axis xyz='"<<axes[j]
                <<"'/><limit lower='-3.14' upper='3.14' effort='100' velocity='2'/></joint>";
        }
        out<<"</robot>";
        std::ofstream stl(dir/"part.stl");
        stl<<"solid part\n";
        const double v[4][3]={{0,0,0},{.1,0,0},{0,.1,0},{0,0,.1}};
        const int faces[4][3]={{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
        for(const auto& f:faces) {
            stl<<"facet normal 0 0 0\nouter loop\n";
            for(int i:f) stl<<"vertex "<<v[i][0]<<' '<<v[i][1]<<' '<<v[i][2]<<'\n';
            stl<<"endloop\nendfacet\n";
        }
        stl<<"endsolid part\n";
    }
    simulation_project::ProjectDocument document;
    simulation_project::RobotDesc robot; robot.id="robot"; robot.sourceType="urdf"; robot.sourcePath=(dir/"robot.urdf").generic_u8string();
    document.robots.push_back(robot);
    simulation_project::CollisionSelectionSetDesc a,b; a.id="robot_set"; b.id="part_set";
    simulation_project::CollisionSelectionSetMemberDesc member; member.robotId="robot"; a.members.push_back(member);
    member={}; member.objectId="part"; b.members.push_back(member);
    document.collision.selectionSets={a,b};
    simulation_project::CollisionDetectorDesc detector; detector.id="check"; detector.type="SelectedObjects"; detector.queryPolicy="BetweenSets";
    detector.selectionSetAId=a.id; detector.selectionSetBId=b.id; document.collision.detectors.push_back(detector);
    ExternalTrajectoryEvaluationRequest request;
    request.plan.robotId="robot"; for(int i=0;i<6;++i) request.plan.jointNames.push_back("j"+std::to_string(i));
    request.tcpLinkName="l5"; request.collisionDetectorIds={"check"};
    simulation_project::SceneObjectDesc part; part.id="part"; part.sourcePath=(dir/"part.stl").generic_u8string(); part.transform.x=10;
    request.workpiece=part;
    request.ik.model.seedJoints={.2,-.4,.5,.2,.6,-.2}; request.ik.seedCount=2;
    request.ik.maxIterations=100;
    simulation_runtime::ProjectSimulationRuntime runtime;
    const auto loaded=runtime.loadProject(document,dir);
    check(loaded.success,"Load independent six-axis URDF");
    if(loaded.success) {
        runtime.setRobotJointValues("robot",request.plan.jointNames,request.ik.model.seedJoints);
        robottrajectory::TimedCartesianPoint point;
        point.tcpPose=runtime.robot("robot")->instance->getLinkTransform("l5");
        request.plan.cartesianControlPoints.points={point,point}; request.plan.cartesianControlPoints.points[1].time=1;
        std::vector<std::string> progressEvents;
        request.returnCandidates=true;
        request.progress=[&](const std::string& value){progressEvents.push_back(value);};
        auto evaluated=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,request);
        for(const auto& d:evaluated.diagnostics) std::cout<<d.code<<": "<<d.message<<'\n';
        check(evaluated.success && evaluated.plan.trajectory.points.size()==2,"External STL and actual-model trajectory evaluation");
        auto budgetRequest=request;budgetRequest.returnCandidates=false;budgetRequest.progress={};budgetRequest.workpiece.reset();
        budgetRequest.plan=evaluated.plan;budgetRequest.plan.trajectory.points.clear();budgetRequest.plan.cartesianControlPoints.points.clear();
        for(int i=0;i<121;++i){auto control=point;control.time=i;budgetRequest.plan.cartesianControlPoints.points.push_back(control);
            auto joint=evaluated.plan.trajectory.points.front();joint.time=i;budgetRequest.plan.trajectory.points.push_back(joint);}
        auto budgetDocument=document;budgetDocument.objects.push_back(part);
        const auto softBudget=runGaApfCdfPipeline(budgetDocument,dir,budgetRequest,false);
        check(softBudget.success && softBudget.plan.trajectory.points.size()==121,"100-point target is soft: all 121 original controls remain valid instead of failing or being discarded");
        check(std::none_of(softBudget.stages.begin(),softBudget.stages.end(),[](const auto& stage){return stage.name.find("CDF accepted")!=std::string::npos;}),"Default GA/APF path does not run optional CDF/QP");
        auto storedReference=softBudget.plan;
        check(!storedReference.optimizationMetadata.empty() &&
            MotionPlanningProjectStore::upsertPlan(budgetDocument,storedReference),
            "Accepted GA/APF stores its original surface/process reference in the project");
        const auto restoredPlans=MotionPlanningProjectStore::plans(budgetDocument);
        const auto restored=std::find_if(restoredPlans.begin(),restoredPlans.end(),[&](const auto& p){return p.id==storedReference.id;});
        check(restored!=restoredPlans.end() && restored->optimizationMetadata==storedReference.optimizationMetadata,
            "Opaque GA/APF reference survives project store serialization without changing process metadata");
        auto surfaceSampler=budgetRequest;surfaceSampler.plan=storedReference;surfaceSampler.preferredOutputWaypoints=7;
        std::string referenceError;
        check(prepareGaApfDownsamplingRequest(surfaceSampler,referenceError) && surfaceSampler.sprayTargets.size()==121 &&
            surfaceSampler.sprayDistances.size()==121 && surfaceSampler.sprayDistances.front()>.099999 &&
            surfaceSampler.plan.cartesianControlPoints.points.front().tcpPose.matrix().isApprox(point.tcpPose.matrix(),1e-8),
            "Downsampling restores original source poses and exact selected distances instead of adjusted TCP anchors");
        const auto surfaceSparse=ProjectExternalTrajectoryEvaluator::evaluate(budgetDocument,dir,surfaceSampler);
        check(surfaceSparse.success && surfaceSparse.plan.trajectory.points.size()==7,
            "Persisted surface reference drives the same safe domain downsampling as the pipeline output");
        auto stale=surfaceSampler;stale.plan.trajectory.points[1].time+=.01;
        check(!prepareGaApfDownsamplingRequest(stale,referenceError),"Edited timing cannot silently reuse stale surface mapping");
        auto changedTool=surfaceSampler;changedTool.linkFromTcp.translation().x()+=.01;
        check(!prepareGaApfDownsamplingRequest(changedTool,referenceError),"Changed mounted TCP invalidates saved optimization reference");
        auto raw=budgetRequest;raw.plan.trajectory.points.clear();raw.plan.cartesianControlPoints.points.resize(2);
        raw.plan.cartesianControlPoints.points[1].time=1.;raw.ik.cachedCandidates.clear();raw.ik.preferredJoints.clear();
        const auto initialJointsBefore=budgetDocument.robots.front().initialJoints.size();
        const auto fromRaw=runGaApfCdfPipeline(budgetDocument,dir,raw,false);
        check(fromRaw.success && std::any_of(fromRaw.diagnostics.begin(),fromRaw.diagnostics.end(),
            [](const auto& d){return d.code=="pipeline_initial_top1";}) &&
            budgetDocument.robots.front().initialJoints.size()==initialJointsBefore,
            "Raw Cartesian pipeline performs physical all-IK and Top-1 initialization without mutating the caller project");
        auto sparseRequest=budgetRequest;sparseRequest.plan=softBudget.plan;
        sparseRequest.pipelineStage="downsample";sparseRequest.preferredOutputWaypoints=7;
        sparseRequest.verificationTrajectory=softBudget.plan.trajectory;
        const auto sparseResult=ProjectExternalTrajectoryEvaluator::evaluate(budgetDocument,dir,sparseRequest);
        check(sparseResult.success && sparseResult.plan.trajectory.points.size()==7 &&
            sparseResult.plan.trajectory.points.front().q==softBudget.plan.trajectory.points.front().q &&
            sparseResult.plan.trajectory.points.back().q==softBudget.plan.trajectory.points.back().q,
            "Domain downsampling reaches the target under actual robot and detector checks");
        sparseRequest.preferredOutputWaypoints=1;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(budgetDocument,dir,sparseRequest).success,
            "Domain downsampling rejects a target that would discard trajectory boundaries");
        // The unchanged joint-linear source traces a curved TCP arc. Its own
        // arc must not fail a corridor built from Cartesian endpoint chords.
        auto curved=sparseRequest;curved.preferredOutputWaypoints=2;
        curved.maximumTcpPositionDeviation=1e-5;curved.maximumTcpOrientationDeviation=1e-6;
        curved.plan.trajectory.points.clear();curved.plan.cartesianControlPoints.points.clear();
        for(int i=0;i<3;++i) {
            auto q=request.ik.model.seedJoints;q[0]+=.2*i;
            runtime.setRobotJointValues("robot",request.plan.jointNames,q);
            auto control=point;control.time=i;control.tcpPose=runtime.robot("robot")->instance->getLinkTransform("l5");
            curved.plan.cartesianControlPoints.points.push_back(control);
            auto joint=evaluated.plan.trajectory.points.front();joint.time=i;joint.q=q;
            curved.plan.trajectory.points.push_back(joint);
        }
        curved.verificationTrajectory=curved.plan.trajectory;
        const auto curvedResult=ProjectExternalTrajectoryEvaluator::evaluate(budgetDocument,dir,curved);
        check(curvedResult.success && curvedResult.plan.trajectory.points.size()==2 &&
            curvedResult.plan.trajectory.points.front().q==curved.plan.trajectory.points.front().q &&
            curvedResult.plan.trajectory.points.back().q==curved.plan.trajectory.points.back().q,
            "Downsampling uses actual source joint-linear FK rather than rejecting its curved TCP arc against endpoint chords");
        runtime.setRobotJointValues("robot",request.plan.jointNames,request.ik.model.seedJoints);
        auto adaptive=request;adaptive.pipelineStage="posture";adaptive.controlPointsOnly=true;adaptive.returnCandidates=true;
        adaptive.postureOptions.pointEvaluations=24;adaptive.postureOptions.graphRefinementRounds=0;
        const auto adapted=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,adaptive);
        check(adapted.success && adapted.plan.trajectory.points.size()==2,"Adaptive stage integrates real-model candidates and layered graph without APF");
        check(!adapted.candidateLayers.empty() && adapted.candidateLayers.size()==2,"Adaptive stage exports original-point candidate layers");
        adaptive.postureOptions.pointEvaluations=0;
        const auto badAdaptive=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,adaptive);
        check(!badAdaptive.success && badAdaptive.diagnostics.front().code=="invalid_posture_parameters","Adaptive stage rejects disabled evaluation budgets");
        check(document.objects.empty(),"Private workpiece import does not mutate input document");
        auto staged=request;staged.pipelineStage="apf";staged.optimizeAvoidance=true;
        staged.optimizationSeed=evaluated.plan.trajectory;staged.cdfOptions.postSmoothingIterations=0;
        std::vector<std::string> stageProgress;
        staged.progress=[&](const std::string& s){stageProgress.push_back(s);};
        const auto apfOnly=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,staged);
        for(const auto& d:apfOnly.diagnostics)std::cout<<"STAGED "<<d.code<<": "<<d.message<<'\n';
        check(apfOnly.success && apfOnly.plan.trajectory.points.size()==2,"APF-only preserves source knot count when densification is disabled");
        check(std::none_of(stageProgress.begin(),stageProgress.end(),[](const auto& s){return s.find("CDF/QP iteration")!=std::string::npos || s.find("Solving the trajectory QP")!=std::string::npos;}),
            "APF-only never starts CDF or OSQP");
        auto tooFast=request;tooFast.pipelineStage="apf";tooFast.optimizeAvoidance=true;
        auto shifted=request.ik.model.seedJoints;shifted[0]+=.02;
        runtime.setRobotJointValues("robot",request.plan.jointNames,shifted);
        tooFast.plan.cartesianControlPoints.points[1].tcpPose=runtime.robot("robot")->instance->getLinkTransform("l5");
        tooFast.plan.cartesianControlPoints.points[1].time=.0001;
        tooFast.ik.cachedCandidates={{request.ik.model.seedJoints},{shifted}};
        const auto timingOnly=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,tooFast);
        double requiredDuration=0;
        for(const auto& d:timingOnly.diagnostics)if(d.code=="apf_velocity_retime_required")
            requiredDuration=nlohmann::json::parse(d.message)["interval_seconds"][0].get<double>();
        check(!timingOnly.success && requiredDuration>.01,"APF timing diagnosis never certifies an over-speed seed");
        if(requiredDuration>0) {
            tooFast.plan.cartesianControlPoints.points[1].time=requiredDuration;
            check(ProjectExternalTrajectoryEvaluator::evaluate(document,dir,tooFast).success,
                "Retimed exact poses pass fresh APF checks under original velocity limits");
        }
        runtime.setRobotJointValues("robot",request.plan.jointNames,request.ik.model.seedJoints);
        staged.pipelineStage="cdf";staged.optimizationSeed=apfOnly.plan.trajectory;stageProgress.clear();
        const auto cdfOnly=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,staged);
        check(cdfOnly.success,"CDF-only accepts exact safe APF output after its own full-path gate");
        check(std::any_of(stageProgress.begin(),stageProgress.end(),[](const auto& s){return s=="Solving the trajectory QP";}),
            "CDF-only actually invokes OSQP rather than publishing its input as optimized");
        staged.workpiece->transform.x=point.tcpPose.translation().x();
        staged.workpiece->transform.y=point.tcpPose.translation().y();
        staged.workpiece->transform.z=point.tcpPose.translation().z();stageProgress.clear();
        const auto unsafeCdf=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,staged);
        check(!unsafeCdf.success && std::any_of(unsafeCdf.diagnostics.begin(),unsafeCdf.diagnostics.end(),[](const auto& d){return d.code=="cdf_input_not_collision_free";}),
            "CDF-only rejects colliding input without invoking APF or QP");
        check(std::none_of(stageProgress.begin(),stageProgress.end(),[](const auto& s){return s.find("CDF/QP iteration")!=std::string::npos;}),"Unsafe CDF input never reaches optimization");

        auto boundary=request;boundary.startJoints=request.ik.model.seedJoints;
        const auto pinned=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,boundary);
        check(pinned.success && pinned.plan.trajectory.points.front().q==boundary.startJoints,"Next scan starts at the exact prior endpoint joints");
        boundary.startJoints[0]+=.5;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,boundary).success,"Mismatched shared endpoint cannot switch inverse-kinematics branch");

        auto surfaceRequest=request;surfaceRequest.optimizeAvoidance=true;surfaceRequest.sprayDistances={.1,.1};
        surfaceRequest.sprayTargets={point.tcpPose.translation()+.1*point.tcpPose.linear()*surfaceRequest.sprayAxis,
            point.tcpPose.translation()+.1*point.tcpPose.linear()*surfaceRequest.sprayAxis};
        const auto surfaceMapped=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,surfaceRequest);
        const auto mapping=surfaceMapped.surfaceMapping.empty()?nlohmann::json::object():nlohmann::json::parse(surfaceMapped.surfaceMapping);
        bool mapped=surfaceMapped.success && mapping.contains("samples") && mapping["samples"].size()>2;
        if(mapping.contains("samples"))for(const auto& sample:mapping["samples"])mapped=mapped && sample["mapping_error_m"].get<double>()<1e-6;
        check(mapped && std::any_of(surfaceMapped.diagnostics.begin(),surfaceMapped.diagnostics.end(),[](const auto& d){return d.code=="avoidance_optimized";}),
            "Whole-path native CDF preserves ordered surface anchors through projection/IK and records mapping evidence");

        auto axial=surfaceRequest;axial.optimizeAvoidance=false;axial.pipelineStage="ik";
        axial.controlPointsOnly=true;axial.independentPointsOnly=true;axial.returnCandidates=true;
        axial.pointSearchReferenceDelta.assign(2,Eigen::Quaterniond(Eigen::AngleAxisd(1.57,axial.sprayAxis)));
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,axial).success,
            "Legacy full-posture limit still rejects 90 degree axial deviation");
        axial.allowSprayAxisRotation=true;
        check(ProjectExternalTrajectoryEvaluator::evaluate(document,dir,axial).success,
            "Authorized axial turn does not consume the 30 degree spray-normal cone");
        axial.pointSearchReferenceDelta.assign(2,Eigen::Quaterniond(Eigen::AngleAxisd(.56,axial.sprayAxis.unitOrthogonal())));
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,axial).success,
            "Free axial turn cannot permit spray-normal tilt beyond 30 degrees");

        auto anchorRequest=surfaceRequest;anchorRequest.pipelineStage="ga";
        anchorRequest.returnCandidates=false; // Regression: GA rescue must initialize its internal arrays regardless.
        anchorRequest.optimizeAvoidance=false;anchorRequest.controlPointsOnly=true;anchorRequest.gaInitialPostures=128;anchorRequest.gaGenerations=8;anchorRequest.gaPointEvaluations=320;anchorRequest.gaFieldEvaluations=8;
        anchorRequest.cdfOptions.postSmoothingIterations=0;
        anchorRequest.workpiece->transform.x=point.tcpPose.translation().x();
        anchorRequest.workpiece->transform.y=point.tcpPose.translation().y();
        anchorRequest.workpiece->transform.z=point.tcpPose.translation().z();
        anchorRequest.sprayDistances={.3,.3};
        for(auto& target:anchorRequest.sprayTargets)target=point.tcpPose.translation()+.3*point.tcpPose.linear()*anchorRequest.sprayAxis;
        // These cases verify evaluation-count schedules, independently of
        // machine speed and Debug instrumentation. Deadline behavior is tested below.
        anchorRequest.stageTimeBudgetSeconds=0;
        anchorRequest.anchorSearchSeconds=0;
        auto wrongGa=anchorRequest;wrongGa.controlPointsOnly=false;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,wrongGa).success,
            "Direct GA API rejects implicit continuous planning flags");
        const auto anchored=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,anchorRequest);
        bool safeEndpoint=false,mappedEndpoint=true;
        for(const auto& text:anchored.pointSearchReports) {
            const auto item=nlohmann::json::parse(text);
            if(item.value("strategy",std::string())!="full_pool_genetic" || !item.value("solved",false))continue;
            safeEndpoint=true;
            check(item.value("initial_pool_completed",0)==128 && item.value("initial_postures_evaluated",0)==128 &&
                item.value("generations_completed",0)==8 && item.value("genetic_offspring",0)==192 && item.value("schedule_complete",false) && item.value("nominal_generation_budget_preserved",false),
                "GA fully evaluates distinct initial poses then all eight generations even after finding safety");
        }
        if(!anchored.surfaceMapping.empty()) {
            const auto anchorMapping=nlohmann::json::parse(anchored.surfaceMapping);
            for(const auto& sample:anchorMapping["samples"])mappedEndpoint &= sample["mapping_error_m"].get<double>()<1e-6;
        }
        check(safeEndpoint && mappedEndpoint,"GA repairs colliding source points with fixed-surface mapping and no appended points");
        check(anchored.success,"GA repairs colliding original points before APF; interval certificate is not claimed");
        auto preserved=surfaceRequest;preserved.pipelineStage="ga";preserved.controlPointsOnly=true;
        preserved.optimizeAvoidance=false;preserved.gaInitialPostures=24;preserved.gaGenerations=1;
        const auto preservedResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,preserved);
        check(preservedResult.success && std::none_of(preservedResult.pointSearchReports.begin(),preservedResult.pointSearchReports.end(),
            [](const auto& text){return text.find("full_pool_genetic")!=std::string::npos;}),
            "Already safe controls retain valid candidates without entering the 5000-pose repair schedule");
        auto preference=preserved;preference.gaEvaluateAllPoints=true;preference.returnCandidates=true;
        preference.gaPointEvaluations=256;preference.gaGenerations=2;
        const auto preferred=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,preference);
        bool keptOriginal=preferred.success;
        if(preferred.success)for(std::size_t i=0;i<preferred.plan.trajectory.points.size();++i) {
            runtime.setRobotJointValues("robot",request.plan.jointNames,preferred.plan.trajectory.points[i].q);
            const auto tcp=runtime.robot("robot")->instance->getLinkTransform("l5");
            const auto& original=preference.plan.cartesianControlPoints.points[i].tcpPose;
            keptOriginal &= (tcp.translation()-original.translation()).norm()<preference.ik.positionTolerance*2 &&
                Eigen::Quaterniond(tcp.linear()).angularDistance(Eigen::Quaterniond(original.linear()))<preference.ik.orientationTolerance*2;
        }
        check(keptOriginal && std::any_of(preferred.diagnostics.begin(),preferred.diagnostics.end(),
            [](const auto& d){return d.code=="ga_original_pose_preference";}),
            "Final GA graph keeps safe original poses even when full search adds adjusted safe alternatives");
        // A physical obstacle intersects a joint chord while both endpoints are safe.
        // Exhaust a deliberately tiny APF effort to verify typed source-clock feedback.
        auto feedbackDocument=document;auto midpointPart=part;
        midpointPart.transform.x=point.tcpPose.translation().x()-.02;
        midpointPart.transform.y=point.tcpPose.translation().y()-.02;
        midpointPart.transform.z=point.tcpPose.translation().z()-.02;
        feedbackDocument.objects.push_back(midpointPart);
        robottrajectory::JointTrajectory crossed;
        for(int sign:{-1,1}) {
            robottrajectory::TimedJointPoint p;p.time=sign<0?3.:7.;p.q=request.ik.model.seedJoints;
            p.q[0]+=.2*sign;crossed.points.push_back(p);
        }
        ProjectCdfQpRepairOptions apfEffort;apfEffort.detectorIds={"check"};
        apfEffort.stage=ProjectCdfQpRepairOptions::Stage::ApfOnly;apfEffort.safetyMargin=0;
        apfEffort.targetClearance=0;apfEffort.apfIterationBudget=1;apfEffort.apfStepSize=.00001;
        apfEffort.apfMaxTcpDeviation=0;apfEffort.densifyKnots=false;apfEffort.validationMaxJointStep=.005;
        const auto failedConnection=ProjectCdfQpTrajectoryRepairService{}.repair(feedbackDocument,dir,"robot",request.plan.jointNames,crossed,apfEffort);
        check(!failedConnection.success && failedConnection.apfConnectionFailures.size()==1 &&
            failedConnection.apfConnectionFailures.front().beginTime==3. && failedConnection.apfConnectionFailures.front().endTime==7. &&
            std::any_of(failedConnection.diagnostics.begin(),failedConnection.diagnostics.end(),[](const auto& d){return d.code=="apf_segment_failed";}),
            "Exhausted physical APF connection returns a typed failed interval in the original seed clock");
        // Real-model feedback: both ends of the failed pair are individually safe.
        // The new GA must change the pair without moving controls outside the window.
        auto feedback=preserved;feedback.gaInitialPostures=128;feedback.gaGenerations=2;
        feedback.gaPointEvaluations=320;feedback.pointSearchSequence=1;
        feedback.plan.cartesianControlPoints.points.clear();feedback.optimizationSeed.points.clear();
        feedback.sprayTargets.clear();feedback.sprayDistances.clear();
        for(int i=0;i<4;++i) {
            auto control=point;control.time=i;
            feedback.plan.cartesianControlPoints.points.push_back(control);
            auto joint=preservedResult.plan.trajectory.points.front();joint.time=i;
            feedback.optimizationSeed.points.push_back(joint);
            feedback.sprayTargets.push_back(point.tcpPose.translation()+.1*point.tcpPose.linear()*feedback.sprayAxis);
            feedback.sprayDistances.push_back(.1);
        }
        feedback.gaResampleControls={1,2};
        feedback.gaRejectedConnections={{1,feedback.optimizationSeed.points[1].q,feedback.optimizationSeed.points[2].q}};
        const auto renewed=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,feedback);
        bool changedPair=false,feedbackAnchors=renewed.success;
        if(renewed.success)for(std::size_t i=0;i<4;++i) {
            const auto& q=renewed.plan.trajectory.points[i].q;
            if(i==1 || i==2)for(std::size_t j=0;j<q.size();++j)
                changedPair |= std::abs(q[j]-feedback.optimizationSeed.points[i].q[j])>=1e-4;
            runtime.setRobotJointValues("robot",request.plan.jointNames,q);
            const auto tcp=runtime.robot("robot")->instance->getLinkTransform("l5");
            feedbackAnchors &= (tcp.translation()+.1*tcp.linear()*feedback.sprayAxis-feedback.sprayTargets[i]).norm()<.001;
        }
        check(renewed.success && changedPair && renewed.plan.trajectory.points.front().q==feedback.optimizationSeed.points.front().q &&
            renewed.plan.trajectory.points.back().q==feedback.optimizationSeed.points.back().q,
            "APF feedback resamples safe GA parents, rejects the failed encoder pair, and freezes all controls outside the window");
        check(feedbackAnchors && renewed.pointSearchReports.size()==2 && std::all_of(renewed.pointSearchReports.begin(),renewed.pointSearchReports.end(),
            [](const auto& text){const auto r=nlohmann::json::parse(text);return r.value("apf_feedback",false) &&
                r.value("feedback_sequence",0)==1 && r.value("initial_pool_completed",0)==128 && r.value("generations_completed",0)==2 && r.value("feedback_local_offspring",0)==48;}),
            "Feedback keeps original surface anchors and runs a fresh complete initial pool and offspring schedule only at requested controls");
        auto malformedFeedback=feedback;malformedFeedback.optimizationSeed.points[2].time+=.01;
        const auto malformedResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,malformedFeedback);
        check(!malformedResult.success && std::any_of(malformedResult.diagnostics.begin(),malformedResult.diagnostics.end(),
            [](const auto& d){return d.code=="invalid_ga_feedback";}),"Feedback cannot remap a seed onto different source control stations");
        auto lockedFeedback=feedback;lockedFeedback.startJoints=feedback.optimizationSeed.points.front().q;
        lockedFeedback.gaResampleControls.push_back(0);
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,lockedFeedback).success,
            "Feedback cannot mutate an explicitly locked boundary");
        bool cancelFeedback=false;auto stoppedFeedback=feedback;
        stoppedFeedback.cancelled=[&]{return cancelFeedback;};
        stoppedFeedback.progress=[&](const std::string& event){if(event.find("genetic_initial_pool")!=std::string::npos)cancelFeedback=true;};
        const auto feedbackCancelled=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,stoppedFeedback);
        check(cancelFeedback && feedbackCancelled.cancelled && !feedbackCancelled.success,"APF feedback remains cancellable during resampling");
        auto cancelRequest=anchorRequest;bool cancelGa=false;
        cancelRequest.cancelled=[&]{return cancelGa;};
        cancelRequest.progress=[&](const std::string& event) {
            if(event.find("genetic_initial_pool")!=std::string::npos)cancelGa=true;
        };
        const auto cancelledGa=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,cancelRequest);
        check(cancelGa && cancelledGa.cancelled && !cancelledGa.success,
            "Unlimited single-point GA remains cancellable after initial candidate evaluation and cannot publish success");
        auto shell=anchorRequest;shell.gaRadiusEvolutionEnabled=true;shell.gaNominalSprayDistances=shell.sprayDistances;
        // Exact full-schedule budget: 24 initial + 5*24 nominal + 3*24 radius.
        // Rescue is covered separately; impossible-clearance rescue adds no evidence here.
        shell.minimumClearance=100.;shell.gaInitialPostures=24;shell.gaPointEvaluations=216;
        shell.gaGenerations=5;shell.gaFeasibilityBatchSize=16;shell.anchorSearchSeconds=0;
        const auto shellResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,shell);
        bool shellSchedule=false;
        for(const auto& text:shellResult.pointSearchReports) {
            const auto r=nlohmann::json::parse(text);
            if(r.value("strategy",std::string())!="full_pool_genetic")continue;
            const auto rounds=r.at("radius_generations");
            shellSchedule=rounds.size()==5 && rounds[0]["fraction"]==0 && rounds[1]["fraction"]==0 &&
                rounds[2]["fraction"]==.3 && rounds[3]["fraction"]==.5 && rounds[4]["fraction"]==.8 &&
                r.value("nominal_generation_budget_preserved",false) && r.value("genetic_offspring",0)==120 &&
                r.value("radius_offspring",0)==72 && r.value("single_point_time_limit_seconds",-1.)==0;
        }
        check(!shellResult.success && shellSchedule,
            "Real IK/collision GA preserves all nominal offspring and adds radius offspring only after unsafe generations 3/4/5; impossible clearance cannot publish safety");
        auto boundedRescue=anchorRequest;
        boundedRescue.minimumClearance=100.;boundedRescue.gaPointEvaluations=208;
        boundedRescue.anchorSearchSeconds=900.;boundedRescue.stageTimeBudgetSeconds=0.;
        const auto rescued=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,boundedRescue);
        bool rescueBudget=false;
        for(const auto& text:rescued.pointSearchReports){const auto r=nlohmann::json::parse(text);
            if(r.value("strategy",std::string())!="full_pool_genetic")continue;
            rescueBudget=r.value("initial_pool_completed",0)==128 && r.value("attempts",0)<=208 &&
                !r.value("schedule_complete",true) && r.value("stop_reason",std::string())=="point_evaluation_budget";}
        check(rescueBudget,"Insufficient full-schedule budget is explicit and cannot claim completed generations");
        check(!rescued.success,"Enhanced GA merit cannot bypass an impossible actual clearance requirement");
        check(!anchored.success || anchored.plan.trajectory.points.size()==2,"Endpoint repair cannot drop or append original knots");
        if(anchored.success) {
            auto nextScan=anchorRequest;nextScan.startJoints=anchored.plan.trajectory.points.back().q;
            const auto continued=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,nextScan);
            check(continued.success && continued.plan.trajectory.points.front().q==nextScan.startJoints,
                "Surface-adjusted shared scan boundary keeps exact preceding joints and original process bounds");
        }
        anchorRequest.pointSearchMaxAngleDeg=.001;anchorRequest.anchorSearchAttempts=1;
        const auto blockedAnchor=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,anchorRequest);
        check(!blockedAnchor.success && std::any_of(blockedAnchor.diagnostics.begin(),blockedAnchor.diagnostics.end(),[](const auto& d){return d.code=="ga_point_unresolved";}),
            "Blocked endpoint stops before APF/CDF and does not fabricate a safe anchor");
        anchorRequest.stageTimeBudgetSeconds=.000001;
        const auto budgetStop=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,anchorRequest);
        check(!budgetStop.success && !budgetStop.cancelled && std::any_of(budgetStop.diagnostics.begin(),budgetStop.diagnostics.end(),[](const auto& d){return d.code=="stage_budget_exhausted";}),
            "Stage budget is bounded failure, distinct from user cancellation");
        anchorRequest.anchorSearchAttempts=513;
        const auto invalidAnchor=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,anchorRequest);
        check(!invalidAnchor.success,"Malformed anchor budget fails closed");


        auto adjustedSurface=surfaceRequest;adjustedSurface.optimizeAvoidance=false;
        for(auto& p:adjustedSurface.plan.cartesianControlPoints.points)
            p.tcpPose.linear()=p.tcpPose.linear()*Eigen::AngleAxisd(.05,Eigen::Vector3d::UnitZ()).toRotationMatrix();
        const auto adjustedSolution=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,adjustedSurface);
        auto stagedReplay=surfaceRequest;stagedReplay.optimizeAvoidance=false;stagedReplay.pipelineStage="validate";
        stagedReplay.verificationTrajectory=adjustedSolution.plan.trajectory;
        const auto processReplay=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,stagedReplay);
        check(adjustedSolution.success && processReplay.success,"Exact staged replay accepts surface-fixed adjusted orientation instead of requiring the old TCP pose");
        auto lockedReplay=stagedReplay;
        lockedReplay.gaNominalSprayDistances={.1,.1};lockedReplay.sprayDistances={.13,.13};
        lockedReplay.pointSearchMinDistances=lockedReplay.sprayDistances;
        lockedReplay.pointSearchMaxDistances=lockedReplay.sprayDistances;
        for(std::size_t i=0;i<lockedReplay.sprayTargets.size();++i) {
            const Eigen::Vector3d shift=.03*lockedReplay.plan.cartesianControlPoints.points[i].tcpPose.linear()*lockedReplay.sprayAxis;
            lockedReplay.plan.cartesianControlPoints.points[i].tcpPose.translation()+=shift;
            lockedReplay.sprayTargets[i]+=shift;
        }
        check(ProjectExternalTrajectoryEvaluator::evaluate(document,dir,lockedReplay).success,
            "Fixed selected 130 mm radius replays exact joints against immutable 100 mm source anchors");
        auto driftedReplay=lockedReplay;
        driftedReplay.sprayDistances={.12,.12};driftedReplay.pointSearchMinDistances=driftedReplay.sprayDistances;
        driftedReplay.pointSearchMaxDistances=driftedReplay.sprayDistances;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,driftedReplay).success,
            "Independent replay rejects a 10 mm drift from the locked GA-selected radius");
        auto speedRequest=request;speedRequest.linkFromTcp.translation()=Eigen::Vector3d(.2,.03,.1);
        speedRequest.maximumTcpPositionDeviation=.01;speedRequest.maximumTcpOrientationDeviation=.02;
        speedRequest.jacobianLengthScale=3.;
        runtime.setRobotJointValues("robot",request.plan.jointNames,request.ik.model.seedJoints);
        auto t0=runtime.robot("robot")->instance->getLinkTransform("l5")*speedRequest.linkFromTcp;
        auto q1=request.ik.model.seedJoints;q1[0]+=.002;
        runtime.setRobotJointValues("robot",request.plan.jointNames,q1);
        auto t1=runtime.robot("robot")->instance->getLinkTransform("l5")*speedRequest.linkFromTcp;
        speedRequest.plan.cartesianControlPoints.points[0].tcpPose=t0;
        speedRequest.plan.cartesianControlPoints.points[1].tcpPose=t1;
        const auto measured=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,speedRequest);
        auto velocity=measured.velocityTelemetry.empty()?nlohmann::json::object():nlohmann::json::parse(measured.velocityTelemetry);
        check(measured.success && velocity.contains("samples") && !velocity["samples"].empty(),"Nonzero calibrated TCP path returns velocity telemetry");
        if(velocity.contains("samples") && !velocity["samples"].empty()) {
            const auto& s=velocity["samples"][0];double residual=0;
            for(int i=0;i<6;++i){double sum=0;for(int j=0;j<6;++j)sum+=s["jacobian_world_tcp"][i][j].get<double>()*s["qdot_rad_s"][j].get<double>();residual+=std::abs(sum-s["tcp_twist"][i].get<double>());}
            check(residual<1e-12 && s["jacobian_fk_residual"].get<double>()<1e-6 && s["tcp_speed_m_s"].get<double>()>1e-5,"Physical J*qdot agrees with independent FK velocity, including TCP offset; singularity scaling is excluded");
        }
        runtime.setRobotJointValues("robot",request.plan.jointNames,request.ik.model.seedJoints);

        check(std::any_of(progressEvents.begin(),progressEvents.end(),[](const auto& e){
            return e.find("\"type\":\"point_validation\"")!=std::string::npos && e.find("\"passed\":2")!=std::string::npos;
        }),"Live point progress counts feasible points rather than IK root count");
        auto batch=request; batch.controlPointsOnly=true;batch.independentPointsOnly=true;
        batch.plan.cartesianControlPoints.points.resize(1);
        const auto singleBatch=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,batch);
        check(singleBatch.success && singleBatch.plan.trajectory.points.empty() && singleBatch.continuousPaths.empty(),
            "Single independent point validates without claiming a connected path");
        batch.plan.cartesianControlPoints.points.push_back(batch.plan.cartesianControlPoints.points.front());
        batch.plan.cartesianControlPoints.points.back().time=1;
        batch.plan.cartesianControlPoints.points.back().tcpPose.translation().x()=1000;
        const auto partialBatch=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,batch);
        check(!partialBatch.success && partialBatch.candidateRejections.size()==2 &&
            std::any_of(partialBatch.candidateRejections.front().begin(),partialBatch.candidateRejections.front().end(),
                [](const auto& reason){return reason.empty();}),
            "Missing IK at one point does not skip collision checks on other batch points");
        batch.controlPointsOnly=false;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,batch).success,
            "Independent point mode cannot claim interpolation validation");
        auto cachedRequest=request; cachedRequest.returnCandidates=true;
        const auto firstCache=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,cachedRequest);
        for(const auto& layer:firstCache.candidateLayers) {
            std::vector<std::vector<double>> roots;
            for(const auto& c:layer.candidates) roots.push_back(c.joints);
            cachedRequest.ik.cachedCandidates.push_back(std::move(roots));
        }
        const auto cached=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,cachedRequest);
        check(cached.success && cached.candidateLayers.front().message.find("Reused")!=std::string::npos,
            "Unchanged actual-model roots reuse IK but still validate path");
        auto continuation=request;
        continuation.ik.preferredJoints={request.ik.model.seedJoints,request.ik.model.seedJoints};
        const auto continued=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,continuation);
        check(continued.success && continued.candidateLayers.front().message.find("Continued")==0 &&
            continued.candidateLayers.front().candidates.size()==1,
            "Preferred chain continues locally with one root and full native validation");
        continuation.ik.preferredJoints[0][0]=1e20;
        const auto fallback=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,continuation);
        check(fallback.success && fallback.candidateLayers.front().message.find("Continued")!=0,
            "Invalid continuation seed falls back to bounded multistart search");
        continuation=request; continuation.ik.preferredJoints={request.ik.model.seedJoints,request.ik.model.seedJoints};
        continuation.minimumSingularValue=1e6;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,continuation).success,
            "Local continuation cannot bypass singularity validation");
        cachedRequest.ik.cachedCandidates[0][0][0]+=.5;
        const auto staleCache=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,cachedRequest);
        check(staleCache.success && staleCache.candidateLayers.front().message.find("Reused")==std::string::npos,
            "Stale cached root is rejected by actual FK and freshly solved");
        cachedRequest.minimumSingularValue=1e6;
        progressEvents.clear();
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,cachedRequest).success,
            "Cached roots cannot bypass fresh singularity validation");
        check(std::any_of(progressEvents.begin(),progressEvents.end(),[](const auto& e){
            return e.find("\"type\":\"point_validation\"")!=std::string::npos && e.find("\"passed\":0")!=std::string::npos;
        }),"IK roots failing singularity checks are not counted as passing points");
        auto relative=request; relative.frame=ExternalTrajectoryFrame::Workpiece; relative.workpieceId="part";
        for(auto& p:relative.plan.cartesianControlPoints.points) p.tcpPose.translation().x()-=10;
        check(ProjectExternalTrajectoryEvaluator::evaluate(document,dir,relative).success,"Workpiece frame transforms targets into world");
        WorkpiecePoseSearchOptions search; search.enabled=true; search.maxCandidates=3; search.requireYVertical=false;
        search.lowerPosition=search.upperPosition=Eigen::Vector3d(10,0,0);
        search.lowerRpy=search.upperRpy=Eigen::Vector3d::Zero();
        auto searchRequest=relative; searchRequest.workpiece->transform.x=11;
        const auto searched=ProjectExternalWorkpiecePlanner::plan(document,dir,searchRequest,search);
        check(searched.evaluation.success && searched.selectedWorkpiecePose && searched.selectedWorkpiecePose->x==10 &&
            searched.rejectedByConstraints==1,"Hard pose bounds reject original pose and search finds a validated replacement");
        auto impossible=relative; impossible.minimumSingularValue=1e6;
        const auto exhausted=ProjectExternalWorkpiecePlanner::plan(document,dir,impossible,search);
        check(!exhausted.evaluation.success && exhausted.attempts.size()==3 && exhausted.evaluation.retryWorkpiecePose,
            "Failed path validation feeds back into subsequent bounded pose candidates");
        search.requireYVertical=true;
        search.lowerRpy=Eigen::Vector3d(-3.14,-3.14,-3.14); search.upperRpy=-search.lowerRpy;
        simulation_project::TransformDesc vertical; vertical.x=10; vertical.roll=1.5707963267948966;
        check(ProjectExternalWorkpiecePlanner::poseAllowed(vertical,search),"UF2 Y axis vertical hard constraint accepts aligned pose");
        vertical.roll=0;
        check(!ProjectExternalWorkpiecePlanner::poseAllowed(vertical,search),"UF2 Y axis hard constraint rejects unaligned pose");
        auto optimizedRequest=request; optimizedRequest.optimizeAvoidance=true; optimizedRequest.optimizationIterations=1;
        const auto optimized=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,optimizedRequest);
        for(const auto& d:optimized.diagnostics) std::cout<<"OPT "<<d.code<<": "<<d.message<<'\n';
        check(optimized.success,"Generic avoidance mode publishes only an independently validated path");
        auto replay=request; replay.verificationTrajectory=optimized.plan.trajectory;
        const auto replayed=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,replay);
        check(replayed.success,"Exact returned joint trajectory passes independent fresh-scene replay");
        replay.verificationTrajectory.points.front().q[0]+=.8;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,replay).success,
            "Independent replay rejects tampered joints rather than finding replacement IK");
        replay=request; replay.verificationTrajectory=optimized.plan.trajectory;
        replay.verificationTrajectory.points.back().time+=1;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,replay).success,"Replay cannot omit original time coverage");
        auto multiple=document;
        auto extraDetector=multiple.collision.detectors.front(); extraDetector.id="check_second";
        multiple.collision.detectors.push_back(extraDetector);
        optimizedRequest.collisionDetectorIds.push_back("check_second");
        const auto multipleResult=ProjectExternalTrajectoryEvaluator::evaluate(multiple,dir,optimizedRequest);
        check(multipleResult.success && multiple.robots.size()==1,"Generic repair retains multiple selected detectors without inserting a legacy obstacle");
        auto blocked=request;
        auto middleJoints=request.ik.model.seedJoints; middleJoints[0]=.4;
        runtime.setRobotJointValues("robot",request.plan.jointNames,middleJoints);
        const auto middlePose=runtime.robot("robot")->instance->getLinkTransform("l5");
        auto endJoints=request.ik.model.seedJoints; endJoints[0]=.6;
        runtime.setRobotJointValues("robot",request.plan.jointNames,endJoints);
        blocked.plan.cartesianControlPoints.points[1].tcpPose=runtime.robot("robot")->instance->getLinkTransform("l5");
        blocked.plan.cartesianControlPoints.points[1].time=4;
        blocked.workpiece->transform.x=middlePose.translation().x()-.025;
        blocked.workpiece->transform.y=middlePose.translation().y()-.025;
        blocked.workpiece->transform.z=middlePose.translation().z()-.025;
        blocked.maximumTcpPositionDeviation=.15; blocked.maximumTcpOrientationDeviation=.3;
        for(double q:request.ik.model.seedJoints) {blocked.ik.lower.push_back(q-.1); blocked.ik.upper.push_back(q+.1);}
        blocked.ik.upper[0]=.61; blocked.ik.lower[0]=.19; blocked.ik.seedCount=1;
        const auto unrepaired=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,blocked);
        check(!unrepaired.success,"Endpoint-solvable constrained IK branch can still have a colliding segment");
        auto sparseBlocked=blocked;sparseBlocked.pipelineStage="downsample";sparseBlocked.returnCandidates=false;
        sparseBlocked.preferredOutputWaypoints=2;
        auto firstJoint=evaluated.plan.trajectory.points.front();firstJoint.time=0;
        auto lastJoint=firstJoint;lastJoint.q=endJoints;lastJoint.time=4;
        sparseBlocked.verificationTrajectory.points={firstJoint,lastJoint};
        const auto sparseCollision=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,sparseBlocked);
        check(!sparseCollision.success && sparseCollision.plan.trajectory.empty(),
            "Domain downsampling rejects colliding segment interiors even when both endpoint configurations are safe");
        blocked.optimizeAvoidance=true; blocked.optimizationIterations=2;
        const auto avoidance=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,blocked);
        std::cout<<"BLOCKED REPAIR success="<<avoidance.success<<" retry="<<avoidance.retryWorkpiecePose<<'\n';
        if(!avoidance.success)for(const auto& d:avoidance.diagnostics)std::cout<<d.code<<": "<<d.message<<'\n';
        check(avoidance.success && std::any_of(avoidance.diagnostics.begin(),avoidance.diagnostics.end(),
            [](const auto& d){return d.code=="avoidance_optimized";}),
            "Generic CDF repairs an interior collision and passes independent timed trajectory validation");
        ProjectCdfQpRepairOptions cancelledRepair; cancelledRepair.detectorIds={"check"}; cancelledRepair.cancelled=[] {return true;};
        const auto repairCancelled=ProjectCdfQpTrajectoryRepairService{}.repair(document,dir,"robot",request.plan.jointNames,
            evaluated.plan.trajectory,cancelledRepair);
        check(repairCancelled.cancelled && !repairCancelled.success,"Native CDF cancellation returns a structured result");
        search.maxDurationSeconds=1e-12;
        const auto timeout=ProjectExternalWorkpiecePlanner::plan(document,dir,relative,search);
        check(timeout.timedOut && !timeout.evaluation.cancelled && !timeout.evaluation.success,"Pose-search timeout is distinguished from user cancellation");
        auto bad=request; bad.minimumSingularValue=1e6;
        const auto rejected=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,bad);
        check(!rejected.success && rejected.retryWorkpiecePose,"Singularity rejection returns pose search feedback");
        bad=request;
        bad.workpiece->transform.x=point.tcpPose.translation().x();
        bad.workpiece->transform.y=point.tcpPose.translation().y();
        bad.workpiece->transform.z=point.tcpPose.translation().z();
        const auto collisionRejected=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,bad);
        check(!collisionRejected.success && collisionRejected.retryWorkpiecePose,"Colliding external workpiece rejects all endpoint candidates");
        auto pointSearch=bad;pointSearch.controlPointsOnly=true;pointSearch.independentPointsOnly=true;
        pointSearch.pointSearchAttempts=16;pointSearch.pointSearchSeconds=2;
        pointSearch.plan.cartesianControlPoints.points.resize(1);
        const auto searchedPoint=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,pointSearch);
        check(!searchedPoint.pointSearchReports.empty(),"Colliding point receives an inner feedback search instead of one proposal");
        if(!searchedPoint.pointSearchReports.empty()) {
            const auto trace=nlohmann::json::parse(searchedPoint.pointSearchReports.front());
            check(trace.at("attempts").get<int>()>1 && trace.at("attempts").get<int>()<=16,
                "Point-local budget permits repeated probes and caps attempts");
            const auto q=trace.at("orientation_delta_xyzw");
            const Eigen::Quaterniond delta(q[3].get<double>(),q[0].get<double>(),q[1].get<double>(),q[2].get<double>());
            check(Eigen::Quaterniond::Identity().angularDistance(delta)<=std::acos(-1.)/6+1e-9,
                "Measured improvement stays inside original 30-degree orientation bound");
        }
        for(int mode=2;mode<=3;++mode) {
            auto populationRequest=pointSearch;populationRequest.pointSearchStartStrategy=mode;
            const auto populationResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,populationRequest);
            bool measured=false;
            if(!populationResult.pointSearchReports.empty()) {
                const auto trace=nlohmann::json::parse(populationResult.pointSearchReports.front());
                measured=trace.at("strategies").at(mode==2?"differential_evolution":"cma_es").at("attempts").get<int>()>0;
            }
            check(measured,"Requested population evaluates real whole-robot IK/collision trials");
        }
        auto timedSearch=pointSearch;timedSearch.pointSearchSeconds=1e-12;
        const auto timedPoint=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,timedSearch);
        check(!timedPoint.success && !timedPoint.cancelled && !timedPoint.pointSearchReports.empty(),
            "Point budget exhaustion is not a solved point or user cancellation");
        check(searchedPoint.continuousPaths.empty() && searchedPoint.plan.trajectory.points.empty(),
            "Point search never claims full-path or interpolation acceptance");
        auto spraySearch=pointSearch;
        spraySearch.sprayAxis=-Eigen::Vector3d::UnitZ();
        spraySearch.sprayDistances={0.1};
        const auto& sprayTcp=spraySearch.plan.cartesianControlPoints.points.front().tcpPose;
        spraySearch.sprayTargets={sprayTcp.translation()+0.1*sprayTcp.linear()*spraySearch.sprayAxis};
        const auto spraySearched=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,spraySearch);
        check(!spraySearched.pointSearchReports.empty(),"Fixed-target spray search runs whole-robot IK and collision probes");
        auto cdfSearch=spraySearch;cdfSearch.cdfPointOptimization=true;cdfSearch.pointSearchAttempts=0;cdfSearch.cdfOptions.maxIterations=2;
        const auto cdfSearched=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,cdfSearch);
        check(!cdfSearched.pointSearchReports.empty() && nlohmann::json::parse(cdfSearched.pointSearchReports.front())["strategies"].contains("apf_qp_cdf"),"Original point repair invokes native CDF/QP rather than population algorithms");
        check(cdfSearched.continuousPaths.empty() && cdfSearched.plan.trajectory.empty(),"Point CDF/QP never queries a path or certifies segment interiors");

        auto transitionSearch=spraySearch;
        transitionSearch.pointTypes={2};transitionSearch.transitionSearchRadius=.2;
        transitionSearch.sprayTargets[0].x()+=1; // Irrelevant diagnostic anchor for non-spraying travel.
        for(int mode=1;mode<=3;++mode) {
            transitionSearch.pointSearchStartStrategy=mode;
            const auto freeResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,transitionSearch);
            bool translated=false,bounded=true,wholeRobot=false;
            if(!freeResult.pointSearchReports.empty()) {
                const auto trace=nlohmann::json::parse(freeResult.pointSearchReports.front());
                check(!trace.at("fixed_spray_target").get<bool>() && trace.at("max_angle_deg").get<double>()==180,
                    "Transition releases fixed spray target and permits full orientation search");
                wholeRobot=trace.at("ik_trials").get<int>()>0;
                for(const auto& trial:trace.at("trials"))if(trial.contains("position_m")) {
                    const auto xyz=trial.at("position_m");
                    const Eigen::Vector3d position(xyz[0].get<double>(),xyz[1].get<double>(),xyz[2].get<double>());
                    const double distance=(position-sprayTcp.translation()).norm();
                    translated=translated || distance>1e-6;bounded=bounded && distance<=.2+1e-9;
                }
            }
            check(translated && bounded && wholeRobot,"Each transition population explores bounded XYZ using real IK and collision evaluation");
        }
        auto boundarySearch=transitionSearch;boundarySearch.pointTypes={1};
        const auto boundaryResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,boundarySearch);
        check(!boundaryResult.diagnostics.empty() && boundaryResult.diagnostics.front().code=="spray_target_error",
            "Boundary cannot release fixed target even with transition settings present");
        auto invalidType=transitionSearch;invalidType.pointTypes={3};
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,invalidType).success,"Unknown point type fails closed");
        auto outOfBounds=transitionSearch;outOfBounds.transitionCenters={sprayTcp.translation()+Eigen::Vector3d(1,0,0)};
        const auto boundsResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,outOfBounds);
        check(!boundsResult.diagnostics.empty() && boundsResult.diagnostics.front().code=="transition_bounds","Source-anchored transition range cannot drift across rounds");
        auto invalidSpray=spraySearch;invalidSpray.sprayTargets[0].x()+=0.01;
        const auto invalidSprayResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,invalidSpray);
        check(!invalidSprayResult.success && !invalidSprayResult.diagnostics.empty() && invalidSprayResult.diagnostics.front().code=="spray_target_error",
            "Moving surface target independently of source TCP fails closed");
        invalidSpray=spraySearch;invalidSpray.sprayDistances[0]=-0.1;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,invalidSpray).success,"Negative spray distance rejected");
        auto zeroSpray=spraySearch;zeroSpray.sprayDistances={0};
        zeroSpray.sprayTargets={zeroSpray.plan.cartesianControlPoints.points.front().tcpPose.translation()};
        const auto zeroSprayResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,zeroSpray);
        check(!zeroSprayResult.pointSearchReports.empty(),"Zero spray distance is accepted and enters whole-robot validation");
        auto expandedSpray=spraySearch;expandedSpray.pointSearchMaxAngleDeg=70;
        expandedSpray.pointSearchMinDistances={0.08};expandedSpray.pointSearchMaxDistances={0.12};
        const auto expandedResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,expandedSpray);
        check(!expandedResult.pointSearchReports.empty(),"Late spray search accepts explicit distance and angle bounds");
        if(!expandedResult.pointSearchReports.empty()) {
            const auto trace=nlohmann::json::parse(expandedResult.pointSearchReports.front());
            bool bounded=trace.at("spray_distance_mm").get<double>()>=80-1e-6 && trace.at("spray_distance_mm").get<double>()<=120+1e-6;
            for(const auto& trial:trace.at("trials"))if(trial.contains("spray_distance_mm"))bounded=bounded && trial.at("spray_distance_mm").get<double>()>=80-1e-6 && trial.at("spray_distance_mm").get<double>()<=120+1e-6 && trial.at("reference_angle_deg").get<double>()<=70+1e-6;
            check(bounded,"Every recorded late-search probe stays inside distance and angle limits");
            check(trace.at("spray_target_error_mm").is_null() || trace.at("spray_target_error_mm").get<double>()<=expandedSpray.maximumTcpPositionDeviation*1000,
                "Retained late-search robot FK still aims at fixed spray target");
        }
        auto badDistance=expandedSpray;badDistance.pointSearchMinDistances={0.11};
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,badDistance).success,"Distance interval excluding current state rejected");
        auto finalSpray=expandedSpray;finalSpray.pointSearchMaxAngleDeg=80;
        finalSpray.pointSearchMinDistances={0.05};finalSpray.pointSearchMaxDistances={0.15};
        finalSpray.pointSearchAttempts=1;finalSpray.pointSearchSeconds=240;
        const auto finalSprayResult=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,finalSpray);
        check(!finalSprayResult.pointSearchReports.empty(),"Round-21 bounds permit 80 degrees, 50 percent distance and long per-point budget");
        auto fullBudget=finalSpray;fullBudget.pointSearchAttempts=2048;fullBudget.pointSearchSeconds=1e-12;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,fullBudget).pointSearchReports.empty(),"2048-attempt budget accepted with immediate deadline cancellation");
        auto excessiveAngle=finalSpray;excessiveAngle.pointSearchMaxAngleDeg=80.1;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,excessiveAngle).success,"Expanded angle limit cannot exceed 80 degrees");
        auto spent=pointSearch;spent.pointSearchAttemptLimits={0};
        check(ProjectExternalTrajectoryEvaluator::evaluate(document,dir,spent).pointSearchReports.empty(),"Exhausted cumulative point budget blocks all population searches");
        auto excess=pointSearch;excess.pointSearchAttempts=2049;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,excess).success,"Per-point attempts above 2048 fail closed");
        auto invalidBudget=pointSearch;invalidBudget.pointSearchAttempts=100000;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,invalidBudget).success,"Invalid point-search budget fails closed");
        auto invalidReference=pointSearch;invalidReference.pointSearchReferenceDelta={Eigen::Quaterniond(2,0,0,0)};
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,invalidReference).success,"Invalid orientation reference cannot relax source bounds");
        auto safeSearch=request;safeSearch.controlPointsOnly=true;safeSearch.independentPointsOnly=true;safeSearch.pointSearchAttempts=16;
        const auto skippedSafe=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,safeSearch);
        check(skippedSafe.success && skippedSafe.pointSearchReports.empty(),"Already valid points are not perturbed by local search");
        bad=request; bad.maxJointVelocities={1};
        const auto invalidVelocity=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,bad);
        check(!invalidVelocity.success && !invalidVelocity.retryWorkpiecePose,"Malformed velocity limits are input errors, not pose search failures");
        bad=request; bad.tcpLinkName="missing";
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,bad).success,"Unknown TCP fails closed");
        bad=request; bad.plan.cartesianControlPoints.points[1].time=0;
        check(!ProjectExternalTrajectoryEvaluator::evaluate(document,dir,bad).success,"Non-increasing timestamps rejected");
        bad=request; bad.cancelled=[] { return true; };
        const auto cancelled=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,bad);
        check(cancelled.cancelled && !cancelled.retryWorkpiecePose,"Cancellation is not reported as unreachable");
        // Endpoints coincide geometrically, but an explicitly requested full turn hits the part.
        {
            std::ofstream continuous(dir/"continuous.urdf");
            continuous<<"<robot name='turn'><link name='base'/><link name='arm'><collision><origin xyz='1 0 0'/>"
                "<geometry><box size='0.1 0.1 0.1'/></geometry></collision></link>"
                "<joint name='turn' type='continuous'><parent link='base'/><child link='arm'/>"
                "<axis xyz='0 0 1'/><limit effort='100' velocity='2'/></joint></robot>";
        }
        auto turnDocument=document;
        turnDocument.robots.front().sourcePath=(dir/"continuous.urdf").generic_u8string();
        part.transform.x=-1; turnDocument.objects={part};
        ProjectPlanningRequest turnRequest; turnRequest.robotId="robot"; turnRequest.jointNames={"turn"}; turnRequest.collisionDetectorIds={"check"};
        std::string error;
        auto turnScene=ProjectPlanningSceneBuilder::build(turnDocument,dir,turnRequest,&error);
        check(bool(turnScene),"Continuous-axis collision fixture builds");
        if(turnScene) {
            MotionValidationOptions options; options.maxJointStep=.02;
            check(turnScene->validateMotion({0},{6.283185307179586},options).valid,"Legacy shortest arc preserves endpoint planner semantics");
            options.preserveJointTurns=true;
            check(!turnScene->validateMotion({0},{6.283185307179586},options).valid,"Unwrapped full-turn validation detects intermediate collision");
            options.maxSamples=2;
            check(turnScene->validateMotion({0},{1},options).diagnosticCode=="motion_sample_budget","Oversized sampling fails closed");
            detail::CdfQueryBatch batch(*turnScene,turnDocument,dir,turnRequest,1,true);
            options={}; options.maxJointStep=.02;
            check(batch.motions({{0},{6.283185307179586}},options)[0].valid,"Cache shortest-arc result");
            options.preserveJointTurns=true;
            check(!batch.motions({{0},{6.283185307179586}},options)[0].valid,"Turn-preserving query cannot reuse shortest-arc cache");
            const auto cachedSafe=batch.motions({{0},{.001}},options);
            check(cachedSafe[0].valid && batch.motions({{0},{.001}},options)[0].valid,
                "Exact unwrapped safe motion can be reused in the same private scene");
            detail::CdfQueryBatch sampleParallel(*turnScene,turnDocument,dir,turnRequest,4,false);
            for(bool preserve : {false,true}) for(double end : {.001,1.,6.283185307179586}) {
                options.preserveJointTurns=preserve;
                const auto serialResult=turnScene->validateMotion({0},{end},options);
                const auto parallelResult=sampleParallel.motions({{0},{end}},options).front();
                check(serialResult.valid==parallelResult.valid && serialResult.diagnosticCode==parallelResult.diagnosticCode &&
                    serialResult.message==parallelResult.message && sampleParallel.pathValid({{0},{end}},options)==serialResult.valid,
                    "Single-edge parallel samples preserve the exact verdict, first collision ratio and winding policy");
            }
            auto cancelledOptions=options;cancelledOptions.cancelled=[](){return true;};
            check(sampleParallel.motions({{0},{1}},cancelledOptions).front().diagnosticCode=="cancelled" &&
                !sampleParallel.pathValid({{0},{1}},cancelledOptions),"Single-edge sample batches respect cancellation");
            auto cappedOptions=options;cappedOptions.maxSamples=2;
            check(sampleParallel.motions({{0},{1}},cappedOptions).front().diagnosticCode=="motion_sample_budget",
                "Single-edge sample batches retain the original sampling budget gate");
            options.maxSamples=1;
            check(!batch.motions({{0},{.001}},options)[0].valid,
                "A lower sample budget cannot reuse another motion certificate");
        }
    }
    {
        auto toolDocument=document;
        toolDocument.objects.push_back(part);
        simulation_project::RobotMountDesc mount;
        mount.id="test_mount"; mount.robotId="robot"; mount.linkName="l5";
        toolDocument.robotMounts.push_back(mount);
        simulation_project::AttachmentAssetDesc asset;
        asset.id="test_asset"; asset.visualPath=(dir/"part.stl").generic_u8string();
        toolDocument.attachmentAssets.push_back(asset);
        simulation_project::MountedAttachmentDesc tool;
        tool.id="test_tool"; tool.mountFrameId=mount.id; tool.assetId=asset.id;
        toolDocument.mountedAttachments.push_back(tool);
        auto& selected=toolDocument.collision.selectionSets.front().members;
        selected.clear(); simulation_project::CollisionSelectionSetMemberDesc toolMember;
        toolMember.attachmentId=tool.id; selected.push_back(toolMember);
        ProjectPlanningRequest toolPlan; toolPlan.robotId="robot";
        toolPlan.jointNames=request.plan.jointNames; toolPlan.collisionDetectorIds={"check"};
        std::string error;
        auto scene=ProjectPlanningSceneBuilder::build(toolDocument,dir,toolPlan,&error);
        check(bool(scene),"Tool-only detector includes its resolved owning planning robot");
        if(scene) {
            auto q=request.ik.model.seedJoints;
            scene->setState(q);
            const auto before=scene->attachmentWorldTransform(tool.id).translation().eval();
            q[0]+=.5; scene->setState(q);
            check((scene->attachmentWorldTransform(tool.id).translation()-before).norm()>1e-3,
                "Selected tool geometry follows planning joints");
        }
        auto other=toolDocument.robots.front(); other.id="other_robot";
        toolDocument.robots.push_back(other);
        toolDocument.robotMounts.front().robotId=other.id;
        check(!ProjectPlanningSceneBuilder::build(toolDocument,dir,toolPlan,&error),
            "Tool owned by a different robot does not satisfy planning coverage");
        toolDocument.robotMounts.front().robotId="robot";
        toolDocument.mountedAttachments.front().enabled=false;
        check(!ProjectPlanningSceneBuilder::build(toolDocument,dir,toolPlan,&error),
            "Disabled tool does not satisfy planning coverage");
    }
    std::filesystem::remove_all(dir);
    return failures?1:0;
}
