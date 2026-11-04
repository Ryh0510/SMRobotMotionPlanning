#include <ProjectMotionPlanning/GaApfCdfPipeline.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/CdfJointAngleImport.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <nlohmann/json.hpp>
#include <ProjectMotionPlanning/VirtualBaffleEnvironment.h>
#include <ProjectMotionPlanning/ExternalTrajectoryEvaluation.h>
#include <ProjectMotionPlanning/ExternalWorkpiecePlanning.h>
#include <ProjectMotionPlanning/CdfQpTrajectoryRepair.h>
#include "../../src/CdfQueryBatch.h"
#include "../../src/PointSearchPopulation.h"
#include "../../src/SurfacePostureSearch.h"
#include "../../src/GaRadiusEvolution.h"
#include "../../src/GpuBaffleSearch.h"
#include <future>
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
        const std::vector<double> mesh={-2,-.1,-.1,-1.8,.1,.1,-1.7,-.1,.1};
        const std::vector<double> edges={-1,-1,1,-1,1,-1,1,1,1,1,-1,1,-1,1,-1,-1};
        detail::GpuBaffleTask task;task.triangleCount=1;task.edgeCount=4;
        task.geometry={1,0,0,0,0,1,0,0,0,0,1,0, 0,0,0, 0,0,1, 1,0,0, 0,1,0, .001,0};
        std::vector<detail::GpuBaffleTask> tasks(128,task);
        for(std::size_t i=0;i<tasks.size();++i)tasks[i].geometry[3]=double(i)*.0001;
        detail::GpuBaffleSearch reference;reference.configure(mesh,edges);std::vector<double> scores;
        if(reference.evaluate(tasks,scores)) {
            std::vector<std::future<bool>> jobs;
            for(int worker=0;worker<3;++worker)jobs.push_back(std::async(std::launch::async,[&,worker] {
                detail::GpuBaffleSearch gpu;gpu.configure(mesh,edges);
                detail::GpuSerialIkModel model{};for(int j=0;j<6;++j)model[6*j+2]=1;
                model[36]=model[41]=model[46]=1;model[39]=.25+.1*worker;model[47]=.5;
                detail::GpuIkTask target;for(int j=0;j<12;++j)target.target[j]=model[36+j];
                for(int iteration=0;iteration<(worker==0?1:8);++iteration) {
                    std::vector<double> actual;if(!gpu.evaluate(tasks,actual) || actual!=scores)return false;
                    std::vector<detail::GpuIkRoot> roots;
                    if(!gpu.solveIk(model,{target},roots,10,1e-6,1e-6,.01,.1) || roots.size()!=1 || !roots[0].converged)return false;
                    for(double q:roots[0].joints)if(q!=0)return false;
                }
                return true;
            }));
            bool isolated=true;for(auto& job:jobs)isolated &= job.get();
            std::vector<double> after;
            check(isolated && reference.evaluate(tasks,after) && after==scores,
                "Concurrent CUDA streams retain independent mesh/IK buffers and releasing one planner does not reset other planners' primary context");
        } else std::cout<<"SKIP CUDA stream isolation: "<<reference.status().value("error",std::string())<<'\n';
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
    {
        CartesianIkOptions options;options.robotId="robot";options.stepSize=1;options.maxIterations=160;
        options.tolerance=1e-8;options.damping=1e-4;
        options.lower={-1,-1,-1,-7,-7,-7};options.upper={1,1,1,7,7,7};
        options.seedJoints={.2,.3,.4,.2,.5,std::acos(-1.)-.05};
        options.worldForwardKinematics=[](const std::vector<double>& q) {
            Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation()=Eigen::Vector3d(q[0],q[1],q[2]);
            pose.linear()=(Eigen::AngleAxisd(q[3],Eigen::Vector3d::UnitX())*
                Eigen::AngleAxisd(q[4],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(q[5],Eigen::Vector3d::UnitZ())).toRotationMatrix();
            return pose;
        };
        StoredMotionPlan original;original.robotId="robot";
        for(int j=0;j<6;++j)original.jointNames.push_back("j"+std::to_string(j));
        for(int i=0;i<8;++i) {
            auto q=options.seedJoints;q[5]+=.02*i;robottrajectory::TimedCartesianPoint point;
            point.time=i;point.tcpPose=options.worldForwardKinematics(q);original.cartesianControlPoints.points.push_back(point);
        }
        std::string text,error;
        check(ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(original,text,&error),"Serialize a real TCP turn-seam roundtrip fixture");
        {std::ofstream file(dir/"turn-seam.txt");file<<text;}
        TrajectoryImportOptions inputOptions;inputOptions.robotId="robot";inputOptions.jointNames=original.jointNames;
        const auto imported=ProjectTrajectoryImporter::importFile(dir/"turn-seam.txt",inputOptions);
        const auto solved=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,imported.plan,options);
        bool continuous=imported.success && solved.success && solved.plan.trajectory.points.size()==8;
        for(std::size_t i=0;i<solved.plan.trajectory.points.size();++i) {
            const auto& q=solved.plan.trajectory.points[i].q;
            continuous &= std::abs(q[5]-(options.seedJoints[5]+.02*i))<1e-6;
            if(i)continuous &= std::abs(q[5]-solved.plan.trajectory.points[i-1].q[5])<.021;
        }
        check(continuous,"TCP text reimport preserves legal wrist turns through pi instead of introducing a full-revolution playback jump");
        auto exact=solved.plan;std::string annotated;
        check(ProjectTrajectoryControlPointEditor::serializeJointTcpMatrices(exact,options.worldForwardKinematics,false,annotated,&error),
            "TCP export embeds optional exact joint seeds without changing numeric matrix rows");
        {std::ofstream file(dir/"turn-seam-exact.txt");file<<annotated;}
        const auto importedExact=ProjectTrajectoryImporter::importFile(dir/"turn-seam-exact.txt",inputOptions);
        auto distant=options;distant.seedJoints.assign(6,0);
        const auto restored=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,importedExact.plan,distant);
        bool same=restored.success && restored.plan.trajectory.points.size()==exact.trajectory.points.size();
        if(same)for(std::size_t i=0;i<exact.trajectory.points.size();++i)same &= restored.plan.trajectory.points[i].q==exact.trajectory.points[i].q;
        check(same,"Reimport restores exact verified joint configurations and turns even from a different robot starting posture");
        auto displayOptions=distant;displayOptions.mapIrb4600StoredJointSigns=true;
        displayOptions.worldForwardKinematics=[&](const auto& q){return options.worldForwardKinematics(ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(q));};
        const auto displayed=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,importedExact.plan,displayOptions);
        bool displaySame=displayed.success && displayed.plan.trajectory.points.size()==exact.trajectory.points.size();
        if(displaySame)for(std::size_t i=0;i<exact.trajectory.points.size();++i)
            displaySame &= ProjectTrajectoryInverseKinematics::irb4600RobotSystemJointValues(displayed.plan.trajectory.points[i].q)==exact.trajectory.points[i].q;
        check(displaySame,"Runtime encoder hints survive the GUI ABB sign conversion without changing branch or wrist turns");
        std::string displayedText;
        check(ProjectTrajectoryControlPointEditor::serializeJointTcpMatrices(displayed.plan,options.worldForwardKinematics,true,displayedText,&error),
            "GUI TCP export stores actual-model encoder hints after converting displayed ABB signs");
        {std::ofstream file(dir/"turn-seam-display.txt");file<<displayedText;}
        const auto displayImport=ProjectTrajectoryImporter::importFile(dir/"turn-seam-display.txt",inputOptions);
        const auto displayRestored=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,displayImport.plan,distant);
        bool runtimeSame=displayRestored.success && displayRestored.plan.trajectory.points.size()==exact.trajectory.points.size();
        if(runtimeSame)for(std::size_t i=0;i<exact.trajectory.points.size();++i)runtimeSame &= displayRestored.plan.trajectory.points[i].q==exact.trajectory.points[i].q;
        check(runtimeSame,"GUI export and native reimport preserve exactly the same actual encoder sequence");
        auto edited=importedExact.plan;edited.cartesianControlPoints.points.front().tcpPose.translation().x()+=.01;
        const auto editedSolve=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(document,edited,distant);
        check(!editedSolve.success || editedSolve.plan.trajectory.points.front().q!=exact.trajectory.points.front().q,
            "Edited TCP targets cannot reuse mismatched joint provenance");
    }
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
        auto raw=budgetRequest;raw.plan.trajectory.points.clear();raw.plan.cartesianControlPoints.points.resize(2);
        raw.plan.cartesianControlPoints.points[1].time=1.;raw.ik.cachedCandidates.clear();raw.ik.preferredJoints.clear();
        const auto initialJointsBefore=budgetDocument.robots.front().initialJoints.size();
        const auto fromRaw=runGaApfCdfPipeline(budgetDocument,dir,raw,false);
        check(fromRaw.success && std::any_of(fromRaw.diagnostics.begin(),fromRaw.diagnostics.end(),
            [](const auto& d){return d.code=="pipeline_initial_top1";}) &&
            budgetDocument.robots.front().initialJoints.size()==initialJointsBefore,
            "Raw Cartesian pipeline performs physical all-IK and Top-1 initialization without mutating the caller project");
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
        {
            auto moveLDocument=document;moveLDocument.objects.push_back(part);
            ProjectPlanningRequest setup;setup.robotId="robot";setup.jointNames=request.plan.jointNames;setup.collisionDetectorIds={"check"};
            std::string error;auto scene=ProjectPlanningSceneBuilder::build(moveLDocument,dir,setup,&error);
            auto sampler=scene?CartesianMotionSampler::create(*scene,"l5",Eigen::Isometry3d::Identity(),error):nullptr;
            check(bool(sampler),"MoveL uses an immutable six-axis chain independently verified against actual runtime FK");
            {
                std::ifstream input(dir/"robot.urdf");
                std::string continuousUrdf((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
                const std::string jointTag = "name='j5' type='revolute'";
                const auto at = continuousUrdf.find(jointTag);
                continuousUrdf.replace(at, jointTag.size(), "name='j5' type='continuous'");
                {std::ofstream output(dir/"robot-continuous.urdf"); output << continuousUrdf;}
                auto continuousDocument = moveLDocument;
                continuousDocument.robots.front().sourcePath = (dir/"robot-continuous.urdf").generic_u8string();
                auto continuousScene = ProjectPlanningSceneBuilder::build(continuousDocument, dir, setup, &error);
                auto continuousSampler = continuousScene ? CartesianMotionSampler::create(*continuousScene, "l5", Eigen::Isometry3d::Identity(), error) : nullptr;
                auto a = request.ik.model.seedJoints, b = a;
                a[5] = 2*std::acos(-1.) + .1; b[5] = a[5] + .1;
                std::vector<double> midpoint;
                const bool continuous = continuousSampler && continuousSampler->sample(a, b, .5, midpoint, error);
                check(continuous && std::abs(midpoint[5] - (a[5] + .05)) < 1e-7,
                    "MoveL supports actual continuous wrists and retains unwrapped encoder turns beyond a full revolution");
            }
            if(sampler) {
                auto a=request.ik.model.seedJoints,b=a;b[0]+=.2;b[1]+=.03;b[3]+=.05;
                const auto from=sampler->forward(a),to=sampler->forward(b);
                bool straight=true;double jointChordDeviation=0;
                for(double f:{.1,.25,.5,.75,.9}) {
                    std::vector<double> q;straight &= sampler->sample(a,b,f,q,error);
                    if(q.size()!=6){straight=false;break;}
                    runtime.setRobotJointValues("robot",request.plan.jointNames,q);
                    const auto actual=runtime.robot("robot")->instance->getLinkTransform("l5");
                    const Eigen::Vector3d target=(1-f)*from.translation()+f*to.translation();
                    straight &= (actual.translation()-target).norm()<1e-8 &&
                        Eigen::Quaterniond(actual.linear()).angularDistance(Eigen::Quaterniond(from.linear()).slerp(f,Eigen::Quaterniond(to.linear())))<1e-8;
                    auto linear=a;for(std::size_t j=0;j<6;++j)linear[j]+=f*(b[j]-a[j]);
                    jointChordDeviation=std::max(jointChordDeviation,(sampler->forward(linear).translation()-target).norm());
                }
                check(straight && jointChordDeviation>1e-4,"MoveL follows the Cartesian straight line and slerp rather than the curved TCP of joint interpolation");
                std::vector<std::vector<double>> internal;
                bool bounded=sampler->connection(a,b,.002,10000,internal,error);
                for(std::size_t i=1;i<internal.size();++i)for(std::size_t j=0;j<6;++j)bounded &= std::abs(internal[i][j]-internal[i-1][j])<=.00200001;
                check(bounded && internal.size()>2 && internal.front()==a && internal.back()==b,
                    "MoveL internal IK sampling preserves exact endpoint turns and the requested actual joint-step bound");
                check(!sampler->connection(a,b,.00025,2,internal,error) && internal.empty(),"MoveL cannot turn a sample-budget failure into a safe connection");
                check(!sampler->connection(a,b,.002,10000,internal,error,[](){return true;}) && internal.empty(),"MoveL respects cancellation before any accepted path");
                StoredMotionPlan moveLPlan;moveLPlan.robotId="robot";moveLPlan.jointNames=request.plan.jointNames;
                moveLPlan.optimizationMetadata=R"({"motion":"cartesian_linear"})";
                for(int k=0;k<2;++k) {
                    robottrajectory::TimedJointPoint p;p.time=k;p.q=k?b:a;moveLPlan.trajectory.points.push_back(p);
                    robottrajectory::TimedCartesianPoint tcp;tcp.time=k;tcp.tcpPose=sampler->forward(p.q);moveLPlan.cartesianControlPoints.points.push_back(tcp);
                }
                {
                    ProjectCdfQpRepairOptions preflight;
                    preflight.detectorIds={"check"};preflight.stage=ProjectCdfQpRepairOptions::Stage::ApfOnly;
                    preflight.selectiveSupport=true;preflight.densifyKnots=false;
                    preflight.safetyMargin=0;preflight.targetClearance=0;preflight.apfMaxTcpDeviation=1.;
                    preflight.retimeOutput=false;preflight.postSmoothingIterations=0;
                    preflight.cartesianMotion=sampler;preflight.validationMaxJointStep=.0013;
                    preflight.worldForwardKinematics=[&](const auto& q){return sampler->forward(q);};
                    std::vector<std::vector<double>> executionSamples;
                    const bool sampled=sampler->connection(a,b,preflight.validationMaxJointStep,10000,executionSamples,error);
                    double largest=0;for(std::size_t j=0;j<a.size();++j)largest=std::max(largest,std::abs(b[j]-a[j]));
                    const auto guideSteps=std::max<std::size_t>(1,static_cast<std::size_t>(std::ceil(largest/.001)));
                    double missedTime=-1;
                    if(sampled)for(std::size_t k=1;k+1<executionSamples.size();++k) {
                        const double f=double(k)/(executionSamples.size()-1);
                        if(std::abs(f*guideSteps-std::round(f*guideSteps))>1e-5 &&
                            std::abs(f*16-std::round(f*16))>1e-5) {missedTime=f;break;}
                    }
                    check(missedTime>0,"Process preflight fixture contains an execution sample absent from the earlier fixed-fraction guide checks");
                    if(missedTime>0) {
                        bool finalCollisionStarted=false;
                        preflight.progress=[&](const std::string& event){if(event=="Final full-path collision verification")finalCollisionStarted=true;};
                        preflight.processStateAdmissible=[missedTime](const auto&,double time){return std::abs(time-missedTime)>1e-10;};
                        const auto rejected=ProjectCdfQpTrajectoryRepairService{}.repair(moveLDocument,dir,"robot",
                            request.plan.jointNames,moveLPlan.trajectory,preflight);
                        check(!rejected.success && rejected.plan.trajectory.empty() && !finalCollisionStarted &&
                            rejected.statistics.inputWaypointCount==2 && rejected.apfConnectionFailures.size()==1 &&
                            rejected.apfConnectionFailures.front().beginTime==0. && rejected.apfConnectionFailures.front().endTime==1. &&
                            rejected.apfConnectionFailures.front().requiresDirectProcess &&
                            std::any_of(rejected.diagnostics.begin(),rejected.diagnostics.end(),[](const auto& d){return d.code=="apf_process_preflight_rejected";}),
                            "Execution-sampled process failure returns its exact source interval to GA before the expensive final collision sweep");
                        preflight.processStateAdmissible=[](const auto&,double){return true;};
                        finalCollisionStarted=false;
                        const auto accepted=ProjectCdfQpTrajectoryRepairService{}.repair(moveLDocument,dir,"robot",
                            request.plan.jointNames,moveLPlan.trajectory,preflight);
                        check(accepted.success && accepted.plan.trajectory.points.size()==2 && finalCollisionStarted &&
                            std::any_of(accepted.diagnostics.begin(),accepted.diagnostics.end(),[](const auto& d){return d.code=="apf_process_preflight_passed";}),
                            "A process-valid MoveL still executes the fresh final collision gate and preserves both original controls");
                    }
                }
                std::string text;check(ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(moveLPlan,text,&error),"Serialize sparse MoveL controls with motion law and exact branch provenance");
                {std::ofstream file(dir/"movel-roundtrip.txt");file<<text;}
                TrajectoryImportOptions input;input.robotId="robot";input.jointNames=request.plan.jointNames;
                const auto imported=ProjectTrajectoryImporter::importFile(dir/"movel-roundtrip.txt",input);
                CartesianIkOptions inverse;inverse.robotId="robot";inverse.jointNames=request.plan.jointNames;inverse.seedJoints.assign(6,0.);
                for(const auto& bound:scene->jointBounds()){inverse.lower.push_back(bound.lower);inverse.upper.push_back(bound.upper);}
                inverse.worldForwardKinematics=[&](const auto& q){return sampler->forward(q);};
                const auto restored=ProjectTrajectoryInverseKinematics::solveCartesianControlPoints(moveLDocument,imported.plan,inverse);
                std::vector<double> oldFrame,newFrame;
                const bool restoredMotion=restored.success && CartesianMotionSampler::isMoveL(restored.plan) &&
                    sampler->sample(a,b,.37,oldFrame,error) && sampler->sample(restored.plan.trajectory.points.front().q,restored.plan.trajectory.points.back().q,.37,newFrame,error);
                check(restoredMotion && oldFrame==newFrame && restored.plan.trajectory.points.size()==2,
                    "Sparse TCP export/reimport retains MoveL and exactly reproduces playback IK without adding exported controls");
                std::string jointText;
                check(ProjectTrajectoryControlPointEditor::serializeJointTable(moveLPlan,jointText,&error),
                    "Joint TXT exports sparse MoveL commands with exact radians and times");
                {std::ofstream file(dir/"movel-joints.txt");file<<jointText;}
                const auto jointImport=ProjectTrajectoryImporter::importFile(dir/"movel-joints.txt",input);
                const auto cdfImport=ProjectCdfJointAngleImporter::importFile(dir/"movel-joints.txt");
                std::vector<double> jointFrame;
                const bool jointMotion=jointImport.success && cdfImport.success && cdfImport.hasMotionMetadata &&
                    CartesianMotionSampler::isMoveL(jointImport.plan) && CartesianMotionSampler::isMoveL(cdfImport.plan) &&
                    jointImport.plan.trajectory.points.size()==2 && cdfImport.plan.trajectory.points.size()==2 &&
                    jointImport.plan.trajectory.points.front().q==a && jointImport.plan.trajectory.points.back().q==b &&
                    cdfImport.plan.trajectory.points.front().q==a && cdfImport.plan.trajectory.points.back().q==b &&
                    sampler->sample(jointImport.plan.trajectory.points.front().q,jointImport.plan.trajectory.points.back().q,.37,jointFrame,error);
                check(jointMotion && jointFrame==oldFrame,
                    "Both joint import routes preserve exact MoveL interior playback, endpoint encoders and turn counts");
                auto wrongRobot=input;wrongRobot.robotId="other_robot";
                check(!ProjectTrajectoryImporter::importFile(dir/"movel-joints.txt",wrongRobot).success,
                    "Versioned joint table refuses a different robot identity");
                const auto numericStart=jointText.find('\n',jointText.find("# RWS_JOINT_TABLE_V1 "))+1;
                auto damaged=jointText;damaged.replace(numericStart,damaged.find('\t',numericStart)-numericStart,"999.000000");
                {std::ofstream file(dir/"movel-joints-damaged.txt");file<<damaged;}
                check(!ProjectTrajectoryImporter::importFile(dir/"movel-joints-damaged.txt",input).success &&
                    !ProjectCdfJointAngleImporter::importFile(dir/"movel-joints-damaged.txt").success,
                    "Edited joint rows cannot silently restore stale exact metadata or fall back to TCP interpretation");
                const auto markerStart=jointText.find("# RWS_JOINT_TABLE_V1 ");
                const auto markerEnd=jointText.find('\n',markerStart);
                auto malformed=jointText;malformed.replace(markerStart,markerEnd-markerStart,"# RWS_JOINT_TABLE_V1 {bad}");
                {std::ofstream file(dir/"movel-joints-malformed.txt");file<<malformed;}
                check(!ProjectTrajectoryImporter::importFile(dir/"movel-joints-malformed.txt",input).success,
                    "Malformed joint metadata fails explicitly without changing the motion law");
                auto legacy=jointText;legacy.erase(markerStart,markerEnd-markerStart+1);
                {std::ofstream file(dir/"legacy-degrees.txt");file<<legacy;}
                const auto legacyImport=ProjectTrajectoryImporter::importFile(dir/"legacy-degrees.txt",input);
                check(legacyImport.success && !CartesianMotionSampler::isMoveL(legacyImport.plan) &&
                    legacyImport.plan.trajectory.points.front().q.size()==6 &&
                    std::abs(legacyImport.plan.trajectory.points.front().q[0]-a[0])<1e-8,
                    "Legacy time_s degree tables convert units and retain their original joint interpolation");
                detail::CdfQueryBatch queries(*scene,moveLDocument,dir,setup,2,true);queries.setCartesianMotion(sampler);
                const auto certificates=std::make_shared<detail::CdfMotionCertificates>();
                queries.setMotionCertificates(certificates);
                MotionValidationOptions validation;validation.maxJointStep=.002;validation.preserveJointTurns=true;
                const auto physical=queries.motions({a,b},validation);
                check(physical.size()==1 && physical.front().valid,"MoveL collision queries use the same actual TCP interpolation with isolated physical scenes");
                auto nextScene=ProjectPlanningSceneBuilder::build(moveLDocument,dir,setup,&error);
                const auto nextSampler=nextScene?CartesianMotionSampler::create(*nextScene,"l5",Eigen::Isometry3d::Identity(),error):nullptr;
                check(bool(nextSampler),"Feedback certificate fixture rebuilds the same immutable actual robot scene");
                if(nextSampler) {
                    detail::CdfQueryBatch next(*nextScene,moveLDocument,dir,setup,1,true);
                    next.setCartesianMotion(nextSampler);next.setMotionCertificates(certificates);
                    const auto before=nextScene->queryCounts().stateValidationRequests;
                    check(next.motions({a,b},validation).front().valid && nextScene->queryCounts().stateValidationRequests==before,
                        "Unchanged original MoveL geometry reuses its exact certificate across feedback scenes");
                    next.setMotionCertificates({});
                    check(next.motions({a,b},validation).front().valid && nextScene->queryCounts().stateValidationRequests==before,
                        "Imported original-edge certificates remain available to later checks in the same repair");
                    next.setMotionCertificates(certificates);
                    auto changed=b;changed[0]+=1e-7;
                    check(next.motions({a,changed},validation).front().valid && nextScene->queryCounts().stateValidationRequests>before,
                        "Even a small endpoint change requires a new exact physical motion query");
                    const auto fineBefore=nextScene->queryCounts().stateValidationRequests;
                    auto fine=validation;fine.maxJointStep*=.5;
                    check(next.motions({a,b},fine).front().valid && nextScene->queryCounts().stateValidationRequests>fineBefore,
                        "A finer MoveL validation step requires fresh physical queries");
                    auto limited=validation;limited.maxSamples=2;
                    check(!next.motions({a,b},limited).front().valid,"A reduced MoveL sample budget cannot reuse a feedback certificate");
                    const auto freshBefore=nextScene->queryCounts().stateValidationRequests;
                    check(next.motions({a,b},validation,false).front().valid && nextScene->queryCounts().stateValidationRequests>freshBefore,
                        "Independent replay bypasses both local and feedback geometry certificates");
                    auto cancelled=validation;cancelled.cancelled=[](){return true;};
                    check(!next.motions({a,b},cancelled).front().valid,"Cancellation rejects an otherwise cached feedback motion");
                    next.setCartesianMotion({});const auto jointBefore=nextScene->queryCounts().stateValidationRequests;
                    check(next.motions({a,b},validation).front().valid && nextScene->queryCounts().stateValidationRequests>jointBefore,
                        "Joint interpolation cannot reuse a Cartesian motion certificate");
                }
            }
        }
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
        check(std::any_of(preferred.diagnostics.begin(),preferred.diagnostics.end(),[](const auto& d){return d.code=="ga_parallel_candidates";}),
            "Independent GA points use isolated parallel candidate workers before the common graph gate");
        auto candidateJob=preserved;candidateJob.gaPointWorkers=1;candidateJob.gaCandidateGenerationOnly=true;
        candidateJob.gaSearchControls={1};candidateJob.ik.cachedCandidates.resize(candidateJob.plan.cartesianControlPoints.points.size());
        int repeatedIkEvents=0;candidateJob.progress=[&](const std::string& text) {
            if(text.find("IK point ")==0 || text.find("ik_candidates")!=std::string::npos)++repeatedIkEvents;
        };
        const auto contextOnly=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,candidateJob);
        check(contextOnly.candidateLayers.size()==candidateJob.plan.cartesianControlPoints.points.size() &&
            contextOnly.candidateLayers.front().candidates.empty() && contextOnly.pointSearchReports.size()==1 &&
            nlohmann::json::parse(contextOnly.pointSearchReports.front()).value("point_index",std::size_t(-1))==1 &&
            repeatedIkEvents==0 && contextOnly.plan.trajectory.empty(),
            "A GA point job preserves empty parent context without re-solving unrelated original IK layers or claiming a continuous path");
        candidateJob.ik.cachedCandidates.clear();
        const auto missingContext=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,candidateJob);
        check(!missingContext.success && std::any_of(missingContext.diagnostics.begin(),missingContext.diagnostics.end(),
            [](const auto& d){return d.code=="invalid_ga_candidate_job";}),
            "A candidate job without its parent's full layer context is rejected instead of silently enumerating the entire trajectory");
        auto serialPreference=preference;serialPreference.gaPointWorkers=1;
        const auto serialPreferred=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,serialPreference);
        bool sameOriginal=serialPreferred.success && preferred.success && serialPreferred.plan.trajectory.points.size()==preferred.plan.trajectory.points.size();
        if(sameOriginal)for(std::size_t i=0;i<preferred.plan.trajectory.points.size();++i)
            for(std::size_t j=0;j<6;++j)sameOriginal &= std::abs(preferred.plan.trajectory.points[i].q[j]-serialPreferred.plan.trajectory.points[i].q[j])<1e-8;
        check(sameOriginal,"Serial and parallel candidate schedules both preserve safe original controls after ordered graph selection");
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
        auto sparseEffort=apfEffort;sparseEffort.selectiveSupport=true;
        const auto sparseBlocked=ProjectCdfQpTrajectoryRepairService{}.repair(feedbackDocument,dir,"robot",request.plan.jointNames,crossed,sparseEffort);
        check(sparseBlocked.statistics.inputWaypointCount==6,
            "A blocked two-control interval adds only four planning knots independently of collision sampling resolution");
        auto clearDocument=document;auto farPart=part;clearDocument.objects.push_back(farPart);
        auto same=crossed;same.points.back().q=same.points.front().q;
        const auto sparseClear=ProjectCdfQpTrajectoryRepairService{}.repair(clearDocument,dir,"robot",request.plan.jointNames,same,sparseEffort);
        check(sparseClear.success && sparseClear.plan.trajectory.points.size()==2 && sparseClear.statistics.inputWaypointCount==2,
            "A directly safe motion retains exactly its two original controls without post-optimization downsampling");
        auto curved=same;curved.points.back().q[0]+=.01;
        auto curvedEffort=sparseEffort;curvedEffort.retimeOutput=false;curvedEffort.apfMaxTcpDeviation=1.;
        curvedEffort.validationMaxJointStep=.00025;
        const double origin=curved.points.front().q[0];
        curvedEffort.worldForwardKinematics=[](const auto& q) {
            Eigen::Isometry3d pose=Eigen::Isometry3d::Identity();pose.translation().x()=q[0];return pose;
        };
        curvedEffort.processStateAdmissible=[origin](const auto& q,double time) {
            const double f=(time-3.)/4.;return std::abs(q[0]-origin-.01*f*f)<=.00002;
        };
        curvedEffort.surfaceProject=[origin](auto& q,std::size_t,double fraction){q[0]=origin+.01*fraction*fraction;return true;};
        const auto curvedResult=ProjectCdfQpTrajectoryRepairService{}.repair(clearDocument,dir,"robot",request.plan.jointNames,curved,curvedEffort);
        check(!curvedResult.success && curvedResult.statistics.inputWaypointCount==6 &&
            curvedResult.apfConnectionFailures.size()==1 && curvedResult.apfConnectionFailures.front().beginTime==3. &&
            curvedResult.apfConnectionFailures.front().endTime==7. && curvedResult.apfConnectionFailures.front().requiresDirectProcess,
            "A process-invalid chord gets only four support knots, then returns its unresolved original interval to GA");
        auto mixed=crossed;auto early=crossed.points.front();early.time=1.;mixed.points.insert(mixed.points.begin(),early);
        const auto mixedResult=ProjectCdfQpTrajectoryRepairService{}.repair(feedbackDocument,dir,"robot",request.plan.jointNames,mixed,sparseEffort);
        check(mixedResult.statistics.inputWaypointCount==7 && !mixedResult.stages.empty() &&
            mixedResult.stages.front().plan.trajectory.points[0].q==early.q &&
            mixedResult.stages.front().plan.trajectory.points[1].q==crossed.points.front().q &&
            mixedResult.stages.front().plan.trajectory.points[1].time==3. && mixedResult.success && mixedResult.plan.trajectory.points.size()==7,
            "Only the colliding original interval receives four knots; its safe neighbor retains its sparse seed and successful APF preserves the knot count");
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
        auto directFeedback=feedback;directFeedback.gaRejectedConnections.front().requiresDirectProcess=true;
        const auto directRenewed=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,directFeedback);
        bool directSamples=directRenewed.success;
        if(directRenewed.success)for(double fraction:{.25,.5,.75}) {
            const auto& a=directRenewed.plan.trajectory.points[1];const auto& b=directRenewed.plan.trajectory.points[2];
            auto q=a.q;for(std::size_t j=0;j<q.size();++j)q[j]+=fraction*(b.q[j]-a.q[j]);
            runtime.setRobotJointValues("robot",request.plan.jointNames,q);
            const auto tcp=runtime.robot("robot")->instance->getLinkTransform("l5");
            directSamples &= (tcp.translation()+.1*tcp.linear()*directFeedback.sprayAxis-
                ((1-fraction)*directFeedback.sprayTargets[1]+fraction*directFeedback.sprayTargets[2])).norm()<=directFeedback.maximumTcpPositionDeviation;
        }
        check(directSamples,"GA consumes direct-process feedback and checks the selected fixed-distance chord before returning it to full APF validation");
        auto pairedJob=directFeedback;pairedJob.gaCandidateGenerationOnly=true;pairedJob.gaSearchControls={1};pairedJob.gaPointWorkers=1;
        pairedJob.ik.cachedCandidates.resize(4);
        for(std::size_t i=0;i<4;++i)pairedJob.ik.cachedCandidates[i]={pairedJob.optimizationSeed.points[i].q};
        const auto processPointPool=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,pairedJob);
        pairedJob.gaRejectedConnections.front().requiresDirectProcess=false;
        const auto collisionPointPool=ProjectExternalTrajectoryEvaluator::evaluate(document,dir,pairedJob);
        bool samePointPool=processPointPool.success && collisionPointPool.success && processPointPool.candidateLayers.size()==4 &&
            collisionPointPool.candidateLayers.size()==4 && processPointPool.candidateLayers[1].candidates.size()==collisionPointPool.candidateLayers[1].candidates.size();
        if(samePointPool)for(std::size_t i=0;i<processPointPool.candidateLayers[1].candidates.size();++i)
            samePointPool &= processPointPool.candidateLayers[1].candidates[i].joints==collisionPointPool.candidateLayers[1].candidates[i].joints;
        check(samePointPool && !processPointPool.candidateLayers[1].candidates.empty(),
            "A direct-process edge requirement does not discard point-safe offspring against fixed old neighbors before paired graph selection");
        auto transitionDirect=directFeedback;transitionDirect.pointTypes.assign(4,2);
        check(ProjectExternalTrajectoryEvaluator::evaluate(document,dir,transitionDirect).success,
            "Direct-process feedback retains transition workspace semantics instead of imposing spray distance on non-spray chords");
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
            const auto turnCertificates=std::make_shared<detail::CdfMotionCertificates>();
            batch.setMotionCertificates(turnCertificates);
            options={}; options.maxJointStep=.02;
            check(batch.motions({{0},{6.283185307179586}},options)[0].valid,"Cache shortest-arc result");
            options.preserveJointTurns=true;
            check(!batch.motions({{0},{6.283185307179586}},options)[0].valid,"Turn-preserving query cannot reuse shortest-arc cache");
            detail::CdfQueryBatch feedbackTurns(*turnScene,turnDocument,dir,turnRequest,1,true);
            feedbackTurns.setMotionCertificates(turnCertificates);
            check(!feedbackTurns.motions({{0},{6.283185307179586}},options)[0].valid,
                "Feedback certificates preserve encoder turns across separate query batches");
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
            detail::CdfQueryBatch eightWorkers(*turnScene,turnDocument,dir,turnRequest,8,true);
            const std::vector<std::vector<double>> progressPath{{0},{.001},{0},{1},{0},{.002},{0},{.003},{0}};
            bool progressOrdered=true;std::size_t lastCompleted=0,lastTotal=0;
            const auto observe=[&](std::size_t done,std::size_t total) {
                progressOrdered &= done>=lastCompleted && done<=total && total==progressPath.size()-1;
                lastCompleted=done;lastTotal=total;
            };
            const auto observed=eightWorkers.motions(progressPath,options,true,observe);
            bool exact=true;for(std::size_t i=0;i<observed.size();++i) {
                const auto expected=turnScene->validateMotion(progressPath[i],progressPath[i+1],options);
                exact &= expected.valid==observed[i].valid && expected.diagnosticCode==observed[i].diagnosticCode && expected.message==observed[i].message;
            }
            check(exact && progressOrdered && lastCompleted==observed.size() && lastTotal==observed.size(),
                "Eight isolated motion workers retain ordered exact verdicts and monotonic completed-edge progress");
            lastCompleted=lastTotal=0;eightWorkers.motions(progressPath,options,true,observe);
            check(progressOrdered && lastCompleted==observed.size(),"Cached motion batches still report complete monotonic progress");
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
