#include <ProjectMotionPlanning/TrajectoryImport.h>
#include <ProjectMotionPlanning/TrajectoryControlPointEditing.h>
#include <ProjectMotionPlanning/CartesianMotion.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cmath>
using namespace motion_planning;
void require(bool value,const std::string& message) { if(!value) throw std::runtime_error(message); }
int main(int argc,char** argv) {
    const auto path=std::filesystem::temp_directory_path()/"rs2026_fanuc_import_regression.LS";
    try {
        const auto write=[&](const std::string& mn,const std::string& positions) {
            std::ofstream out(path);out<<"/PROG TEST\n/MN\n"<<mn<<"\n/POS\n"<<positions<<"\n/END\n";
        };
        const std::string positions=
            "P[2]{GP1: UF : 2, UT : 1, CONFIG : 'N U T, 0, 0, 0', X = 200 mm, Y = -20 mm, Z = 300 mm, W = 40 deg, P = -30 deg, R = 90 deg};\n"
            "P[1]{GP1: UF : 2, UT : 1, CONFIG : 'N U T, 0, 0, 0', X = 100 mm, Y = -20 mm, Z = 300 mm, W = 179.97 deg, P = 0 deg, R = 0 deg};\n";
        const std::string mn="1: UFRAME_NUM=2 ;\n2: UTOOL_NUM=1 ;\n3: L P[1] 500mm/sec FINE ;\n4: L P[2] 250mm/sec CNT100 ;\n5: L P[1] 500mm/sec FINE ;";
        write(mn,positions);
        TrajectoryImportOptions options;options.robotId="robot";options.jointNames={"J1","J2","J3","J4","J5","J6"};
        auto result=ProjectTrajectoryImporter::importFile(path,options);
        require(result.success,result.message());const auto& points=result.plan.cartesianControlPoints.points;
        require(points.size()==3 && points[0].tcpPose.translation().x()==.1 && points[2].tcpPose.translation().x()==.1,"MN order and repeat reference");
        require(std::abs(points[1].time-.4)<1e-12 && std::abs(points[2].time-.6)<1e-12,"Destination commanded speed determines nominal timing");
        const double r=3.14159265358979323846/180.;
        const Eigen::Matrix3d expected=(Eigen::AngleAxisd(90*r,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-30*r,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(40*r,Eigen::Vector3d::UnitX())).toRotationMatrix();
        require((points[1].tcpPose.linear()-expected).norm()<1e-12,"FANUC WPR convention");
        const auto metadata=nlohmann::json::parse(result.plan.optimizationMetadata);
        require(metadata["fanuc_ls"]["moves"][1]["termination"]=="CNT100" && CartesianMotionSampler::isMoveL(result.plan),"Motion and source metadata preserved");
        auto transformed=result.plan;std::string error;
        require(ProjectTrajectoryControlPointEditor::transformCartesianTrajectory(transformed,Eigen::Vector3d(10,20,30),Eigen::Vector3d::Zero(),&error),error);
        require((transformed.cartesianControlPoints.points[0].tcpPose.translation()-Eigen::Vector3d(.11,0,.33)).norm()<1e-12,"Common trajectory transform accepts LS poses");
        std::string matrices;require(ProjectTrajectoryControlPointEditor::serializeCartesianMatrices(transformed,matrices,&error),error);
        write("1: L P[99] 500mm/sec FINE ;",positions);require(!ProjectTrajectoryImporter::importFile(path,options).success,"Undefined position rejected");
        write("1: JMP LBL[2] ;\n2: L P[1] 500mm/sec FINE ;",positions);require(!ProjectTrajectoryImporter::importFile(path,options).success,"Control flow must not be silently flattened");
        write("1: L P[1] 0mm/sec FINE ;",positions);require(!ProjectTrajectoryImporter::importFile(path,options).success,"Zero speed rejected");
        write(mn,positions+positions);require(!ProjectTrajectoryImporter::importFile(path,options).success,"Duplicate positions rejected");
        write("1: L P[1] 500mm/sec FINE ;\n2: L P[1] 500mm/sec FINE ;",positions);
        result=ProjectTrajectoryImporter::importFile(path,options);require(result.success && result.plan.cartesianControlPoints.points[1].time==options.defaultTimeStep,"Stationary move retains valid timestamp");
        if(argc>1) {
            result=ProjectTrajectoryImporter::importFile(std::filesystem::u8path(argv[1]),options);require(result.success,result.message());
            const auto& all=result.plan.cartesianControlPoints.points;require(all.size()==2480,"AZ500 full point count");
            require((all.front().tcpPose.translation()-Eigen::Vector3d(1.1,.600105,.699985)).norm()<1e-12,"AZ500 first position SI conversion");
            require((all.back().tcpPose.translation()-Eigen::Vector3d(1.1,.599325,.699669)).norm()<1e-12,"AZ500 last position SI conversion");
            for(std::size_t i=1;i<all.size();++i) require(all[i].time>all[i-1].time && all[i].tcpPose.matrix().allFinite(),"Every AZ500 timestamp and pose is valid");
            std::cout<<"AZ500 points="<<all.size()<<" nominal_duration_s="<<all.back().time<<'\n';
        }
        std::filesystem::remove(path);std::cout<<"FANUC import tests passed\n";return 0;
    } catch(const std::exception& e) {std::filesystem::remove(path);std::cerr<<e.what()<<'\n';return 1;}
}
