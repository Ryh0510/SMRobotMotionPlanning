#include <SlicingTrajPlan/VirtualBaffle.h>
#include <SlicingTrajPlan/STLProcessor.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
int main(int argc,char** argv){
    if(argc!=3){std::cerr<<"Usage: RwsBaffleInspect mesh-in-mm.stl report.json\n";return 2;}
    nlohmann::json report;
    try{
        ModelTrajPlan::ModelData model;ModelTrajPlan::STLProcessor loader;
        if(!loader.preprocessSTL(argv[1],model))throw std::runtime_error("Failed to read mesh");
        for(auto& v:model.mainModelData_.m_vertices)v*=.001f;
        const auto b=ModelTrajPlan::buildVirtualBaffle(model);
        const auto vec=[](const Eigen::Vector3d& p){return nlohmann::json::array({p.x(),p.y(),p.z()});};
        report={{"source",std::filesystem::path(argv[1]).generic_u8string()},{"input_length_unit","mm"},{"geometry_length_unit","m"},{"enabled",b.enabled},{"diagnostic",b.diagnostic},
            {"boundary_count",b.topology.loopCount},{"branch_vertices",b.topology.branchVertices},{"open_vertices",b.topology.openVertices},{"nonmanifold_edges",b.topology.nonManifoldEdges},
            {"center_m",vec(b.center)},{"normal",vec(b.normal)},{"axis_x",vec(b.axisX)},{"axis_y",vec(b.axisY)},{"thickness_mm",b.thickness*1000},{"fit_rms_mm",b.rmsResidual*1000},{"fit_max_mm",b.maxResidual*1000},
            {"infinite",true},{"section_count",b.contours.size()},{"contours_m",nlohmann::json::array()},{"display_triangles_m",nlohmann::json::array()},{"boundaries_m",nlohmann::json::array()}};
        double extent=0.5;
        for(const auto& ring:b.contours){nlohmann::json contour=nlohmann::json::array();for(const auto& p:ring){contour.push_back(vec(b.center+p.x()*b.axisX+p.y()*b.axisY));extent=std::max(extent,p.cwiseAbs().maxCoeff()*1.6+.05);}report["contours_m"].push_back(contour);}
        for(const auto& t:b.displayTriangles(extent))report["display_triangles_m"].push_back({vec(t[0]),vec(t[1]),vec(t[2])});
        ModelTrajPlan::BoundaryDetector detector;ModelTrajPlan::BoundaryData data;ModelTrajPlan::Epiphyseal ignored;ModelTrajPlan::BoundaryTopologyDiagnostics topology;
        if(detector.detectBoundaries(model,data,ignored,&topology))for(const auto* line:{&data.lowerBoundary_,&data.upperBoundary_}){nlohmann::json points=nlohmann::json::array();for(const auto& p:line->points)points.push_back(vec(p.position));report["boundaries_m"].push_back(points);}
    }catch(const std::exception& e){report={{"enabled",false},{"error",e.what()}};}
    std::ofstream out(std::filesystem::u8path(argv[2]));out<<report.dump(2)<<'\n';return out && !report.contains("error")?0:1;
}
