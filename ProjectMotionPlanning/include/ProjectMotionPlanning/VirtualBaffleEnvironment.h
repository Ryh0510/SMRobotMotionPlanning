#pragma once
#include <SlicingTrajPlan/VirtualBaffle.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <nlohmann/json.hpp>
#include <memory>
namespace motion_planning {
namespace detail { class BaffleRimIndex; class GpuBaffleSearch; }
struct WorkpieceBaffle { std::string objectId; ModelTrajPlan::VirtualBaffle geometry; bool visible=true; };
class VirtualBaffleEnvironment {
public:
    void build(const simulation_runtime::ProjectSimulationRuntime& runtime);
    // Uses every enabled link collision element and installed tool, independently of detector pair filters.
    bool validState(const simulation_runtime::ProjectSimulationRuntime& runtime,
        double clearance, std::string& reason) const;
    const std::vector<WorkpieceBaffle>& baffles() const { return baffles_; }
    // Geometric search merit (not signed clearance and never an acceptance test).
    double searchViolation(const simulation_runtime::ProjectSimulationRuntime&, double clearance) const;
    struct SearchPose {
        bool valid=false;
        std::vector<Eigen::Isometry3d> bodies,objects;
    };
    SearchPose captureSearchPose(const simulation_runtime::ProjectSimulationRuntime&) const;
    std::vector<double> searchViolationBatch(const std::vector<SearchPose>&,double clearance,bool gpu) const;
    nlohmann::json gpuSearchStatus() const;
    nlohmann::json status() const;
    static constexpr const char* extensionKey="rws.virtual_baffles";
private:
    struct Body {
        std::string robotId,link,attachmentId;
        Eigen::Isometry3d local=Eigen::Isometry3d::Identity();
        std::vector<ModelTrajPlan::VirtualBaffle::Triangle> triangles;
        Eigen::Vector3d lower=Eigen::Vector3d::Constant(1e300),upper=Eigen::Vector3d::Constant(-1e300);
        struct Node {Eigen::Vector3d lower,upper;std::size_t first=0,count=0;int left=-1,right=-1;};
        std::vector<Node> nodes;
        std::vector<std::size_t> triangleOrder;
        double margin=0;
    };
    std::vector<WorkpieceBaffle> baffles_;
    std::vector<std::shared_ptr<const detail::BaffleRimIndex>> searchIndices_;
    std::vector<Body> bodies_;
    std::string error_;
    double searchViolation(const SearchPose&,double clearance) const;
    mutable std::shared_ptr<detail::GpuBaffleSearch> gpu_;
    mutable std::vector<int> gpuBodyOffsets_,gpuEdgeOffsets_;
    mutable std::size_t gpuFallbackBatches_=0,cpuSearchQueries_=0;
};
}
