#pragma once
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <array>
#include <vector>

namespace motion_planning::detail {
// Immutable world-space first-hit surface, built from the original workpiece.
class SpraySurfaceReference {
public:
    SpraySurfaceReference(const simulation_runtime::ProjectSimulationRuntime& runtime,const std::string& id);
    bool hit(const Eigen::Vector3d& origin,const Eigen::Vector3d& direction,double& distance) const;
    std::size_t triangleCount() const {return m_triangles.size();}
private:
    using Triangle=std::array<Eigen::Vector3d,3>;
    struct Node {Eigen::Vector3d lo,hi;std::size_t begin=0,end=0;int left=-1,right=-1;};
    std::vector<Triangle> m_triangles;
    std::vector<Node> m_nodes;
    int build(std::size_t begin,std::size_t end);
};
}
