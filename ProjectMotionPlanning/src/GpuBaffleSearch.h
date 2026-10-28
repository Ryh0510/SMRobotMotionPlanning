#pragma once
#include <array>
#include <memory>
#include <vector>
#include <nlohmann/json.hpp>

namespace motion_planning { namespace detail {
// Actual mesh/slab/aperture search merit only. Never a collision certificate.
struct GpuBaffleTask {
    int triangleBegin=0,triangleCount=0,edgeBegin=0,edgeCount=0;
    // Row-major 3x4 transform, center, normal, axisX, axisY, half slab, margin.
    std::array<double,26> geometry{};
    // Optional conservative slab broad-phase subset (body-relative indices).
    // triangleCount remains the FULL mesh count for unchanged merit normalization.
    std::vector<int> triangleIndices;
};
// Product of exponentials from the selected actual six-revolute-joint chain:
// six world-axis/pivot pairs (36 doubles), followed by world home TCP (3x4).
using GpuSerialIkModel=std::array<double,48>;
struct GpuIkTask {std::array<double,12> target{};std::array<double,6> seed{};};
struct GpuIkRoot {std::array<double,6> joints{};bool converged=false;};
class GpuBaffleSearch {
public:
    GpuBaffleSearch();
    ~GpuBaffleSearch();
    void configure(std::vector<double> triangles,std::vector<double> edges);
    bool evaluate(const std::vector<GpuBaffleTask>& tasks,std::vector<double>& scores);
    bool solveIk(const GpuSerialIkModel& model,const std::vector<GpuIkTask>& tasks,
        std::vector<GpuIkRoot>& roots,int iterations,double positionTolerance,double orientationTolerance,double damping,double step);
    nlohmann::json status() const;
private:
    class Impl;
    std::unique_ptr<Impl> impl;
};
}}
