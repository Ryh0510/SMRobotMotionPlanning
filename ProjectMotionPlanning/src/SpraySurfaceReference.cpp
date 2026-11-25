#include "SpraySurfaceReference.h"
#include <AssetCore/AssetManager.h>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace motion_planning::detail {
SpraySurfaceReference::SpraySurfaceReference(const simulation_runtime::ProjectSimulationRuntime& runtime,const std::string& id) {
    for(const auto& object:runtime.objects()) {
        if(id.empty()?object.objectType!="workpiece":object.documentId!=id)continue;
        const auto model=assetcore::AssetManager::instance().loadModel(object.sourcePath.u8string(),1.f);
        if(!model)throw std::runtime_error("Cannot load spray target mesh: "+object.sourcePath.u8string());
        for(const auto& part:model->subMeshes()) {
            const auto& g=part.geometry;const std::size_t n=g.indices.empty()?g.positions.size():g.indices.size();
            if(n%3)throw std::runtime_error("Spray target mesh is not triangulated.");
            for(std::size_t i=0;i<n;i+=3) {
                Triangle t;
                for(int k=0;k<3;++k){const auto& v=g.positions.at(g.indices.empty()?i+k:g.indices.at(i+k));t[k]=object.transform*(object.visualScale*Eigen::Vector3d(v.x(),v.y(),v.z()));}
                m_triangles.push_back(t);
            }
        }
    }
    if(!m_triangles.empty())build(0,m_triangles.size());
}
int SpraySurfaceReference::build(std::size_t begin,std::size_t end) {
    Node node;node.begin=begin;node.end=end;node.lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());node.hi=-node.lo;
    for(auto i=begin;i<end;++i)for(const auto& v:m_triangles[i]){node.lo=node.lo.cwiseMin(v);node.hi=node.hi.cwiseMax(v);}
    const int index=int(m_nodes.size());m_nodes.push_back(node);
    if(end-begin>12) {
        Eigen::Index axis;(node.hi-node.lo).maxCoeff(&axis);const auto middle=(begin+end)/2;
        std::nth_element(m_triangles.begin()+begin,m_triangles.begin()+middle,m_triangles.begin()+end,[&](const auto& a,const auto& b){return a[0][axis]+a[1][axis]+a[2][axis]<b[0][axis]+b[1][axis]+b[2][axis];});
        const int left=build(begin,middle),right=build(middle,end);m_nodes[index].left=left;m_nodes[index].right=right;
    }
    return index;
}
bool SpraySurfaceReference::hit(const Eigen::Vector3d& origin,const Eigen::Vector3d& direction,double& distance) const {
    distance=std::numeric_limits<double>::infinity();if(m_nodes.empty())return false;
    std::vector<int> stack{0};
    while(!stack.empty()) {
        const auto node=m_nodes[stack.back()];stack.pop_back();double low=0,high=distance;
        for(int j=0;j<3 && low<=high;++j) {
            if(std::abs(direction[j])<1e-14){if(origin[j]<node.lo[j]-1e-10 || origin[j]>node.hi[j]+1e-10)high=-1;}
            else {double a=(node.lo[j]-1e-10-origin[j])/direction[j],b=(node.hi[j]+1e-10-origin[j])/direction[j];if(a>b)std::swap(a,b);low=std::max(low,a);high=std::min(high,b);}
        }
        if(low>high)continue;
        if(node.left>=0){stack.push_back(node.left);stack.push_back(node.right);continue;}
        for(auto i=node.begin;i<node.end;++i) {
            const auto& t=m_triangles[i];const Eigen::Vector3d a=t[1]-t[0],b=t[2]-t[0],h=direction.cross(b);const double determinant=a.dot(h);
            if(std::abs(determinant)<1e-14)continue;const Eigen::Vector3d s=origin-t[0];const double u=s.dot(h)/determinant;
            if(u< -1e-10 || u>1+1e-10)continue;const Eigen::Vector3d q=s.cross(a);const double v=direction.dot(q)/determinant;
            if(v< -1e-10 || u+v>1+1e-10)continue;const double d=b.dot(q)/determinant;
            if(d>1e-9 && d<distance)distance=d;
        }
    }
    return std::isfinite(distance);
}
}
