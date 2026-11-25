#pragma once
#include <Collision/CollisionGeometry.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <limits>
#include <vector>
namespace motion_planning::detail {
// A separating projection is a distance lower bound, even when it is not
// the closest-feature direction. BVH boxes and triangle projections prove
// separation only; an inconclusive result always causes temporal subdivision.
class MeshSeparation {
    using Triangle=std::array<Eigen::Vector3d,3>;
    struct Node {Eigen::Vector3d lo,hi;int left=-1,right=-1;std::size_t from=0,to=0;};
    std::vector<Triangle> m_triangles;
    std::vector<Node> m_nodes;
    double m_margin=0;
    int build(std::size_t from,std::size_t to) {
        Node node;node.from=from;node.to=to;node.lo=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());node.hi=-node.lo;
        for(auto i=from;i<to;++i)for(const auto& p:m_triangles[i]){node.lo=node.lo.cwiseMin(p);node.hi=node.hi.cwiseMax(p);}
        const int index=int(m_nodes.size());m_nodes.push_back(node);
        if(to-from>4) {
            Eigen::Index axis;(node.hi-node.lo).maxCoeff(&axis);const auto middle=(from+to)/2;
            std::nth_element(m_triangles.begin()+from,m_triangles.begin()+middle,m_triangles.begin()+to,[&](const auto& a,const auto& b){return a[0][axis]+a[1][axis]+a[2][axis]<b[0][axis]+b[1][axis]+b[2][axis];});
            const int left=build(from,middle),right=build(middle,to);m_nodes[index].left=left;m_nodes[index].right=right;
        }
        return index;
    }
public:
    explicit MeshSeparation(const collision::CollisionGeometry& geometry) {
        const auto& shape=geometry.shape();
        if(shape.type!=collision::CollisionShapeType::TriangleMesh || shape.vertices.empty())return;
        const auto n=shape.indices.empty()?shape.vertices.size():shape.indices.size();if(n%3)return;
        m_margin=std::max(0.,shape.inflationMargin);
        for(std::size_t i=0;i<n;i+=3) {
            Triangle triangle;
            for(int j=0;j<3;++j) {
                const auto index=shape.indices.empty()?i+j:shape.indices[i+j];
                if(index>=shape.vertices.size()){m_triangles.clear();return;}
                triangle[j]=shape.vertices[index];if(!triangle[j].allFinite()){m_triangles.clear();return;}
            }
            m_triangles.push_back(triangle);
        }
        if(!m_triangles.empty())build(0,m_triangles.size());
    }
    bool valid() const {return !m_nodes.empty();}
    bool separated(const MeshSeparation& other,const Eigen::Isometry3d& firstFromSecond,double distance) const {
        if(!valid() || !other.valid())return false;
        distance+=m_margin+other.m_margin+1e-8;
        struct Box {Eigen::Vector3d lo,hi;};
        std::vector<Box> boxes(other.m_nodes.size());std::vector<bool> ready(other.m_nodes.size(),false);
        const Eigen::Matrix3d absolute=firstFromSecond.linear().cwiseAbs();
        const auto boxAt=[&](int i)->const Box& {
            if(!ready[i]) {
                const auto& n=other.m_nodes[i];const Eigen::Vector3d center=firstFromSecond*(.5*(n.lo+n.hi));
                const Eigen::Vector3d extent=absolute*(.5*(n.hi-n.lo));boxes[i]={center-extent,center+extent};ready[i]=true;
            }
            return boxes[i];
        };
        const auto apart=[&](const Triangle& a,const Triangle& b) {
            const auto projection=[&](const Eigen::Vector3d& axis) {
                const double norm=axis.norm();if(norm<1e-18)return false;
                double alo=axis.dot(a[0]),ahi=alo,blo=axis.dot(b[0]),bhi=blo;
                for(int i=1;i<3;++i){const double x=axis.dot(a[i]),y=axis.dot(b[i]);alo=std::min(alo,x);ahi=std::max(ahi,x);blo=std::min(blo,y);bhi=std::max(bhi,y);}
                return std::max(alo-bhi,blo-ahi)>distance*norm;
            };
            const Eigen::Vector3d na=(a[1]-a[0]).cross(a[2]-a[0]),nb=(b[1]-b[0]).cross(b[2]-b[0]);
            if(projection(na) || projection(nb))return true;
            for(int i=0;i<3;++i) {
                const Eigen::Vector3d ea=a[(i+1)%3]-a[i],eb=b[(i+1)%3]-b[i];
                if(projection(na.cross(ea)) || projection(nb.cross(eb)))return true;
                for(int j=0;j<3;++j)if(projection(ea.cross(b[(j+1)%3]-b[j])))return true;
            }
            return false;
        };
        std::vector<std::pair<int,int>> pending{{0,0}};std::size_t visits=0;
        while(!pending.empty()) {
            if(++visits>1000000)return false;
            const auto pair=pending.back();pending.pop_back();const auto& a=m_nodes[pair.first];const auto& b=other.m_nodes[pair.second];const auto& box=boxAt(pair.second);
            const Eigen::Vector3d gap=(a.lo-box.hi).cwiseMax(box.lo-a.hi).cwiseMax(0.);if(gap.norm()>distance)continue;
            if(a.left<0 && b.left<0) {
                for(auto i=a.from;i<a.to;++i)for(auto j=b.from;j<b.to;++j) {
                    Triangle triangle;for(int k=0;k<3;++k)triangle[k]=firstFromSecond*other.m_triangles[j][k];
                    if(!apart(m_triangles[i],triangle))return false;
                }
            } else if(b.left<0 || (a.left>=0 && (a.hi-a.lo).squaredNorm()>(box.hi-box.lo).squaredNorm())) {
                pending.emplace_back(a.left,pair.second);pending.emplace_back(a.right,pair.second);
            } else {pending.emplace_back(pair.first,b.left);pending.emplace_back(pair.first,b.right);}
        }
        return true;
    }
};


inline Eigen::Matrix<double,6,1> moveLChartHessian(const Eigen::Matrix<double,6,6>& jac,
    const Eigen::Matrix3d& rotation,double reach,int ancestor,int descendant) {
    Eigen::Matrix<double,6,1> h;
    h.head<3>()=jac.bottomRows<3>().col(ancestor).cross(jac.topRows<3>().col(descendant))/reach;
    h.tail<3>()=.5*rotation.transpose()*jac.bottomRows<3>().col(ancestor).cross(jac.bottomRows<3>().col(descendant));
    return h;
}
}
