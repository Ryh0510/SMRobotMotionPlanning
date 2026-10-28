#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace motion_planning { namespace detail {
// Exact segment-distance and even/odd ray queries. This index is for search
// feedback only; it supplies neither a signed clearance nor a safety verdict.
class BaffleRimIndex {
public:
    using Point=std::array<double,2>;
    using Rings=std::vector<std::vector<Point>>;
    explicit BaffleRimIndex(const Rings& rings) {
        for(const auto& ring:rings)for(std::size_t j=0;j<ring.size();++j)
            segments.push_back({ring[j],ring[(j+1)%ring.size()],ring.size()>=3});
        order.resize(segments.size());std::iota(order.begin(),order.end(),0);
        nodes.reserve(2*segments.size());
        if(!segments.empty())build(0,int(segments.size()));
    }
    bool contains(const Point& p) const {
        return finite(p) && !nodes.empty() && parity(0,p);
    }
    double distance(const Point& p) const {
        if(!finite(p) || nodes.empty())return std::numeric_limits<double>::infinity();
        double best=std::numeric_limits<double>::infinity();nearest(0,p,best);return std::sqrt(best);
    }
private:
    struct Segment {Point a,b;bool parity;};
    struct Node {Point lo,hi;int begin=0,end=0,left=-1,right=-1;};
    std::vector<Segment> segments;
    std::vector<int> order;
    std::vector<Node> nodes;
    static bool finite(const Point& p){return std::isfinite(p[0]) && std::isfinite(p[1]);}
    int build(int begin,int end) {
        Node n;n.begin=begin;n.end=end;
        n.lo={std::numeric_limits<double>::infinity(),std::numeric_limits<double>::infinity()};
        n.hi={-n.lo[0],-n.lo[1]};
        for(int k=begin;k<end;++k){const auto& s=segments[order[k]];for(int j=0;j<2;++j){n.lo[j]=std::min({n.lo[j],s.a[j],s.b[j]});n.hi[j]=std::max({n.hi[j],s.a[j],s.b[j]});}}
        const int id=int(nodes.size());nodes.push_back(n);
        if(end-begin>8){const int axis=n.hi[0]-n.lo[0]>=n.hi[1]-n.lo[1]?0:1,mid=(begin+end)/2;
            std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](int a,int b){
                const double x=segments[a].a[axis]+segments[a].b[axis],y=segments[b].a[axis]+segments[b].b[axis];return x==y?a<b:x<y;});
            n.left=build(begin,mid);n.right=build(mid,end);nodes[id]=n;}
        return id;
    }
    static double lowerBound(const Node& n,const Point& p){double v=0;for(int j=0;j<2;++j){const double d=std::max({n.lo[j]-p[j],0.,p[j]-n.hi[j]});v+=d*d;}return v;}
    static double segmentDistance(const Segment& s,const Point& p){
        const double x=s.b[0]-s.a[0],y=s.b[1]-s.a[1],rx=p[0]-s.a[0],ry=p[1]-s.a[1],length=x*x+y*y;
        const double t=length>1e-24?std::clamp((rx*x+ry*y)/length,0.,1.):0.;
        return (rx-t*x)*(rx-t*x)+(ry-t*y)*(ry-t*y);
    }
    void nearest(int id,const Point& p,double& best) const {
        const auto& n=nodes[id];if(lowerBound(n,p)>best+1e-24)return;
        if(n.left<0){for(int k=n.begin;k<n.end;++k)best=std::min(best,segmentDistance(segments[order[k]],p));return;}
        const bool left=lowerBound(nodes[n.left],p)<=lowerBound(nodes[n.right],p);
        nearest(left?n.left:n.right,p,best);nearest(left?n.right:n.left,p,best);
    }
    bool parity(int id,const Point& p) const {
        const auto& n=nodes[id];
        if(n.hi[1]<=p[1] || n.lo[1]>p[1] || n.hi[0]<=p[0])return false;
        if(n.left>=0)return parity(n.left,p)!=parity(n.right,p);
        bool hit=false;
        for(int k=n.begin;k<n.end;++k){const auto& s=segments[order[k]];
            // Same edge orientation/arithmetic as VirtualBaffle::apertureContains.
            const auto& a=s.b;const auto& b=s.a;
            if(s.parity && (a[1]>p[1])!=(b[1]>p[1]) && p[0]<(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0])hit=!hit;}
        return hit;
    }
};
}}
