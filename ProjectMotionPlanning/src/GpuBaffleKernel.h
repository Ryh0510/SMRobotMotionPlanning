#pragma once
namespace motion_planning { namespace detail {
inline const char* gpuBaffleKernel=R"CUDA(
struct V3 { double x,y,z; };
struct V2 { double x,y; };
__device__ V3 add(V3 a,V3 b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
__device__ V3 sub(V3 a,V3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
__device__ V3 mul(V3 a,double s){return {a.x*s,a.y*s,a.z*s};}
__device__ double dot(V3 a,V3 b){return a.x*b.x+a.y*b.y+a.z*b.z;}
__device__ V3 v3(const double* p){return {p[0],p[1],p[2]};}
__device__ V2 sub2(V2 a,V2 b){return {a.x-b.x,a.y-b.y};}
__device__ double cross2(V2 a,V2 b){return a.x*b.y-a.y*b.x;}
__device__ double seg2(V2 p,V2 a,V2 b,double eps2){
    V2 e=sub2(b,a),r=sub2(p,a);double len=e.x*e.x+e.y*e.y;
    double t=len>eps2?fmin(1.,fmax(0.,(r.x*e.x+r.y*e.y)/len)):0.;
    double x=r.x-t*e.x,y=r.y-t*e.y;return x*x+y*y;
}
__device__ bool ray(V2 p,V2 a,V2 b){return ((a.y>p.y)!=(b.y>p.y)) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x;}
__device__ bool inside(V2 p,const double* edges,int begin,int count){
    bool hit=false;for(int i=0;i<count;++i){const double* e=edges+4*(begin+i);if(ray(p,{e[2],e[3]},{e[0],e[1]}))hit=!hit;}return hit;
}
__device__ bool inPoly(V2 p,const V2* a,int n){bool hit=false;for(int i=0,j=n-1;i<n;j=i++)if(ray(p,a[i],a[j]))hit=!hit;return hit;}
__device__ bool intersect(V2 a,V2 b,V2 c,V2 d,double margin){
    double u=cross2(sub2(b,a),sub2(c,a)),v=cross2(sub2(b,a),sub2(d,a));
    double w=cross2(sub2(d,c),sub2(a,c)),z=cross2(sub2(d,c),sub2(b,c));
    if(((u>1e-9&&v<-1e-9)||(u<-1e-9&&v>1e-9))&&((w>1e-9&&z<-1e-9)||(w<-1e-9&&z>1e-9)))return true;
    double dist=fmin(fmin(seg2(a,c,d,1e-18),seg2(b,c,d,1e-18)),fmin(seg2(c,a,b,1e-18),seg2(d,a,b,1e-18)));
    return dist<=(margin+1e-9)*(margin+1e-9);
}
extern "C" __global__ void scoreMesh(const double* triangles,const double* edges,
    const double* taskData,const int* indices,const unsigned long long* starts,int taskCount,
    unsigned long long base,unsigned long long total,double* outside,unsigned int* hits){
    unsigned long long id=base+(unsigned long long)blockIdx.x*blockDim.x+threadIdx.x;if(id>=total)return;
    int lo=0,hi=taskCount;while(lo+1<hi){int mid=(lo+hi)/2;if(starts[mid]<=id)lo=mid;else hi=mid;}
    int task=lo;const double* g=taskData+26*task;const int* ind=indices+6*task;
    const auto relative=int(id-starts[task]);
    const double* tri=triangles+9*(ind[5]?indices[ind[4]+relative]:ind[0]+relative);
    V3 center=v3(g+12),normal=v3(g+15),axisX=v3(g+18),axisY=v3(g+21);
    V3 p[12],next[12];double low=1e300,high=-1e300;
    for(int j=0;j<3;++j){V3 a=v3(tri+3*j);p[j]={g[0]*a.x+g[1]*a.y+g[2]*a.z+g[3],g[4]*a.x+g[5]*a.y+g[6]*a.z+g[7],g[8]*a.x+g[9]*a.y+g[10]*a.z+g[11]};double z=dot(normal,sub(p[j],center));low=fmin(low,z);high=fmax(high,z);}
    double half=g[24];if(low>half+1e-12||high<-half-1e-12)return;
    int n=3;
    for(int side=0;side<2;++side){if(n==0)return;double sign=side==0?1.:-1.;int m=0;V3 a=p[n-1];double da=sign*dot(normal,sub(a,center))-half;
        for(int j=0;j<n;++j){V3 b=p[j];double db=sign*dot(normal,sub(b,center))-half;if((db<=0)!=(da<=0))next[m++]=add(a,mul(sub(b,a),da/(da-db)));if(db<=0)next[m++]=b;a=b;da=db;}
        n=m;for(int j=0;j<n;++j)p[j]=next[j];
    }
    if(n==0)return;V2 uv[12];bool hit=false;double worst=0.;
    for(int j=0;j<n;++j){V3 d=sub(p[j],center);uv[j]={dot(d,axisX),dot(d,axisY)};
        if(inside(uv[j],edges,ind[2],ind[3]))continue;hit=true;double nearest=1e300;
        for(int k=0;k<ind[3];++k){const double* e=edges+4*(ind[2]+k);nearest=fmin(nearest,seg2(uv[j],{e[0],e[1]},{e[2],e[3]},1e-24));}
        if(nearest<1e299)worst=fmax(worst,sqrt(nearest));
    }
    if(!hit)for(int k=0;k<ind[3]&&!hit;++k){const double* e=edges+4*(ind[2]+k);V2 a={e[0],e[1]},b={e[2],e[3]};
        for(int j=0;j<n;++j)if(intersect(a,b,uv[j],uv[(j+1)%n],g[25])){hit=true;break;}
        if(!hit&&n>=3&&inPoly(a,uv,n))hit=true;
    }
    if(hit)atomicAdd(hits+task,1u);
    if(worst>0)atomicMax((unsigned long long*)(outside+task),(unsigned long long)__double_as_longlong(worst));
}
)CUDA";
}}
