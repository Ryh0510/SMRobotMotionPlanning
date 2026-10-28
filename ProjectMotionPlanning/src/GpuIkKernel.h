#pragma once
namespace motion_planning { namespace detail {
inline const char* gpuIkKernel=R"CUDA(
__device__ V3 cross3(V3 a,V3 b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
__device__ V3 rot3(const double* a,V3 p){return {a[0]*p.x+a[1]*p.y+a[2]*p.z,a[4]*p.x+a[5]*p.y+a[6]*p.z,a[8]*p.x+a[9]*p.y+a[10]*p.z};}
__device__ V3 point3(const double* a,V3 p){return add(rot3(a,p),{a[3],a[7],a[11]});}
__device__ void identity3(double* a){for(int i=0;i<12;++i)a[i]=(i==0||i==5||i==10)?1.:0.;}
__device__ void product3(const double* a,const double* b,double* out){double c[12];for(int i=0;i<3;++i){for(int j=0;j<3;++j)c[4*i+j]=a[4*i]*b[j]+a[4*i+1]*b[4+j]+a[4*i+2]*b[8+j];c[4*i+3]=a[4*i]*b[3]+a[4*i+1]*b[7]+a[4*i+2]*b[11]+a[4*i+3];}for(int i=0;i<12;++i)out[i]=c[i];}
__device__ void revolute3(V3 w,V3 p,double q,double* a){
    double s=sin(q),c=cos(q),v=1-c;
    a[0]=c+w.x*w.x*v;a[1]=w.x*w.y*v-w.z*s;a[2]=w.x*w.z*v+w.y*s;
    a[4]=w.y*w.x*v+w.z*s;a[5]=c+w.y*w.y*v;a[6]=w.y*w.z*v-w.x*s;
    a[8]=w.z*w.x*v-w.y*s;a[9]=w.z*w.y*v+w.x*s;a[10]=c+w.z*w.z*v;
    V3 t=sub(p,rot3(a,p));a[3]=t.x;a[7]=t.y;a[11]=t.z;
}
__device__ void serialFk(const double* model,const double* q,double* pose,double* jac){
    double t[12];identity3(t);V3 axes[6],pivots[6];
    for(int i=0;i<6;++i){V3 w=v3(model+6*i),p=v3(model+6*i+3);axes[i]=rot3(t,w);pivots[i]=point3(t,p);double a[12];revolute3(w,p,q[i],a);product3(t,a,t);}
    product3(t,model+36,pose);
    if(jac){V3 end={pose[3],pose[7],pose[11]};for(int i=0;i<6;++i){V3 v=cross3(axes[i],sub(end,pivots[i]));jac[i]=v.x;jac[6+i]=v.y;jac[12+i]=v.z;jac[18+i]=axes[i].x;jac[24+i]=axes[i].y;jac[30+i]=axes[i].z;}}
}
__device__ double poseError3(const double* current,const double* target,double* e){
    e[0]=target[3]-current[3];e[1]=target[7]-current[7];e[2]=target[11]-current[11];
    double r[9];for(int i=0;i<3;++i)for(int j=0;j<3;++j)r[3*i+j]=target[4*i]*current[4*j]+target[4*i+1]*current[4*j+1]+target[4*i+2]*current[4*j+2];
    double qw,qx,qy,qz,trace=r[0]+r[4]+r[8];
    if(trace>0){double s=2*sqrt(fmax(0.,trace+1));qw=.25*s;qx=(r[7]-r[5])/s;qy=(r[2]-r[6])/s;qz=(r[3]-r[1])/s;}
    else if(r[0]>r[4]&&r[0]>r[8]){double s=2*sqrt(fmax(0.,1+r[0]-r[4]-r[8]));qw=(r[7]-r[5])/s;qx=.25*s;qy=(r[1]+r[3])/s;qz=(r[2]+r[6])/s;}
    else if(r[4]>r[8]){double s=2*sqrt(fmax(0.,1+r[4]-r[0]-r[8]));qw=(r[2]-r[6])/s;qx=(r[1]+r[3])/s;qy=.25*s;qz=(r[5]+r[7])/s;}
    else{double s=2*sqrt(fmax(0.,1+r[8]-r[0]-r[4]));qw=(r[3]-r[1])/s;qx=(r[2]+r[6])/s;qy=(r[5]+r[7])/s;qz=.25*s;}
    if(qw<0){qw=-qw;qx=-qx;qy=-qy;qz=-qz;}double n=sqrt(qx*qx+qy*qy+qz*qz),scale=n>1e-14?2*atan2(n,qw)/n:2.;
    e[3]=scale*qx;e[4]=scale*qy;e[5]=scale*qz;double norm=0;for(int i=0;i<6;++i)norm+=e[i]*e[i];return norm;
}
__device__ bool solve6(double a[6][7],double* x){
    for(int k=0;k<6;++k){int p=k;for(int i=k+1;i<6;++i)if(fabs(a[i][k])>fabs(a[p][k]))p=i;if(fabs(a[p][k])<1e-20)return false;
        if(p!=k)for(int j=k;j<7;++j){double t=a[k][j];a[k][j]=a[p][j];a[p][j]=t;}
        for(int i=k+1;i<6;++i){double v=a[i][k]/a[k][k];for(int j=k;j<7;++j)a[i][j]-=v*a[k][j];}}
    for(int i=5;i>=0;--i){double v=a[i][6];for(int j=i+1;j<6;++j)v-=a[i][j]*x[j];x[i]=v/a[i][i];if(!isfinite(x[i]))return false;}return true;
}
extern "C" __global__ void solveSerialIk(const double* model,const double* inputs,int count,int iterations,
    double positionTolerance,double orientationTolerance,double damping,double step,double* outputs){
    int id=blockIdx.x*blockDim.x+threadIdx.x;if(id>=count)return;const double* target=inputs+18*id;double q[6];for(int i=0;i<6;++i)q[i]=target[12+i];bool converged=false;
    for(int iteration=0;iteration<=iterations;++iteration){double current[12],j[36],e[6];serialFk(model,q,current,j);double error=poseError3(current,target,e);
        double pe=e[0]*e[0]+e[1]*e[1]+e[2]*e[2],re=e[3]*e[3]+e[4]*e[4]+e[5]*e[5];
        if(pe<=positionTolerance*positionTolerance&&re<=orientationTolerance*orientationTolerance){converged=true;break;}if(iteration==iterations||!isfinite(error))break;
        double a[6][7],x[6]={},delta[6]={};for(int r=0;r<6;++r){for(int c=0;c<6;++c){a[r][c]=r==c?damping*damping:0.;for(int k=0;k<6;++k)a[r][c]+=j[6*r+k]*j[6*c+k];}a[r][6]=e[r];}
        if(!solve6(a,x))break;double maxDelta=.35;for(int k=0;k<6;++k){for(int r=0;r<6;++r)delta[k]+=j[6*r+k]*x[r];maxDelta=fmax(maxDelta,fabs(delta[k]));}
        double scale=fmin(step,.35/maxDelta);bool improved=false;
        for(int backtrack=0;backtrack<12;++backtrack){double trial[6],pose[12],err[6];for(int k=0;k<6;++k)trial[k]=q[k]+scale*delta[k];serialFk(model,trial,pose,0);
            if(poseError3(pose,target,err)<error){for(int k=0;k<6;++k)q[k]=trial[k];improved=true;break;}scale*=.5;}
        if(!improved)break;
    }
    for(int k=0;k<6;++k)outputs[7*id+k]=q[k];outputs[7*id+6]=converged?1.:0.;
}
)CUDA";
}}
