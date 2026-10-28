#pragma once
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <random>
#include <vector>
#include <string>
#include <cmath>

namespace motion_planning {
// Only the failing constraint's own measurements may rank an infeasible state.
inline bool isVirtualBaffleFailure(const std::string& reason) {
    return reason.rfind("virtual_baffle",0)==0;
}
inline double pointSearchRank(bool valid,const std::string& reason,double distance) {
    if(valid)return 2.0+(std::isfinite(distance)?std::clamp(distance,0.0,1.0):0.0);
    if(isVirtualBaffleFailure(reason) || reason=="state_in_collision")return -1.0;
    // Noncollision failures have no measured gradient in this search space.
    return -2.0;
}
class PointSearchSchedule {
public:
    explicit PointSearchSchedule(int first,int previous=0):active(first){(void)previous;}
    int begin() const { return active; }
    void feedback(bool improved) { (void)improved; }
private:
    const int active;

};
// Normalized rotation vector (unit ball) and distance ([-1,1]).
// No fitness is invented here: the caller supplies full IK/collision results.
template<int D=4> class BasicPointSearchPopulation {
public:
    using X = Eigen::Matrix<double,D,1>;
    using Matrix = Eigen::Matrix<double,D,D>;
    static constexpr int populationSize() { return N; }
    struct Fitness {
        double rank=-2, depth=0; bool hasDepth=false;
        bool better(const Fitness& b) const {
            if(rank!=b.rank)return rank>b.rank;
            if(hasDepth!=b.hasDepth)return hasDepth; // Prefer measured evidence, never a fabricated depth.
            return hasDepth && b.hasDepth && depth<b.depth;
        }
    };
    BasicPointSearchPopulation(unsigned seed,const X& start):rng(seed),mean(repair(start)) {
        for(int i=0;i<N;++i) {
            X x=i==0?mean:random();
            population.push_back(x);fitness.push_back(Fitness{});
        }
    }
    static X repair(X x) {
        const double n=x.template head<3>().norm();if(n>1)x.template head<3>()/=n;
        for(int j=3;j<D;++j)x[j]=std::clamp(x[j],-1.0,1.0);
        if constexpr(D==7){const double t=x.template tail<3>().norm();if(t>1)x.template tail<3>()/=t;}
        return x;
    }
    // Axial spray rotation is periodic; distance/translation genes remain bounded.
    void setPeriodicAxis(int axis) { periodicAxis=axis>=3 && axis<D?axis:-1; }
    // Opt-in steady-state GA: guided individuals are already evaluated, so do
    // not spend another population's worth of IK replaying them before evolving.
    // Other users (including DE/CMA-ES) keep their existing initialization.
    void useGuidedGeneticEvolution(const Eigen::Vector3d& axis,bool swingOnly=true) {
        steadyGenetic=true;cursor=N;rotationAxis=swingOnly?axis.normalized().eval():Eigen::Vector3d::Zero();
        for(auto& x:population)x=repairPeriodic(x);
    }
    void guide(const X& x,Fitness f) {
        if(steadyGenetic){insertGenetic(repairPeriodic(x),f);return;}
        int worst=0;for(int i=1;i<N;++i)if(fitness[worst].better(fitness[i]))worst=i;
        if(f.better(fitness[worst])){population[worst]=repairPeriodic(x);fitness[worst]=f;}
    }
    X ask(int strategy) {
        mode=strategy;target=cursor%N;
        if(cursor<N)candidate=population[target];
        else if(mode==1) { // Elitist real-coded GA, tournament selection.
            int a=tournament(),b=tournament();double t=uniform();
            candidate=t*population[a]+(1-t)*population[b];
            if(periodicAxis>=0)candidate[periodicAxis]=population[a][periodicAxis]+(1-t)*
                std::remainder(population[b][periodicAxis]-population[a][periodicAxis],2.);
            // Fresh immigrants keep exploration alive on Boolean collision
            // plateaus. Otherwise mix small refinement with occasional broad
            // mutation; pure elite clones spend no useful IK work.
            if(uniform()<.15)candidate=random();
            else {const double scale=uniform()<.25?.3:.08;for(int j=0;j<D;++j)candidate[j]+=scale*normal(rng);}
        } else if(mode==2) { // DE/rand/1/bin; distinct donors and forced crossover.
            int a,b,c;do{a=index();}while(a==target);
            do{b=index();}while(b==target||b==a);
            do{c=index();}while(c==target||c==a||c==b);
            X donor=population[a]+.7*(population[b]-population[c]);
            candidate=population[target];int forced=index()%D;
            for(int j=0;j<D;++j)if(j==forced||uniform()<.9)candidate[j]=donor[j];
        } else { // Full covariance CMA-ES sampling.
            Eigen::SelfAdjointEigenSolver<Matrix> eig(C);
            Matrix B=eig.eigenvectors();
            X z;for(int j=0;j<D;++j)z[j]=normal(rng);
            candidate=mean+sigma*B*eig.eigenvalues().cwiseMax(1e-12).cwiseSqrt().asDiagonal()*z;
        }
        candidate=repairPeriodic(candidate);return candidate;
    }
    void tell(const X& evaluated,Fitness f) { candidate=repairPeriodic(evaluated);tell(f); }
    void tell(Fitness f) {
        if(mode==1 && steadyGenetic)insertGenetic(candidate,f);
        else if(mode==3 && cursor>=N) {
            offspring.push_back(candidate);scores.push_back(f);
            if(offspring.size()==N)adapt();
        } else if(cursor<N || f.better(fitness[target])) {
            population[target]=candidate;fitness[target]=f;
        }
        ++cursor;
        if(cursor==N) {
            int best=0;for(int i=1;i<N;++i)if(fitness[i].better(fitness[best]))best=i;
            mean=population[best];
        }
    }
private:
    static constexpr int N=24,mu=12;
    std::mt19937 rng;std::normal_distribution<double> normal{0,1};
    std::vector<X> population,offspring;std::vector<Fitness> fitness,scores;
    X mean,candidate,pc=X::Zero(),ps=X::Zero();
    Matrix C=Matrix::Identity();
    double sigma=.35;int cursor=0,target=0,mode=1,generation=0,periodicAxis=-1;
    bool steadyGenetic=false;
    Eigen::Vector3d rotationAxis=Eigen::Vector3d::Zero();
    X repairPeriodic(X x) const {
        if(steadyGenetic)x.template head<3>()-=rotationAxis.dot(x.template head<3>())*rotationAxis;
        if(periodicAxis>=0)x[periodicAxis]=std::remainder(x[periodicAxis],2.);
        return repair(x);
    }
    void insertGenetic(const X& x,Fitness f) {
        int worst=0,closest=0;double nearest=1e100;
        for(int i=0;i<N;++i){
            if(fitness[worst].better(fitness[i]))worst=i;
            X d=x-population[i];if(periodicAxis>=0)d[periodicAxis]=std::remainder(d[periodicAxis],2.);
            if(d.squaredNorm()<nearest){nearest=d.squaredNorm();closest=i;}
        }
        // Crowding preserves separate posture/spin basins instead of filling
        // the entire population with almost-identical elites from one opening.
        const int slot=nearest<.04*.04?closest:worst;
        if(f.better(fitness[slot])){population[slot]=x;fitness[slot]=f;}
    }
    double uniform(){return std::generate_canonical<double,53>(rng);}
    int index(){return std::uniform_int_distribution<int>(0,N-1)(rng);}
    X random(){X x;for(int j=0;j<D;++j)x[j]=normal(rng);x.template head<3>().normalize();x.template head<3>()*=std::cbrt(uniform());for(int j=3;j<D;++j)x[j]=2*uniform()-1;return x;}
    int tournament(){int a=index();for(int i=0;i<2;++i){int b=index();if(fitness[b].better(fitness[a]))a=b;}return a;}
    void adapt() {
        std::array<int,N> order;for(int i=0;i<N;++i)order[i]=i;
        std::stable_sort(order.begin(),order.end(),[&](int a,int b){return scores[a].better(scores[b]);});
        double sum=0,sq=0;std::array<double,mu> w;
        for(int i=0;i<mu;++i){w[i]=std::log(mu+.5)-std::log(i+1.);sum+=w[i];}
        X next=X::Zero();for(int i=0;i<mu;++i){w[i]/=sum;sq+=w[i]*w[i];next+=w[i]*offspring[order[i]];}
        double eff=1/sq,cs=(eff+2)/(D+eff+5),cc=(4+eff/D)/(D+4+2*eff/D);
        double c1=2/(std::pow(D+1.3,2)+eff),cm=std::min(1-c1,2*(eff-2+1/eff)/(std::pow(D+2.,2)+eff));
        Eigen::SelfAdjointEigenSolver<Matrix> eig(C);
        auto B=eig.eigenvectors().eval();
        Matrix inv=B*eig.eigenvalues().cwiseMax(1e-12).cwiseSqrt().cwiseInverse().asDiagonal()*B.transpose();
        X y=(next-mean)/sigma;
        ps=(1-cs)*ps+std::sqrt(cs*(2-cs)*eff)*inv*y;
        double chi=std::sqrt(double(D))*(1-1./(4*D)+1./(21*D*D));
        bool h=ps.norm()/std::sqrt(1-std::pow(1-cs,2*(generation+1)))<(1.4+2./(D+1))*chi;
        pc=(1-cc)*pc+(h?1:0)*std::sqrt(cc*(2-cc)*eff)*y;
        Matrix rankMu=Matrix::Zero();
        for(int i=0;i<mu;++i){X d=(offspring[order[i]]-mean)/sigma;rankMu+=w[i]*d*d.transpose();}
        C=(1-c1-cm+(h?0:c1*cc*(2-cc)))*C+c1*pc*pc.transpose()+cm*rankMu;
        C=(.5*(C+C.transpose())).eval();C.diagonal().array()+=1e-12;
        sigma=std::clamp(sigma*std::exp(cs/(1+2*std::max(0.,std::sqrt((eff-1)/(D+1))-1)+cs)*(ps.norm()/chi-1)),.01,1.0);
        mean=repair(next);++generation;offspring.clear();scores.clear();
    }
};
using PointSearchPopulation=BasicPointSearchPopulation<4>;
using TransitionSearchPopulation=BasicPointSearchPopulation<7>;
}
