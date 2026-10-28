#pragma once
#include <algorithm>

namespace motion_planning {
// One-based offspring generations; the complete initial pool stays nominal.
// This is relative to the immutable source distance, never a previous result.
class GaRadiusEvolution {
public:
    explicit GaRadiusEvolution(bool enabled):enabled_(enabled) {}
    double beginGeneration(int generation,bool hasSafePosture) {
        if(enabled_ && !hasSafePosture) {
            const double requested=generation>=5?.8:generation==4?.5:generation==3?.3:0.;
            fraction_=std::max(fraction_,requested);
        }
        return fraction_;
    }
    double fraction() const { return fraction_; }
    double clampGene(double gene) const { return std::clamp(gene,-fraction_/.8,fraction_/.8); }
    // Fixed encoding preserves already evaluated parent fitness when bounds grow.
    static double distance(double nominal,double gene) { return nominal*(1.+.8*gene); }
private:
    bool enabled_;double fraction_=0.;
};
}
