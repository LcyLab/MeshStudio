#include "mesh/surface/legacy/SurfaceQualityPolicy.h"
#include <iostream>
#include <cmath>
#include <limits>
using namespace mesher;
int main() {
    SurfaceQualityPolicy policy(.01,1.4142135623730951);
    policy.protectCorner(0,{0,0,0},5);
    bool ok=policy.decision({1,2,3},.025)==SurfaceQualityReason::Eligible &&
        policy.decision({0,1,2},.025)==SurfaceQualityReason::ProtectedCorner &&
        policy.decision({1,2,3},.002)==SurfaceQualityReason::MinimumSize;
    // 达到 radius/edge 底线后，仍须能区分匀称片；尺度不改变形状指标。
    const double eq=surfaceShapeQuality({0,0,0},{1,0,0},{.5,std::sqrt(3.)/2,0});
    const double right=surfaceShapeQuality({0,0,0},{1,0,0},{0,1,0});
    ok=ok && std::abs(eq-1)<1e-12 && right<eq && right>.8 &&
        surfaceShapeQuality({0,0,0},{1000,0,0},{0,1000,0})==right;
    SurfaceShapeScore before,better,worse;
    before.add(.9,.75);before.add(.8,.85);
    better.add(.88,.8);better.add(.78,.86);
    worse.add(.89,.7);worse.add(.79,.99);
    ok=ok && better.improves(before) && !before.improves(before) && !worse.improves(before);
    policy.protectCorner(4,{1,1,0},90);
    ok=ok && policy.decision({4,1,2},.1)==SurfaceQualityReason::Eligible;
    // 已有短边不等于新增边必然低于 floor；保留 base=.008 可用 >=.01 的两腰改善。
    ok=ok && policy.decision({1,2,3},.008)==SurfaceQualityReason::Eligible;
    ok=ok && policy.decision({1,2,3},.002,.001)==SurfaceQualityReason::Eligible &&
        policy.decision({0,1,2},.002,.001)==SurfaceQualityReason::ProtectedCorner &&
        policy.decision({1,2,3},.002,.1)==SurfaceQualityReason::MinimumSize;
    // 旧边约 .233 可以保留；这次新建的三个边均大于 .258。
    ok=ok && qualityBoundarySplitAllowed({},{1.48125,0,0},{.13489817340094107,.1895616229920316,0},
        {.740625,0,0},.258,std::sqrt(2.));
    for(double scale:{1e-6,1.,1e6}){
        // 实际 phone 窄面：细侧 .4296875、粗侧 1.71875、宽 .5；切粗侧的中点对齐细侧端点。
        ok=ok && narrowBoundaryAlignmentTrialAllowed({},{.4296875*scale,0,0},{.859375*scale,.5*scale,0},
            {-.859375*scale,.5*scale,0},{0,.5*scale,0},.39*scale,std::sqrt(2.));
        ok=ok && !narrowBoundaryAlignmentTrialAllowed({},{.4296875*scale,0,0},{.859375*scale,.5*scale,0},
            {2.578125*scale,.5*scale,0},{1.71875*scale,.5*scale,0},.39*scale,std::sqrt(2.));
        ok=ok && narrowBoundarySplitTrialAllowed({},{2*scale,0,0},{2*scale,.5*scale,0},{0,.5*scale,0},
            {scale,0,0},.39*scale,std::sqrt(2.));
        ok=ok && !narrowBoundarySplitTrialAllowed({},{2*scale,0,0},{2*scale,.5*scale,0},{2*scale,2*scale,0},
            {scale,0,0},.39*scale,std::sqrt(2.));
        ok=ok && !narrowBoundarySplitTrialAllowed({},{2*scale,0,0},{2*scale,.3*scale,0},{0,.3*scale,0},
            {scale,0,0},.39*scale,std::sqrt(2.));
        const Point3D a{},b{24*scale,0,0},p{5*scale,6*scale,0},mid{12*scale,0,0};
        ok=ok && qualityBoundarySplitAllowed(a,b,p,mid,4.3*scale,std::sqrt(2.));
        ok=ok && !qualityBoundarySplitAllowed(a,b,p,{2*scale,0,0},4.3*scale,std::sqrt(2.));
        ok=ok && !qualityBoundarySplitAllowed(a,b,p,{12*scale,-100*scale,0},4.3*scale,std::sqrt(2.));
        ok=ok && !qualityBoundarySplitAllowed(a,b,{12*scale,20*scale,0},mid,4.3*scale,std::sqrt(2.));
    }
    ok=ok && !qualityBoundarySplitAllowed({},{24,0,0},{.1,.1,0},{12,0,0},4.3,std::sqrt(2.)) &&
        !qualityBoundarySplitAllowed({},{24,0,0},{5,6,0},{12,0,0},std::numeric_limits<double>::quiet_NaN(),std::sqrt(2.));
    // 实际凹球裁剪弧：即使总和改善，一个子片恶化仍拒绝；不能据此宣称残差不可改善。
    ok=ok && !qualityBoundarySplitAllowed({50,167.60058590697537,-468.41225816970092},
        {50,144.41480352637041,-476.07180605706935},{44.244175905591312,149.59741371140638,-475.04006853032246},
        {50,156.054695473319,-472.38430543438847},4.330127019788219,std::sqrt(2.));
    std::cout<<(ok?"PASS":"FAIL")<<" explicit sharp corners and regular insertion floor\n";return ok?0:1;
}
