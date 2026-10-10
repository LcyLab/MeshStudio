// 文件职责：物理切平面图表：把局部三维邻域投影到切平面，构造候选后有界反求回原 CAD 参数。
#include "SurfaceTangentChart.h"
#include "SurfaceRefinementCandidates.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace mesher {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p){return std::isfinite(p.x)&&std::isfinite(p.y);}
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
/// 计算两个点的坐标差。
Point3D sub(Point3D a,Point3D b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
/// 读取统一缩放比例。
Point3D scale(Point3D a,double s){return {a.x*s,a.y*s,a.z*s};}
/// 计算向量点积。
double dot(Point3D a,Point3D b){return a.x*b.x+a.y*b.y+a.z*b.z;}
/// 计算向量的欧氏长度。
double norm(Point3D a){return std::hypot(a.x,a.y,a.z);}
/// 计算向量叉积，确定有向关系。
Point3D cross(Point3D a,Point3D b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
/// 判断 UV 点是否位于图表参数窗口内。
bool inWindow(Point2D p,const SurfaceParameterBounds& b){return p.x>=b.uMin&&p.x<=b.uMax&&p.y>=b.vMin&&p.y<=b.vMax;}
}
/// 将三维点投影到局部正交切平面坐标。
Point2D SurfaceTangentChart::project(Point3D p) const {
    auto d=sub(p,origin);return {dot(d,axisU),dot(d,axisV)};
}
/// 四边形空圆判断专用：四点排序后构造相同物理平面，换对角线不改变判据。
bool surfaceTangentQuadChartAt(const GeometrySource& geo,int face,const std::array<Point2D,4>& uv,
    const std::array<Point3D,4>& xyz,SurfaceTangentChart& chart,std::string* error) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    for(int k=0;k<4;++k)if(!finite(uv[k])||!finite(xyz[k]))return fail("non-finite physical quad source");
    std::array<int,4> order{0,1,2,3};
    std::sort(order.begin(),order.end(),[&](int a,int b){return std::tie(uv[a].x,uv[a].y,xyz[a].x,xyz[a].y,xyz[a].z)<
        std::tie(uv[b].x,uv[b].y,xyz[b].x,xyz[b].y,xyz[b].z);});
    SurfaceTangentChart result;
    for(int k:order){result.seed.x+=uv[k].x/4;result.seed.y+=uv[k].y/4;}
    result.origin=geo.facePoint(face,result.seed);result.normal=geo.faceNormal(face,result.seed);
    const double n=norm(result.normal);
    if(!finite(result.origin)||!finite(result.normal)||!(n>0)||!std::isfinite(n))return fail("undefined physical quad normal");
    result.normal=scale(result.normal,1/n);double longest=0,diameter=0;
    for(int a=0;a<4;++a)for(int b=a+1;b<4;++b) {
        auto edge=sub(xyz[order[b]],xyz[order[a]]);diameter=std::max(diameter,norm(edge));
        edge=sub(edge,scale(result.normal,dot(edge,result.normal)));const double d=norm(edge);
        if(d>longest){longest=d;result.axisU=edge;}
    }
    if(!(longest>diameter*1e-12)||!std::isfinite(longest))return fail("collapsed physical quad chart");
    result.axisU=scale(result.axisU,1/longest);result.axisV=cross(result.normal,result.axisU);
    chart=result;return true;
}
/// 建立实际三维邻域的切平面及原 UV 分支窗口。
bool surfaceTangentChartAt(const GeometrySource& geo,int face,const std::array<Point2D,3>& uv,
    const std::array<Point3D,3>& xyz,SurfaceTangentChart& chart,std::string* error){
    const auto fail=[&](const char* message){if(error)*error=message;return false;};
    SurfaceTangentChart c;
    double uMin=uv[0].x,uMax=uMin,vMin=uv[0].y,vMax=vMin,diameter=0;
    for(int i=0;i<3;++i){if(!finite(uv[i])||!finite(xyz[i]))return fail("non-finite tangent chart input");
        c.seed.x+=uv[i].x/3;c.seed.y+=uv[i].y/3;
        uMin=std::min(uMin,uv[i].x);uMax=std::max(uMax,uv[i].x);
        vMin=std::min(vMin,uv[i].y);vMax=std::max(vMax,uv[i].y);
        diameter=std::max(diameter,norm(sub(xyz[i],xyz[(i+1)%3])));
    }
    if(!(diameter>0 && uMax>uMin && vMax>vMin))return fail("degenerate tangent chart neighborhood");
    c.origin=geo.facePoint(face,c.seed);c.normal=geo.faceNormal(face,c.seed);
    const double normalLength=norm(c.normal);
    if(!finite(c.origin)||!finite(c.normal)||!(normalLength>0))return fail("undefined tangent chart normal");
    c.normal=scale(c.normal,1/normalLength);
    double longest=0;
    for(int i=0;i<3;++i){auto d=sub(xyz[(i+1)%3],xyz[i]);d=sub(d,scale(c.normal,dot(d,c.normal)));
        const double len=norm(d);if(len>longest){longest=len;c.axisU=d;}}
    if(!(longest>diameter*1e-12))return fail("collapsed tangent chart neighborhood");
    c.axisU=scale(c.axisU,1/longest);c.axisV=cross(c.normal,c.axisU);
    // 保留源片 UV 分支，并允许候选落入同一局部邻域的相邻片。
    c.window={uMin-(uMax-uMin),uMax+(uMax-uMin),vMin-(vMax-vMin),vMax+(vMax-vMin)};
    SurfaceParameterBounds b;
    if(geo.faceParameterBounds(face,b)){
        if(!std::isfinite(b.uMin)||!std::isfinite(b.uMax)||!std::isfinite(b.vMin)||!std::isfinite(b.vMax)||
            !(b.uMax>b.uMin && b.vMax>b.vMin))return fail("invalid CAD tangent chart parameter bounds");
        // 已展开的周期分支不折回基础区间；只有源片属于该区间时才裁窗口。
        if(uMin>=b.uMin && uMax<=b.uMax){c.window.uMin=std::max(c.window.uMin,b.uMin);c.window.uMax=std::min(c.window.uMax,b.uMax);}
        if(vMin>=b.vMin && vMax<=b.vMax){c.window.vMin=std::max(c.window.vMin,b.vMin);c.window.vMax=std::min(c.window.vMax,b.vMax);}
    }
    c.tolerance=std::max(diameter*1e-9,128*std::numeric_limits<double>::epsilon()*std::max(diameter,norm(c.origin)));
    if(!inWindow(c.seed,c.window)||!std::isfinite(c.tolerance)||!(c.tolerance>0))return fail("invalid tangent chart window/tolerance");
    chart=c;return true;
}
/// 有界 Newton/缩步反解切平面坐标，使用原曲面坐标与 D1 查询。
SurfaceChartInverse invertSurfaceTangentChart(const GeometrySource& geo,int face,
    const SurfaceTangentChart& chart,Point2D target){
    SurfaceChartInverse out;out.uv=chart.seed;
    const auto fail=[&](const char* reason){out.error=reason;return out;};
    if(!finite(target)||!finite(chart.seed)||!finite(chart.origin)||!finite(chart.axisU)||!finite(chart.axisV)||
        !std::isfinite(chart.tolerance)||!(chart.tolerance>0)||!inWindow(out.uv,chart.window))return fail("invalid tangent inverse input");
    for(int i=0;i<20;++i){out.iterations=i+1;out.point=geo.facePoint(face,out.uv);
        if(!finite(out.point))return fail("non-finite tangent inverse CAD point");
        const auto q=chart.project(out.point);const Point2D r{q.x-target.x,q.y-target.y};
        out.residual=std::hypot(r.x,r.y);
        if(!std::isfinite(out.residual))return fail("non-finite tangent inverse residual");
        if(out.residual<=chart.tolerance){out.valid=true;return out;}
        Point3D du,dv;geo.faceDerivatives(face,out.uv,du,dv);
        if(!finite(du)||!finite(dv))return fail("non-finite tangent inverse derivatives");
        const double a=dot(du,chart.axisU),b=dot(dv,chart.axisU),c=dot(du,chart.axisV),d=dot(dv,chart.axisV);
        const double su=std::hypot(a,c),sv=std::hypot(b,d);
        if(!(su>0 && sv>0))return fail("singular tangent inverse derivatives");
        const double det=(a/su)*(d/sv)-(b/sv)*(c/su);
        if(!std::isfinite(det)||std::abs(det)<=1e-12)return fail("singular tangent inverse Jacobian");
        const Point2D step{((d/sv)*r.x-(b/sv)*r.y)/(det*su),((a/su)*r.y-(c/su)*r.x)/(det*sv)};
        if(!finite(step))return fail("non-finite tangent inverse step");
        bool moved=false;
        for(int trial=0;trial<8;++trial){const double alpha=std::ldexp(1.,-trial);
            const Point2D candidate{out.uv.x-alpha*step.x,out.uv.y-alpha*step.y};
            if(!finite(candidate)||!inWindow(candidate,chart.window))continue;
            const auto point=geo.facePoint(face,candidate);if(!finite(point))continue;
            const auto p=chart.project(point);const double residual=std::hypot(p.x-target.x,p.y-target.y);
            const auto normal=geo.faceNormal(face,candidate);const double len=norm(normal);
            // 同侧法向和严格减小残差限制反解留在局部图表，不接受远侧解。
            if(std::isfinite(residual)&&residual<out.residual&&finite(normal)&&len>0&&dot(normal,chart.normal)>0.5*len){
                out.uv=candidate;moved=true;break;}
        }
        if(!moved)return fail("tangent inverse cannot progress in branch window");
    }
    return fail("tangent inverse iteration limit");
}
/// 最多两个物理偏心/外心候选；成功反解后仍由调用方验收有效裁剪域和三维质量。
SurfaceTangentCandidates surfaceTangentRefinementCandidates(const GeometrySource& geo,int face,
    const std::array<Point2D,3>& uv,const std::array<Point3D,3>& xyz,double target){
    SurfaceTangentCandidates result;SurfaceTangentChart chart;std::string error;
    if(!surfaceTangentChartAt(geo,face,uv,xyz,chart,&error)){result.refusals.push_back(error);return result;}
    std::array<Point2D,3> plane;for(int i=0;i<3;++i)plane[i]=chart.project(xyz[i]);
    const auto candidates=surfaceRefinementCandidates(plane,SurfaceMetric{},target);
    if(candidates.empty())result.refusals.push_back("no finite tangent plane candidate");
    for(auto p:candidates){const auto inverse=invertSurfaceTangentChart(geo,face,chart,p);
        if(inverse.valid)result.uv.push_back(inverse.uv);else result.refusals.push_back(inverse.error);}
    return result;
}
} // namespace mesher
