#pragma once
/** @file SurfaceReferenceFront.h
 * @brief 自有 CDT 上的参考前沿生成：独立尺寸来源、UV 度量及物理圆修正。
 * @date 2026-10-06
 * @modified 2026-10-07
 */
#include "SurfaceFrontalDelaunay.h"
#include "SurfaceChart.h"

namespace mesher {
// mesh 是边界传播/所在片插值得到的尺寸；background 是固定背景场尺寸。
struct SurfaceReferenceSize { double mesh=0,background=0; };
struct SurfaceReferenceOptions {
    bool extendBoundarySize=true;
    bool correctOnSurface=true;
    // 产品对照项：局部物理切图圆判定；false 保持参考 UV 度量空腔。
    bool physicalCavity=false;
    // 产品对照项：用物理切图限制偏心点高度；要求启用曲面圆修正。
    bool physicalPointHeight=false;
};
struct SurfaceReferenceCandidate {
    Point2D uv{};Point3D point{};
    SurfaceMetric metric;
    double target=0,height=0;
    bool circleCorrected=false,circleFallback=false;
};
struct SurfaceReferenceStatistics : SurfaceFrontalStatistics {
    int circleCorrections=0,circleFallbacks=0;
    int remainingLargeTriangles=0; // 有活动片拒绝后结束不等于尺寸全部满足。
    int collapsedPoleTriangles=0; // UV 桥接片，共享物理节点使其没有物理面积。
};

/** 从初始域内 CDT 的最短物理邻边建立节点尺寸，再应用半边界段长下限。
 * limitAt 非空时提供产品尺寸上界，须为正数且不可NaN；允许正无穷。
 * fixedSizes 非空时覆盖指定有效种子的最短邻边尺寸，随后仍应用边界下限/产品上界。
 * 固定来源须为有限正数、索引有效且不重复；它由调用方提供，默认参考行为不变。
 * 未引用的super槽为0；失败不覆盖output。初始两种尺寸来源相同，后续
 * 新点的 mesh 来源插值、background 来源由独立查询提供。
 */
bool initializeSurfaceReferenceSizes(const Mesh2D&,const std::vector<int>& global,
    const std::vector<Point3D>& xyz,const SurfaceDomainState&,
    const std::vector<std::pair<int,int>>& constraints,std::vector<SurfaceReferenceSize>& output,
    std::string* error,const std::function<double(Point2D,Point3D)>& limitAt={},
    const std::vector<std::pair<int,double>>& fixedSizes={});

/** 初始周期图表中跨域连接了同一物理节点时，按该内部边的UV中点建立种子。
 * 受保护极点边不拆；只处理显式共享身份，不以距离容差合并节点。
 * 插入仍用自有CDT/域事务，预算、取消和无物理展开明确失败。
 */
bool seedSurfaceReferenceChart(Mesh2D&,std::vector<int>& global,const std::vector<Point3D>& xyz,
    SurfaceDomainState&,const std::vector<std::pair<int,int>>& constraints,const GeometrySource&,int face,
    const SurfaceCriteria&,const std::function<int(Point3D)>& allocate,int& work,int& inserted,
    std::string* error,MeshingProgress={},MeshObservation={},const std::function<size_t()>& activeVertexCount={});

/** 根据活动边构造参考点，edge 对应 Mesh2D 的 (v[e],v[(e+1)%3])。
 * 度量取 UV 重心；点位使用端点均值和度量外心。可尝试物理圆/曲面求交；
 * 有限、非奇异初始候选在求交不收敛时保留，供后续裁剪/拓扑验收判断。
 * 不进行尺寸曲率换算，也不代替 CAD 偏差或域验收。
 */
bool surfaceReferenceCandidate(const GeometrySource&,int face,
    const std::array<Point2D,3>& uv,const std::array<Point3D,3>& xyz,
    const std::array<SurfaceReferenceSize,3>& sizes,int edge,
    SurfaceReferenceOptions,SurfaceReferenceCandidate&,std::string* error);

/** 从已有边界 CDT 生成点；原 UV、约束、域及自有几何谓词保持。
 * initialSizes 与 mesh.points 同长度，未引用的 super 槽可以为0；geometry、
 * backgroundAt 在运行期间不变，allocate 与 xyz 共用调用方的点池。
 * criteria 仅提供预算；尺寸由显式 sources 提供，不隐含旧曲率/硬细化策略。
 * maximumEdgeAt 非空时是产品独立上界；将超长片保留在前沿调度，默认无此项。
 * 候选插点拒绝完整回滚。终止错误/取消保留此前已接受的点，不公布完整结果。
 * 正常退出只证明生成步骤结束，remainingLargeTriangles 和最终产品验收另行处理。
 */
bool generateSurfaceReferenceFront(Mesh2D&,std::vector<int>& global,
    const std::vector<Point3D>& xyz,SurfaceDomainState&,
    const std::vector<std::pair<int,int>>& constraints,const GeometrySource&,int face,
    const SurfaceCriteria&,const std::vector<SurfaceReferenceSize>& initialSizes,
    const std::function<double(Point2D,Point3D)>& backgroundAt,const std::function<int(Point3D)>& allocate,
    int& work,SurfaceReferenceStatistics&,std::string* error,
    SurfaceReferenceOptions={},MeshingProgress={},MeshObservation={},
    const std::function<size_t()>& activeVertexCount={},
    const std::function<double(Point2D,Point3D)>& maximumEdgeAt={});
}
