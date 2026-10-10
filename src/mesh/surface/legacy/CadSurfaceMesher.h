#pragma once

#include "SurfaceMesh3D.h"
#include "mesh/runtime/MeshingObserver.h"

#include <array>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace mesher {

// GeometrySource 是几何内核与网格算法之间的查询接口；实体编号从零开始且单次运行内稳定，几何查询结果必须保持不变以支持面缓存。
struct EdgeUse { int edge = -1; bool reversed = false; int branch = 0; };
struct FaceTopology { std::vector<std::vector<EdgeUse>> loops; };
struct SurfaceParameterBounds { double uMin=0,uMax=0,vMin=0,vMax=0; };
enum class CadBodyContact {None,BoundaryContact,VolumeOverlap,QueryFailed};
enum class AssemblySurfacePolicy {RejectBodyOverlap,IndependentBodies};
enum class CadTrimLocation { Unknown, Outside, Boundary, Inside };

class GeometrySource {
public:
    /// 支持通过几何查询基类释放具体适配器。
    virtual ~GeometrySource() = default;
    /// 返回 CAD 顶点数量。
    virtual int vertexCount() const = 0;
    /// 查询指定 CAD 顶点的三维坐标。
    virtual Point3D vertexPoint(int vertex) const = 0;
    /// 返回 CAD 边数量。
    virtual int edgeCount() const = 0;
    /// 查询 CAD 边关联的端点编号。
    virtual std::pair<int, int> edgeVertices(int edge) const = 0;
    /// 查询 CAD 边的曲线参数范围。
    virtual std::pair<double, double> edgeRange(int edge) const = 0;
    /// 计算 CAD 边在指定曲线参数处的三维位置。
    virtual Point3D edgePoint(int edge, double t) const = 0;
    // 可覆写精确 CAD D1；默认在 edgeRange 内用有界差分。
    virtual Point3D edgeDerivative(int edge,double t) const;
    /// 判断 CAD 边是否为退化边。
    virtual bool edgeDegenerate(int edge) const { return false; }
    /// 返回 CAD 面数量。
    virtual int faceCount() const = 0;
    /// 返回面的线环、边使用关系和参数信息。
    virtual FaceTopology faceTopology(int face) const = 0;
    /// 计算边上节点在指定面分支中的 UV 坐标。
    virtual Point2D edgeUV(int face, const EdgeUse& use, double t) const = 0;
    // 精确原生 pcurve 能力；reversed 相对全局 CAD 边的原朝向，不是 EdgeUse.branch。
    // 范围对应 CAD parBoundsOnFace（退化边可依面变化）。缺少能力返回 false，保留输出。
    // 缺失 pcurve 不暗用面投影或旧有限差分；调用方单独处理原生回退路线。
    virtual bool edgeOnFaceRange(int face,int edge,std::pair<double,double>& range) const {return false;}
    /// 计算指定面分支中边参数对应的 UV 坐标。
    virtual bool edgeOnFaceCoordinates(int face,int edge,double t,bool reversed,Point2D& uv) const {return false;}
    /// 判断指定边是否为该面的周期接缝。
    virtual bool edgeSeamOnFace(int face,int edge,bool& seam) const {return false;}
    /// 计算指定面在 UV 参数处的三维位置。
    virtual Point3D facePoint(int face, Point2D uv) const = 0;
    /// 计算面的参数导数，供局部度量和法向计算使用。
    virtual void faceDerivatives(int face,Point2D uv,Point3D& du,Point3D& dv) const;
    // 导数差分可使用的参数范围；周期分支仍保留原 UV。
    /// 查询底层曲面的参数范围。
    virtual bool faceParameterBounds(int face,SurfaceParameterBounds& bounds) const { return false; }
    // 裁剪 pcurve 的实际 UV 范围，用于原生图表尺度；与导数求值器范围分开。
    // OCC 对应 ShapeAnalysis::GetFaceUVBounds；缺少能力返回 false 并保留输出。
    virtual bool faceTrimParameterBounds(int face,SurfaceParameterBounds& bounds) const { return false; }
    // 原生支撑面投影，不按孔/裁剪分类夹紧，不替换为 closestTrimCoordinates。
    // 精确投影不可用返回 false 并保留 UV/XYZ；通用反求回退由调用方独立处理。
    virtual bool projectOnFaceSurface(int face,Point3D point,Point2D& uv,Point3D& projected) const { return false; }
    // 按 CAD 面方向定向的法向；奇异曲面点可以返回零向量。
    /// 计算指定面参数处的单位法向。
    virtual Point3D faceNormal(int face, Point2D uv) const = 0;
    // 可选的精确约定：法向由 du×dv 结合面反向标记生成，长度为零时返回零向量；未提供此约定时保留 faceNormal。
    /// 说明法向是否已包含 CAD 面的方向约定。
    virtual bool normalUsesOrientedDerivatives() const { return false; }
    // 为真时，CAD 面方向与底层 UV 曲面法向相反。
    /// 判断 CAD 面是否相对底层曲面反向。
    virtual bool faceReversed(int face) const { return false; }
    /// 返回几何输入的校验错误说明。
    virtual std::string validationError() const { return {}; }
    /// 查询几何的三维包围范围。
    virtual bool bounds(Point3D& lo,Point3D& hi) const { return false; }
    /// 判断输入拓扑是否要求生成闭合表面。
    virtual bool expectsClosedSurface() const { return false; }
    // 部件编号为 -1 表示无实体归属；严格策略接受经 CAD 证实的零体积接触，独立部件策略还保留实体间干涉。
    /// 查询指定 CAD 面所属的部件编号。
    virtual int faceBodyId(int face) const {return 0;}
    /// 查询不同部件的 CAD 接触或相交关系。
    virtual CadBodyContact bodyContact(int firstFace,int secondFace) const {return CadBodyContact::None;}
    // 可选的共同解析支撑判定包含法向方向一致性；只有 G1 接触不足以满足单图表分区约定。
    /// 判断两个面是否共享可直接使用的解析参数图表。
    virtual bool commonSurfaceChart(int firstFace,int secondFace) const {return false;}
    // 投影到底层曲面，并选择靠近初始参数的周期分支；调用方须核对重建物理点后才能接受图表。
    /// 反求三维点在指定 CAD 面上的 UV 坐标。
    virtual bool faceCoordinates(int face,Point3D point,Point2D seed,Point2D& uv) const {return false;}
    /// 查询面的 U、V 周期长度。
    virtual Point2D facePeriods(int face) const {return {0,0};}
    /// 判断 UV 点位于裁剪面的内部、边界还是外部。
    virtual CadTrimLocation faceTrimLocation(int face,Point2D uv) const {return CadTrimLocation::Unknown;}
    // 在实际裁剪面上求数值最近点；UV 必须属于内部或边界，距离相对给定三维点计算。
    /// 在实际裁剪面上求最近点及对应 UV 参数。
    virtual bool closestTrimCoordinates(int face,Point3D point,Point2D seed,Point2D& uv,double& distance) const {return false;}
};

class MeshSizeField {
public:
    /// 支持通过尺寸场基类释放具体尺寸源。
    virtual ~MeshSizeField() = default;
    // 同次生成中，同一位置的尺寸查询结果必须一致。
    /// 查询指定位置的外部尺寸场约束。
    virtual double sizeAt(const Point3D& p) const = 0;
};

enum class SurfacePointPlacement { LegacyRefinement, FrontalDelaunay, ReferenceFrontal, PhysicalReferenceFrontal, Algorithm6Experimental };
/// 返回表面候选点放置策略的标识名称。
inline const char* surfacePointPlacementName(SurfacePointPlacement placement) {
    switch(placement) {
    case SurfacePointPlacement::LegacyRefinement:return "legacy";
    case SurfacePointPlacement::FrontalDelaunay:return "frontal";
    case SurfacePointPlacement::ReferenceFrontal:return "reference";
    case SurfacePointPlacement::PhysicalReferenceFrontal:return "reference-physical";
    case SurfacePointPlacement::Algorithm6Experimental:return "algorithm6-experimental";
    }
    return "invalid";
}

// 独立算法6实验入口；不把产品的偏差/质量/工作预算映射为算法6参数。
struct Algorithm6ExperimentalSettings {
    double targetSize = 0; // 必须显式提供，模型坐标单位；尺寸场上限，不是三角边长上限。
    int extendBoundarySize = 1;
    int smoothingIterations = 1;
    int maximumNodes = 200000;
    int maximumGeometryEvaluations = 20000000;
};

struct SurfaceCriteria {
    double targetSize = 0;        // 物理长度；零表示模型对角线的 2.5%。
    double minSize = 0;           // 零表示目标尺寸的 10%。
    double maxDeviation = 0;      // 零表示目标尺寸的 10%。
    double maxNormalAngleDeg = 30;
    double maxRadiusEdgeRatio = 1.4142135623730951;
    double growthRatio = 1.3;
    int maxVertices = 200000;
    int maxIterations = 200000;
    // 可选物理边长上限；零保留原目标加 5% 的规则，显式上限仅允许 1e-8 相对比较容差。
    double maxSize = 0;
    // 有界合格片形状优化；0 用于算法对照，非产品密度参数。
    int shapeOptimizationSweeps = 4;
    bool reuseFailedQualityPasses = true; // 同一运行中的同版本失败面复用；false 用于等价性对照。
    // 新生成路线先独立对照，跨面/原模型验收完成后再决定产品默认。
    SurfacePointPlacement pointPlacement = SurfacePointPlacement::LegacyRefinement;
    Algorithm6ExperimentalSettings algorithm6;
    // 完整 CAD 与产品验收前须显式启用；语义保护边由调用方传给分区构造器，不能仅从 G1 接触推断。
    bool crossSmoothCadSplits = false;
};

struct SurfaceBoundaryEdge { int a = -1, b = -1, cadEdge = -1; };
// 记录跨部件网格相交；无接触表示未找到精确 CAD 接触，查询失败表示关系未验证，不能当作无重叠。
struct SurfaceBodyInteraction {
    int firstBody=-1,secondBody=-1,firstFace=-1,secondFace=-1;
    CadBodyContact contact=CadBodyContact::None;
    // 多面分区的原面编号为 -1；关系查询覆盖原裁剪面并集，不仅检查代表面。
    int firstPartition=-1,secondPartition=-1;
};

// 有限规模诊断快照独立于接受网格；坐标在丢弃面构造和最终节点压缩后仍有效，不代表节点身份。
struct SurfaceFacetSnapshot {
    std::array<Point2D,3> uv;
    std::array<Point3D,3> sharedPoints,surfacePoints;
    double sharedSignedArea=0,surfaceSignedArea=0; // 有向面积的两倍，单位为长度平方。
};
struct SurfaceRefinementSample {
    int face=-1,cadEdge=-1;
    int partition=-1; // 分区输出快照使用本图表 UV；包含多个原支撑面时面编号为 -1。
    double parameterStart=0,parameterEnd=0;
    SurfaceFacetSnapshot facet;
    std::vector<SurfaceFacetSnapshot> neighbors;
};

struct SurfaceWorkStatistics {
    int totalWork=0; // 共享预算包括边采样细分、请求切分、插点、换边和相交细化。
    int boundarySamplingSplits=0,sharedEdgeSplits=0;
    int intersectionRefinements=0; // 采样近似穿过其他面片后收紧的面容差。
    int insertedSurfacePoints=0,delaunayFlips=0;
    int faceBuilds=0,schedulingPasses=0;
    int violationsAfterLegalization=0;
    int allocatedVertices=0; // 节点池大小峰值；已丢弃内部槽位会被复用。
    int qualityFlips=0,qualityMoves=0; // 约束细化完成后的物理质量改善统计。
    int frontalCandidates=0,frontalInserted=0,frontalRejected=0;
    // 已选细化任务统计；同一三角形的原因可能重叠。
    int refinementEdgeViolations=0,refinementDeviationViolations=0;
    int refinementNormalViolations=0,refinementQualityViolations=0;
};

struct SurfaceMeshResult {
    bool ok = false;
    std::string error;
    SurfaceMesh3D mesh;
    std::vector<int> triangleFaceIds;
    std::vector<int> triangleBodyIds;
    AssemblySurfacePolicy assemblyPolicy=AssemblySurfacePolicy::RejectBodyOverlap;
    std::vector<SurfaceBodyInteraction> bodyInteractions;
    std::vector<std::array<Point2D, 3>> triangleCornerUV;
    // 分区 UV 独立于原 CAD 面 UV；多面分区的 CAD 面编号为 -1，旧逐角 UV 字段为 NaN。
    std::vector<int> trianglePartitionIds;
    std::vector<std::vector<int>> partitionSourceFaces;
    std::vector<std::array<Point2D,3>> trianglePartitionUV;
    struct CadSupportSample {
        int face=-1;Point2D uv{};
        double distance=0; // 物理面片采样点到记录的实际 CAD 支撑的距离。
        bool projectedToTrim=false; // 裁剪近似是否需要最近点查询。
    };
    // 记录角点、边中点和重心的已验证裁剪归属；这些是点支撑记录，不代表对整个面片的精确裁剪。
    std::vector<std::array<CadSupportSample,7>> triangleCadSupportSamples;
    std::vector<SurfaceBoundaryEdge> boundaryEdges;
    std::vector<std::string> diagnostics;
    std::vector<SurfaceRefinementSample> refinementSamples; // 最多保存前八个边界反求或受阻细分快照。
    SurfaceWorkStatistics work;
    double measuredMaxDeviation = 0;
    double measuredMaxNormalAngleDeg = 0;
    double measuredMaxEdgeLength = 0;
    double measuredWorstRadiusEdgeRatio = 0;
};

// 返回假表示取消；计数描述已完成的工作单元。
using MeshingProgress = std::function<bool(int completed, int total)>;

// observer 可空、只借用；统计事件不改变输出，取消仍由 progress 控制。
// 独立部件策略保留跨实体相交并记录 CAD 关系，同体自相交仍失败；装配策略不生成共形接触界面，也不证明多区域体边界可用。
SurfaceMeshResult generateCadSurfaceMesh(const GeometrySource& geometry,
                                         const SurfaceCriteria& criteria,
                                         const MeshSizeField* sizeField = nullptr,
                                         MeshingProgress progress = {},
                                         MeshingObserver* observer = nullptr,
                                         AssemblySurfacePolicy assemblyPolicy = AssemblySurfacePolicy::RejectBodyOverlap);

} // namespace mesher
