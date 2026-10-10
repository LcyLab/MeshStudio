#pragma once

#include "CadSurfaceMesher.h"
#include <map>

namespace mesher {

// 只在同实体且同语义区域内移除纯几何分割，保护边优先；空区域列表表示未提供额外标签，有材料或边界条件标签时调用方须传入。
struct SurfacePartitionPolicy {
    bool enabled=false;
    std::vector<int> protectedCadEdges;
    std::vector<int> faceRegions;
};

// 适配器借用不可变原 CAD；仅合并共享精确解析图表的面，无法支持或非双射图表保留原边界。
class SurfaceMeshingPartition final : public GeometrySource {
public:
    /// 检查 CAD 图表兼容性，并按策略建立合并面分区。
    SurfaceMeshingPartition(const GeometrySource& source,const SurfacePartitionPolicy& policy,MeshingProgress progress={});
    /// 返回各分区包含的原 CAD 面列表。
    const std::vector<std::vector<int>>& sourceFaces() const {return m_sourceFaces;}
    /// 返回分区内部已移除的几何分割边。
    const std::vector<int>& removedCadEdges() const {return m_removedEdges;}
    /// 读取模块保存的诊断信息。
    const std::vector<std::string>& diagnostics() const {return m_diagnostics;}
    /// 读取分区边对应的原 CAD 边编号。
    int originalEdge(int edge) const;
    /// 读取分区使用的代表 CAD 面编号。
    int representativeFace(int partition) const;
    // 查询实际原裁剪面中的点归属及原 UV。
    /// 查找分区参数点对应的原裁剪面及原 UV。
    bool supportAt(int partition,Point2D uv,SurfaceMeshResult::CadSupportSample& support) const;
    // 仅标注用本适配器成功生成的结果；多面分区不能标成单一原 CAD 面。
    /// 校验分区结果的 CAD 支撑，并补充原面标签和支撑记录。
    bool decorateResult(SurfaceMeshResult& result,MeshingProgress progress={},double maximumSupportDeviation=0) const;

    /// 返回 CAD 顶点数量。
    int vertexCount() const override;
    /// 查询指定 CAD 顶点的三维坐标。
    Point3D vertexPoint(int vertex) const override;
    /// 返回 CAD 边数量。
    int edgeCount() const override;
    /// 查询 CAD 边关联的端点编号。
    std::pair<int,int> edgeVertices(int edge) const override;
    /// 查询 CAD 边的曲线参数范围。
    std::pair<double,double> edgeRange(int edge) const override;
    /// 计算 CAD 边在指定曲线参数处的三维位置。
    Point3D edgePoint(int edge,double t) const override;
    /// 计算 CAD 边在指定参数处的一阶导数。
    Point3D edgeDerivative(int edge,double t) const override;
    /// 判断 CAD 边是否为退化边。
    bool edgeDegenerate(int edge) const override;
    /// 返回 CAD 面数量。
    int faceCount() const override;
    /// 返回面的线环、边使用关系和参数信息。
    FaceTopology faceTopology(int face) const override;
    /// 计算边上节点在指定面分支中的 UV 坐标。
    Point2D edgeUV(int face,const EdgeUse& use,double t) const override;
    /// 计算指定面在 UV 参数处的三维位置。
    Point3D facePoint(int face,Point2D uv) const override;
    /// 计算面的参数导数，供局部度量和法向计算使用。
    void faceDerivatives(int face,Point2D uv,Point3D& du,Point3D& dv) const override;
    /// 查询底层曲面的参数范围。
    bool faceParameterBounds(int face,SurfaceParameterBounds& bounds) const override;
    /// 计算指定面参数处的单位法向。
    Point3D faceNormal(int face,Point2D uv) const override;
    /// 说明法向是否已包含 CAD 面的方向约定。
    bool normalUsesOrientedDerivatives() const override;
    /// 判断 CAD 面是否相对底层曲面反向。
    bool faceReversed(int face) const override;
    /// 返回几何输入的校验错误说明。
    std::string validationError() const override;
    /// 查询几何的三维包围范围。
    bool bounds(Point3D& lo,Point3D& hi) const override;
    /// 判断输入拓扑是否要求生成闭合表面。
    bool expectsClosedSurface() const override;
    /// 查询指定 CAD 面所属的部件编号。
    int faceBodyId(int face) const override;
    /// 查询不同部件的 CAD 接触或相交关系。
    CadBodyContact bodyContact(int firstFace,int secondFace) const override;

private:
    struct AffineChart {
        double a=1,b=0,c=0,d=1;
        Point2D offset{};
        /// 将原 CAD 面 UV 转换到合并分区图表。
        Point2D toPatch(Point2D uv) const;
        /// 将分区图表 UV 转换回原 CAD 面参数。
        Point2D toSource(Point2D uv) const;
    };
    struct BoundaryUse {int sourceFace=-1;EdgeUse sourceUse;};
    struct Patch {
        int representative=-1;
        FaceTopology topology;
        SurfaceParameterBounds bounds{};
        bool bounded=false;
        std::map<int,AffineChart> charts;
        std::vector<BoundaryUse> uses;
    };
    const GeometrySource& m_source;
    std::vector<int> m_vertices,m_edges,m_originalToVertex;
    std::vector<Patch> m_patches;
    std::vector<std::vector<int>> m_sourceFaces;
    std::vector<int> m_removedEdges;
    std::vector<std::string> m_diagnostics;
    std::string m_error;
    double m_tolerance=1e-8; // 准备阶段一次计算的模型尺度残差容差。
};

/// 构造分区适配器，生成表面并补充原 CAD 支撑信息。
SurfaceMeshResult generatePartitionedCadSurfaceMesh(const GeometrySource& source,
    const SurfaceCriteria& criteria,const SurfacePartitionPolicy& policy,
    const MeshSizeField* sizeField=nullptr,MeshingProgress progress={},
    MeshingObserver* observer=nullptr,
    AssemblySurfacePolicy assemblyPolicy=AssemblySurfacePolicy::RejectBodyOverlap);

} // namespace mesher
