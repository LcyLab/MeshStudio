#pragma once
// 文件职责：CAD 查询适配：建立实体身份和面拓扑，按需准备 OCC 曲线、曲面及投影器供一次串行运行复用。

#include "mesh/surface/legacy/CadSurfaceMesher.h"

#include <TopoDS_Shape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>

#include <map>
#include <memory>
#include <optional>
#include <tuple>
#include <vector>

// 持有 BRep 及求值器，通过 GeometrySource 向纯算法提供查询，不把 OCC 类型传入算法库。
// 顶点/边/面查询使用零基索引；Algorithm 6 的一基实体身份由适配器转换。
// 求值器和投影缓存按需准备，同一实例供一次运行串行使用，不能并发共享。
class OccCadGeometry final : public mesher::GeometrySource {
public:
    struct PreparationStatistics { size_t surfaceAdaptors=0,curveAdaptors=0,pcurves=0; };
    /// 默认严格校验；允许修复时仅协调共享端点附近的微小裁剪交叉容差。
    /// 修复在深副本上，曲线/曲面/位置/拓扑不改；完整校验失败仍拒绝。
    explicit OccCadGeometry(const TopoDS_Shape& shape,bool allowLocalToleranceRepair=false);
    /// 返回经输入检查后实际使用的 BREP 模型。
    const TopoDS_Shape& acceptedShape() const {return m_shape;}
    /// 返回输入检查和局部容差处理的诊断记录。
    const std::vector<std::string>& inputDiagnostics() const {return m_inputDiagnostics;}
    /// 判断输入是否经过局部容差修正。
    bool inputWasReconciled() const {return !m_inputDiagnostics.empty();}
    /// 返回 CAD 顶点数量。
    int vertexCount() const override;
    /// 查询指定 CAD 顶点的三维坐标。
    mesher::Point3D vertexPoint(int vertex) const override;
    /// 返回 CAD 边数量。
    int edgeCount() const override;
    /// 查询 CAD 边关联的端点编号。
    std::pair<int,int> edgeVertices(int edge) const override;
    /// 查询 CAD 边的曲线参数范围。
    std::pair<double,double> edgeRange(int edge) const override;
    /// 计算 CAD 边在指定曲线参数处的三维位置。
    mesher::Point3D edgePoint(int edge,double t) const override;
    /// 计算 CAD 边在指定参数处的一阶导数。
    mesher::Point3D edgeDerivative(int edge,double t) const override;
    /// 判断 CAD 边是否为退化边。
    bool edgeDegenerate(int edge) const override;
    /// 返回 CAD 面数量。
    int faceCount() const override;
    /// 返回面的线环、边使用关系和参数信息。
    mesher::FaceTopology faceTopology(int face) const override;
    /// 计算边上节点在指定面分支中的 UV 坐标。
    mesher::Point2D edgeUV(int face,const mesher::EdgeUse& use,double t) const override;
    /// 查询边在指定面上的参数曲线范围。
    bool edgeOnFaceRange(int face,int edge,std::pair<double,double>& range) const override;
    /// 计算指定面分支中边参数对应的 UV 坐标。
    bool edgeOnFaceCoordinates(int face,int edge,double t,bool reversed,mesher::Point2D& uv) const override;
    /// 判断指定边是否为该面的周期接缝。
    bool edgeSeamOnFace(int face,int edge,bool& seam) const override;
    /// 计算指定面在 UV 参数处的三维位置。
    mesher::Point3D facePoint(int face,mesher::Point2D uv) const override;
    /// 计算面的参数导数，供局部度量和法向计算使用。
    void faceDerivatives(int face,mesher::Point2D uv,mesher::Point3D& du,mesher::Point3D& dv) const override;
    /// 复刻OCCFace::firstDer：直接用含位置变换的BRep支撑面D1，保持原生B样条舍入。
    /// 与通用BRepAdaptor求导分开；face为零基ID，uv为支撑面参数，不检查裁剪域。
    void faceSurfaceDerivatives(int face,mesher::Point2D uv,mesher::Point3D& du,mesher::Point3D& dv) const;
    /// 复刻OCCFace::point：直接支撑面Value，用于圆/曲面Newton；不检查裁剪域。
    mesher::Point3D faceSurfacePoint(int face,mesher::Point2D uv) const;
    /// 查询底层曲面的参数范围。
    bool faceParameterBounds(int face,mesher::SurfaceParameterBounds& bounds) const override;
    /// 查询裁剪面实际使用的 UV 包围范围。
    bool faceTrimParameterBounds(int face,mesher::SurfaceParameterBounds& bounds) const override;
    /// 将三维点投影到底层曲面，返回曲面参数和位置。
    bool projectOnFaceSurface(int face,mesher::Point3D point,mesher::Point2D& uv,mesher::Point3D& projected) const override;
    /// 计算指定面参数处的单位法向。
    mesher::Point3D faceNormal(int face,mesher::Point2D uv) const override;
    /// 说明法向是否已包含 CAD 面的方向约定。
    bool normalUsesOrientedDerivatives() const override { return true; }
    /// 判断 CAD 面是否相对底层曲面反向。
    bool faceReversed(int face) const override;
    /// 返回几何输入的校验错误说明。
    std::string validationError() const override;
    /// 查询几何的三维包围范围。
    bool bounds(mesher::Point3D& lo,mesher::Point3D& hi) const override;
    /// 判断输入拓扑是否要求生成闭合表面。
    bool expectsClosedSurface() const override;
    /// 查询指定 CAD 面所属的部件编号。
    int faceBodyId(int face) const override;
    /// 查询不同部件的 CAD 接触或相交关系。
    mesher::CadBodyContact bodyContact(int firstFace,int secondFace) const override;
    /// 判断两个面是否共享可直接使用的解析参数图表。
    bool commonSurfaceChart(int firstFace,int secondFace) const override;
    /// 反求三维点在指定 CAD 面上的 UV 坐标。
    bool faceCoordinates(int face,mesher::Point3D point,mesher::Point2D seed,mesher::Point2D& uv) const override;
    /// 查询面的 U、V 周期长度。
    mesher::Point2D facePeriods(int face) const override;
    /// 判断 UV 点位于裁剪面的内部、边界还是外部。
    mesher::CadTrimLocation faceTrimLocation(int face,mesher::Point2D uv) const override;
    /// 在实际裁剪面上求最近点及对应 UV 参数。
    bool closestTrimCoordinates(int face,mesher::Point3D point,mesher::Point2D seed,mesher::Point2D& uv,double& distance) const override;
    /// 一次性求值器准备量；不随重复求值增加，不包含拓扑构造的其他 OCC 调用。
    PreparationStatistics preparationStatistics() const;

private:
    TopoDS_Shape m_shape;
    TopTools_IndexedMapOfShape m_vertices;
    TopTools_IndexedMapOfShape m_edges;
    TopTools_IndexedMapOfShape m_faces;
    std::vector<mesher::FaceTopology> m_topology;
    struct EdgeGeometry {
        std::pair<int,int> vertices;
        std::pair<double,double> range;
        bool degenerate=false;
        std::optional<BRepAdaptor_Curve> curve;
    };
    struct TrimCurve { Handle(Geom2d_Curve) curve; double first=0,last=0; };
    std::vector<mesher::Point3D> m_vertexPoints;
    std::vector<EdgeGeometry> m_edgeGeometry;
    std::vector<BRepAdaptor_Surface> m_surfaces;
    mutable std::map<int,std::unique_ptr<GeomAPI_ProjectPointOnSurf>> m_surfaceProjectors; // 单实例串行，复现入口首次查询才准备。
    mutable std::map<int,Handle(Geom_Surface)> m_supportSurfaces; // 复刻入口懒准备，保留位置变换；不改变通用适配器。
    std::map<std::tuple<int,int,int>,TrimCurve> m_trimCurves;
    bool m_closed = false;
    std::string m_error;
    std::vector<std::string> m_inputDiagnostics;
    std::vector<int> m_faceBodies;
    TopTools_IndexedMapOfShape m_solids;
    mutable std::map<std::pair<int,int>,mesher::CadBodyContact> m_contactPairs;
    mutable std::map<std::pair<int,int>,mesher::CadBodyContact> m_bodyPairs;
};
