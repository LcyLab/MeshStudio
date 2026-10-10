#pragma once
/** @file SurfaceEvaluationCache.h
 * @brief 面/棱处理作用域内的原参数几何缓存；不缓存尺寸或精度判据。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include "CadSurfaceMesher.h"
#include <map>
#include <tuple>

namespace mesher {
// 按精确有限 UV 缓存单个 CAD 面，正负零视为相同；借用的几何源必须不变，边或面构造结束后丢弃缓存。
class SurfaceEvaluationCache final : public GeometrySource {
    const GeometrySource& source;
    int face;
    struct Sample { Point3D point{},normal{},du{},dv{}; bool hasPoint=false,hasNormal=false,hasD1=false; };
    mutable std::map<std::pair<double,double>,Sample> samples;
    mutable std::map<std::tuple<int,int,bool,double>,Point2D> trims;
    mutable bool hasBounds=false,bounded=false,hasReversed=false,isReversed=false;
    mutable SurfaceParameterBounds parameterBounds;
    bool derivativeNormal;
    /// 按精确 UV 键查询或创建曲面采样缓存项。
    Sample& sample(Point2D uv) const;
public:
    MeshObservation observation;
    /// 绑定一个不可变 CAD 查询源，建立单面采样缓存。
    SurfaceEvaluationCache(const GeometrySource& g,int f,MeshObservation o={});
    /// 查询绑定面的三维位置并复用采样缓存。
    Point3D point(Point2D uv) const { return facePoint(face,uv); }
    /// 查询绑定面的法向并复用采样缓存。
    Point3D normal(Point2D uv) const { return faceNormal(face,uv); }
    /// 读取绑定面的方向标记。
    bool reversed() const { return faceReversed(face); }
    /// 返回曲面采样和裁剪位置缓存的条目数量。
    size_t sampleCount() const { return samples.size()+trims.size(); }

    /// 返回 CAD 顶点数量。
    int vertexCount() const override { return source.vertexCount(); }
    /// 查询指定 CAD 顶点的三维坐标。
    Point3D vertexPoint(int v) const override { return source.vertexPoint(v); }
    /// 返回 CAD 边数量。
    int edgeCount() const override { return source.edgeCount(); }
    /// 查询 CAD 边关联的端点编号。
    std::pair<int,int> edgeVertices(int e) const override { return source.edgeVertices(e); }
    /// 查询 CAD 边的曲线参数范围。
    std::pair<double,double> edgeRange(int e) const override { return source.edgeRange(e); }
    /// 计算 CAD 边在指定曲线参数处的三维位置。
    Point3D edgePoint(int e,double t) const override { return source.edgePoint(e,t); }
    /// 计算 CAD 边在指定参数处的一阶导数。
    Point3D edgeDerivative(int e,double t) const override { return source.edgeDerivative(e,t); }
    /// 判断 CAD 边是否为退化边。
    bool edgeDegenerate(int e) const override { return source.edgeDegenerate(e); }
    /// 返回 CAD 面数量。
    int faceCount() const override { return source.faceCount(); }
    /// 返回面的线环、边使用关系和参数信息。
    FaceTopology faceTopology(int f) const override { return source.faceTopology(f); }
    /// 计算边上节点在指定面分支中的 UV 坐标。
    Point2D edgeUV(int f,const EdgeUse& use,double t) const override;
    /// 计算指定面在 UV 参数处的三维位置。
    Point3D facePoint(int f,Point2D uv) const override;
    /// 计算指定面参数处的单位法向。
    Point3D faceNormal(int f,Point2D uv) const override;
    /// 计算面的参数导数，供局部度量和法向计算使用。
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override;
    /// 查询底层曲面的参数范围。
    bool faceParameterBounds(int f,SurfaceParameterBounds& b) const override;
    /// 判断 CAD 面是否相对底层曲面反向。
    bool faceReversed(int f) const override;
    /// 说明法向是否已包含 CAD 面的方向约定。
    bool normalUsesOrientedDerivatives() const override { return derivativeNormal; }
    /// 查询几何的三维包围范围。
    bool bounds(Point3D& a,Point3D& b) const override { return source.bounds(a,b); }
    /// 判断输入拓扑是否要求生成闭合表面。
    bool expectsClosedSurface() const override { return source.expectsClosedSurface(); }
    /// 返回几何输入的校验错误说明。
    std::string validationError() const override { return source.validationError(); }
};
} // namespace mesher
