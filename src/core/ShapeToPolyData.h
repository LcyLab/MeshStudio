#pragma once

#include <array>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <vtkSmartPointer.h>

class vtkPolyData;

/// 三角化片体的裸数据（顶点坐标 + 三角形顶点索引）。
/// 供四面体网格引擎直接消费，避免在算法侧引入 VTK 依赖。
struct TessellatedSoup
{
    std::vector<std::array<double, 3>> points;
    std::vector<std::array<int, 3>> triangles;
};

/// OCC 形状 -> VTK 多边形数据（单 VTK 视口方案的核心转换层）。
/// surfaceMesh: 所有面的三角化片体，附带 "FaceId" 单元标量便于以后拾取联动。
/// edgeLines : 所有非退化棱边的折线。
///
/// ★ 关于 B-Rep 输入的三角面片来源 ★
/// STEP / IGES / BRep 是**边界表示**，本身不携带三角面片，必须先做离散化
/// （tessellation）：本类用 `BRepMesh_IncrementalMesh` 逐面生成 `Poly_Triangulation`。
/// 注意逐面离散的相邻面在共享棱上的顶点只是**近似相等**（受弦高与 OCC 内部
/// 容差影响），因此下游 `mesher::weldSurfaceMesh` 必须做真正的容差焊接。
class ShapeToPolyData
{
public:
    static vtkSmartPointer<vtkPolyData> surfaceMesh(const TopoDS_Shape& shape);

    /// 指定弦高偏差（deflection）的版本。用于四面体化输入：离散精度必须随
    /// 目标单元尺寸收紧，否则粗四面体会挂在低精度折线上。
    static vtkSmartPointer<vtkPolyData> surfaceMesh(const TopoDS_Shape& shape,
                                                    double deflection);

    static vtkSmartPointer<vtkPolyData> edgeLines(const TopoDS_Shape& shape);

    /// 形状包围盒对角线（形状退化 / 空时返回 0）。
    static double boundingDiagonal(const TopoDS_Shape& shape);

    /// 按目标单元尺寸推离散弦高：
    ///   targetSize > 0  ->  min(diag · 1e-3, targetSize · 0.25)
    ///   targetSize <= 0 ->  diag · 1e-3（与历史固定值一致）
    /// 上界取 diag·1e-3 意味着**默认设置下离散精度与从前完全一致（不退步）**；
    /// 用户把目标尺寸调得比对角线的 0.4% 更细时，弦高同步收紧到目标边长的 1/4。
    static double deflectionFor(const TopoDS_Shape& shape, double targetSize);

    /// 三角化并输出裸三角汤。deflection <= 0 时按 diagonal·1e-3。
    /// 返回 false 表示形状里没有可用的三角面片。
    static bool tessellate(const TopoDS_Shape& shape, double deflection,
                           TessellatedSoup& out);
};
