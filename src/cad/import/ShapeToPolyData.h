#pragma once
// 文件职责：CAD 展示转换：提取 OCC 离散三角片和棱线供 VTK 显示；这些片体不等于自研表面生成结果。

#include <array>
#include <vector>

#include <TopoDS_Shape.hxx>
#include <vtkSmartPointer.h>

class vtkPolyData;

/// 三角化片体的裸数据（顶点坐标 + 三角形顶点索引）。
/// 供离散兼容工具和独立测试消费；当前 CAD 体网格主链使用自研索引表面。
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
/// 本类用 BRepMesh_IncrementalMesh 准备展示用 Poly_Triangulation，再按面提取顶点。
/// 相邻面可能保留重复顶点；若兼容调用方把这些片送入三角汤体网格入口，需先焊接。
/// 自研 CAD 表面主链则由共享边界身份连接各面，不以展示离散片替代生成结果。
class ShapeToPolyData
{
public:
    /// 将 OCC 面的离散三角形转换为 VTK 表面数据。
    static vtkSmartPointer<vtkPolyData> surfaceMesh(const TopoDS_Shape& shape);

    /// 指定展示离散的弦高偏差（模型长度单位）；兼容三角汤调用方也可显式指定。
    static vtkSmartPointer<vtkPolyData> surfaceMesh(const TopoDS_Shape& shape,
                                                    double deflection);

    /// 采样 CAD 边并生成 VTK 折线数据。
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
