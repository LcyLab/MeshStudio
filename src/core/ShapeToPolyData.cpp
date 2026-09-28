#include "ShapeToPolyData.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Bnd_Box.hxx>
#include <GCPnts_QuasiUniformDeflection.hxx>
#include <Poly_Polygon3D.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <gp_Pnt.hxx>

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkIdTypeArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kRelativeDeflection = 0.001;   // 对角线的千分之一（OCC 默认相对精度）
constexpr double kTargetDeflectionRatio = 0.25; // 弦高 ≤ 目标边长的 1/4
constexpr double kFallbackDeflection = 0.1;     // 包围盒退化时的兜底值

} // namespace

double ShapeToPolyData::boundingDiagonal(const TopoDS_Shape& shape)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);
    if (box.IsVoid())
        return 0.0;
    double xmin, ymin, zmin, xmax, ymax, zmax;
    box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    return std::sqrt((xmax - xmin) * (xmax - xmin)
                     + (ymax - ymin) * (ymax - ymin)
                     + (zmax - zmin) * (zmax - zmin));
}

double ShapeToPolyData::deflectionFor(const TopoDS_Shape& shape, double targetSize)
{
    const double diag = boundingDiagonal(shape);
    const double base = diag > 0.0 ? diag * kRelativeDeflection : kFallbackDeflection;
    if (targetSize <= 0.0)
        return base;
    return std::min(base, targetSize * kTargetDeflectionRatio);
}

vtkSmartPointer<vtkPolyData> ShapeToPolyData::surfaceMesh(const TopoDS_Shape& shape)
{
    return surfaceMesh(shape, deflectionFor(shape, 0.0));
}

vtkSmartPointer<vtkPolyData> ShapeToPolyData::surfaceMesh(const TopoDS_Shape& shape,
                                                          double deflection)
{
    // ★ Clean 不可省 ★：形状可能已被更早的调用（视口加载）用较粗的弦高离散过，
    // 而 BRepMesh_IncrementalMesh 在已有三角化满足新弦高时会直接复用旧网格。
    // 先清掉才能保证按本次 deflection 重新离散。
    BRepTools::Clean(shape);
    BRepMesh_IncrementalMesh mesher(shape, deflection, false, 0.5, true);
    (void)mesher;

    // 面 -> 所属实体（同一探索顺序），用于 SolidId 单元标量
    TopTools_IndexedDataMapOfShapeListOfShape faceAncestors;
    TopExp::MapShapesAndAncestors(shape, TopAbs_FACE, TopAbs_SOLID, faceAncestors);
    TopTools_IndexedMapOfShape solidMap;
    TopExp::MapShapes(shape, TopAbs_SOLID, solidMap);

    auto points = vtkSmartPointer<vtkPoints>::New();
    auto triangles = vtkSmartPointer<vtkCellArray>::New();
    auto faceIds = vtkSmartPointer<vtkIdTypeArray>::New();
    faceIds->SetName("FaceId");
    auto solidIds = vtkSmartPointer<vtkIdTypeArray>::New();
    solidIds->SetName("SolidId");

    TopTools_IndexedMapOfShape faceMap;
    TopExp::MapShapes(shape, TopAbs_FACE, faceMap);
    for (int faceId = 1; faceId <= faceMap.Extent(); ++faceId) {
        const TopoDS_Face face = TopoDS::Face(faceMap(faceId));
        TopLoc_Location location;
        const Handle(Poly_Triangulation)& triangulation =
            BRep_Tool::Triangulation(face, location);
        if (triangulation.IsNull() || triangulation->NbTriangles() <= 0)
            continue;

        int solidId = -1;
        const TopTools_ListOfShape& ancestors =
            faceAncestors.FindFromIndex(faceId);
        if (!ancestors.IsEmpty())
            solidId = solidMap.FindIndex(ancestors.First()) - 1; // 0 基

        const gp_Trsf& transform = location.Transformation();
        const vtkIdType base = points->GetNumberOfPoints();
        for (int i = 1; i <= triangulation->NbNodes(); ++i) {
            const gp_Pnt p = triangulation->Node(i).Transformed(transform);
            points->InsertNextPoint(p.X(), p.Y(), p.Z());
        }
        for (int t = 1; t <= triangulation->NbTriangles(); ++t) {
            const Poly_Triangle triangle = triangulation->Triangle(t);
            const int n1 = triangle.Value(1);
            const int n2 = triangle.Value(2);
            const int n3 = triangle.Value(3);
            const vtkIdType ids[3] = {base + n1 - 1, base + n2 - 1, base + n3 - 1};
            triangles->InsertNextCell(3, ids);
            faceIds->InsertNextValue(faceId - 1);
            solidIds->InsertNextValue(solidId);
        }
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetPolys(triangles);
    if (faceIds->GetNumberOfValues() > 0)
        polyData->GetCellData()->AddArray(faceIds);
    if (solidIds->GetNumberOfValues() > 0)
        polyData->GetCellData()->AddArray(solidIds);
    return polyData;
}

vtkSmartPointer<vtkPolyData> ShapeToPolyData::edgeLines(const TopoDS_Shape& shape)
{
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    auto edgeIds = vtkSmartPointer<vtkIdTypeArray>::New();
    edgeIds->SetName("EdgeId");

    const double deflection = deflectionFor(shape, 0.0);
    TopTools_IndexedMapOfShape edgeMap;
    TopExp::MapShapes(shape, TopAbs_EDGE, edgeMap);
    for (int edgeId = 1; edgeId <= edgeMap.Extent(); ++edgeId) {
        const TopoDS_Edge edge = TopoDS::Edge(edgeMap(edgeId));
        if (BRep_Tool::Degenerated(edge))
            continue;

        std::vector<gp_Pnt> polyline;
        TopLoc_Location location;
        const Handle(Poly_Polygon3D)& polygon = BRep_Tool::Polygon3D(edge, location);
        if (!polygon.IsNull()) {
            const gp_Trsf& transform = location.Transformation();
            const TColgp_Array1OfPnt& nodes = polygon->Nodes();
            polyline.reserve(nodes.Length());
            for (int i = nodes.Lower(); i <= nodes.Upper(); ++i)
                polyline.push_back(nodes.Value(i).Transformed(transform));
        } else {
            BRepAdaptor_Curve curve(edge);
            GCPnts_QuasiUniformDeflection discretizer(curve, deflection);
            if (!discretizer.IsDone() || discretizer.NbPoints() < 2)
                continue;
            polyline.reserve(discretizer.NbPoints());
            for (int i = 1; i <= discretizer.NbPoints(); ++i)
                polyline.push_back(discretizer.Value(i));
        }
        if (polyline.size() < 2)
            continue;

        const vtkIdType base = points->GetNumberOfPoints();
        for (const gp_Pnt& p : polyline)
            points->InsertNextPoint(p.X(), p.Y(), p.Z());
        vtkIdType count = static_cast<vtkIdType>(polyline.size());
        std::vector<vtkIdType> ids(count);
        for (vtkIdType i = 0; i < count; ++i)
            ids[i] = base + i;
        lines->InsertNextCell(count, ids.data());
        edgeIds->InsertNextValue(edgeId - 1);
    }

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);
    if (edgeIds->GetNumberOfValues() > 0)
        polyData->GetCellData()->AddArray(edgeIds);
    return polyData;
}

bool ShapeToPolyData::tessellate(const TopoDS_Shape& shape, double deflection,
                                 TessellatedSoup& out)
{
    out.points.clear();
    out.triangles.clear();

    const double d = (deflection > 0.0) ? deflection : deflectionFor(shape, 0.0);
    // 复用 surfaceMesh 的逐面遍历：保证四面体化输入与视口显示的是**同一批**三角形。
    vtkSmartPointer<vtkPolyData> poly = surfaceMesh(shape, d);
    if (!poly || poly->GetNumberOfPoints() == 0 || poly->GetNumberOfPolys() == 0)
        return false;

    const vtkIdType nPts = poly->GetNumberOfPoints();
    out.points.resize(static_cast<std::size_t>(nPts));
    for (vtkIdType i = 0; i < nPts; ++i) {
        double p[3];
        poly->GetPoint(i, p);
        out.points[static_cast<std::size_t>(i)] = { p[0], p[1], p[2] };
    }

    vtkCellArray* polys = poly->GetPolys();
    out.triangles.reserve(static_cast<std::size_t>(poly->GetNumberOfPolys()));
    polys->InitTraversal();
    vtkIdType npts = 0;
    const vtkIdType* ids = nullptr;
    while (polys->GetNextCell(npts, ids)) {
        if (npts != 3)
            continue;   // 逐面离散只产出三角形；保险起见跳过非三角单元
        out.triangles.push_back({ static_cast<int>(ids[0]),
                                  static_cast<int>(ids[1]),
                                  static_cast<int>(ids[2]) });
    }
    return !out.triangles.empty();
}
