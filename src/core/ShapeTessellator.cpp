#include "ShapeTessellator.h"

#include <QFileInfo>

#include <TopoDS_Shape.hxx>

#include <string>

#include "ShapeLoader.h"
#include "ShapeToPolyData.h"

namespace ShapeTessellator {

bool isBRepFormat(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return suffix == QStringLiteral("step") || suffix == QStringLiteral("stp")
        || suffix == QStringLiteral("iges") || suffix == QStringLiteral("igs")
        || suffix == QStringLiteral("brep");
}

bool tessellateFile(const QString& filePath,
                    double targetSize,
                    std::vector<mesher::Point3D>& vertices,
                    std::vector<std::array<int, 3>>& triangles,
                    QString* error)
{
    vertices.clear();
    triangles.clear();

    TopoDS_Shape shape;
    QString loadErr;
    if (!ShapeLoader::load(filePath, shape, &loadErr)) {
        if (error)
            *error = loadErr;
        return false;
    }
    if (shape.IsNull()) {
        if (error)
            *error = QStringLiteral("几何为空：%1").arg(filePath);
        return false;
    }

    TessellatedSoup soup;
    const double deflection = ShapeToPolyData::deflectionFor(shape, targetSize);
    if (!ShapeToPolyData::tessellate(shape, deflection, soup)) {
        if (error)
            *error = QStringLiteral("面向网格离散化未产生三角面片：%1").arg(filePath);
        return false;
    }

    vertices.reserve(soup.points.size());
    for (const auto& p : soup.points)
        vertices.push_back({ p[0], p[1], p[2] });
    triangles = std::move(soup.triangles);
    return true;
}

} // namespace ShapeTessellator
