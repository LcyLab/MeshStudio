#pragma once

#include <QString>
#include <TopoDS_Shape.hxx>

/// 按后缀路由的几何加载器（与 EMStudio ShapeLoader 对齐）：
/// STEP(.step/.stp) / IGES(.iges/.igs) / BRep(.brep)
class ShapeLoader
{
public:
    static bool isSupported(const QString& filePath);

    static bool load(const QString& filePath, TopoDS_Shape& out,
                     QString* errorMessage = nullptr);
};
