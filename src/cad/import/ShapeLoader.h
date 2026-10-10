#pragma once
// 文件职责：CAD 文件入口：按格式调用 OCC 读取器，并为非 ASCII 文件路径提供编码回退。

#include <QString>
#include <TopoDS_Shape.hxx>

/// 按后缀路由的几何加载器（与 EMStudio ShapeLoader 对齐）：
/// STEP(.step/.stp) / IGES(.iges/.igs) / BRep(.brep)
class ShapeLoader
{
public:
    /// 判断文件后缀是否属于支持的 CAD 格式。
    static bool isSupported(const QString& filePath);

    /**
     * @brief 读取 CAD 几何；路径编码失败时尝试本地编码和 ASCII 临时副本。
     * @param out 成功时获得 OCC 形状句柄，调用方持有并交给展示或几何适配器。
     * @param errorMessage 非空时接收失败原因；返回 false 后不得使用 out 作为有效输入。
     * @return 是否读取到非空形状；此步骤不生成自研网格，也不代表 CAD 完整验收。
     */
    static bool load(const QString& filePath, TopoDS_Shape& out,
                     QString* errorMessage = nullptr);
};
