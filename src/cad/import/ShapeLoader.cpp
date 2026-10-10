// 文件职责：CAD 文件入口：按格式调用 OCC 读取器，并为非 ASCII 文件路径提供编码回退。
#include "ShapeLoader.h"

#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QString>

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <IGESControl_Reader.hxx>
#include <STEPControl_Reader.hxx>

namespace {

enum class Format { Step, Iges, BRepFile };

/// 根据文件后缀识别 CAD 格式。
Format formatOf(const QString& suffix)
{
    if (suffix == QStringLiteral("step") || suffix == QStringLiteral("stp"))
        return Format::Step;
    if (suffix == QStringLiteral("iges") || suffix == QStringLiteral("igs"))
        return Format::Iges;
    return Format::BRepFile;
}

/// 检查路径能否仅用 ASCII 字符表示。
bool isAsciiPath(const QString& path)
{
    for (const QChar c : path) {
        if (c.unicode() > 127)
            return false;
    }
    return true;
}

// 复制到 ASCII 临时路径，规避各种旧运行时的路径编码问题
QString makeAsciiTempCopy(const QString& filePath, const QString& suffix)
{
    const QString tempDir =
        QStandardPaths::writableLocation(QStandardPaths::TempLocation);
    if (tempDir.isEmpty())
        return QString();
    const QString tempPath = tempDir + QStringLiteral("/meshstudio_%1.%2")
                                                  .arg(uint(qHash(filePath)), 0, 16)
                                                  .arg(suffix);
    if (!QFile::exists(tempPath) && !QFile::copy(filePath, tempPath))
        return QString();
    return tempPath;
}

/// 使用指定路径编码调用相应的 OCC 文件读取器。
bool readWithEncoding(const QByteArray& pathBytes, Format format,
                      TopoDS_Shape& out, QString& lastError)
{
    const Standard_CString cpath = pathBytes.constData();
    switch (format) {
    case Format::Step: {
        STEPControl_Reader reader;
        if (reader.ReadFile(cpath) != IFSelect_RetDone)
            return false;
        if (reader.TransferRoots() <= 0)
            return false;
        out = reader.OneShape();
        return !out.IsNull();
    }
    case Format::Iges: {
        IGESControl_Reader reader;
        if (reader.ReadFile(cpath) != IFSelect_RetDone)
            return false;
        if (reader.TransferRoots() <= 0)
            return false;
        out = reader.OneShape();
        return !out.IsNull();
    }
    case Format::BRepFile: {
        BRep_Builder builder;
        return BRepTools::Read(out, cpath, builder) && !out.IsNull();
    }
    }
    return false;
}

} // namespace

/// 判断文件后缀是否属于支持的 CAD 格式。
bool ShapeLoader::isSupported(const QString& filePath)
{
    const QString suffix = QFileInfo(filePath).suffix().toLower();
    return suffix == QStringLiteral("step") || suffix == QStringLiteral("stp")
        || suffix == QStringLiteral("iges") || suffix == QStringLiteral("igs")
        || suffix == QStringLiteral("brep");
}

/// 读取 CAD 几何；路径编码失败时尝试本地编码和 ASCII 临时副本。
bool ShapeLoader::load(const QString& filePath, TopoDS_Shape& out, QString* errorMessage)
{
    const auto fail = [errorMessage](const QString& message) {
        if (errorMessage)
            *errorMessage = message;
        return false;
    };

    const QFileInfo info(filePath);
    if (!info.exists() || !info.isFile())
        return fail(QObject::tr("文件不存在: %1").arg(filePath));

    const QString absolutePath = info.absoluteFilePath();
    const QString suffix = info.suffix().toLower();
    const Format format = formatOf(suffix);

    // OCCT 7.2+ 按 UTF-8 解析路径；旧构建按 ANSI。两级都试，最后退到 ASCII 临时副本
    if (readWithEncoding(absolutePath.toUtf8(), format, out, QString()))
        return true;
    if (readWithEncoding(absolutePath.toLocal8Bit(), format, out, QString()))
        return true;

    if (!isAsciiPath(absolutePath)) {
        const QString tempPath = makeAsciiTempCopy(absolutePath, suffix);
        if (!tempPath.isEmpty()) {
            const bool ok =
                readWithEncoding(QFile::encodeName(tempPath), format, out, QString());
            QFile::remove(tempPath);
            if (ok)
                return true;
        }
    }

    return fail(QObject::tr("%1 文件解析失败: %2")
                    .arg(suffix.toUpper(), absolutePath));
}
