// 文件职责：ZIP 容器读写：检查包内相对路径，限制条目读取，并通过临时文件原子替换保存结果。
#include "ProjectPackage.h"

#include <QCoreApplication>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QSet>

#include <quazip.h>
#include <quazipfile.h>
#include <quazipnewinfo.h>

namespace {
constexpr int kCopyBufferSize = 1024 * 1024;

/// 存在错误输出指针时写入错误说明。
void setError(QString* error, const QString& text)
{
    if (error)
        *error = text;
}

/// 拒绝绝对路径和目录越界的工程包条目路径。
bool isSafeRelativePath(const QString& path)
{
    if (path.isEmpty() || QDir::isAbsolutePath(path) || path.contains(':'))
        return false;

    const QString normalized = QDir::cleanPath(path).replace('\\', '/');
    if (normalized.startsWith(QStringLiteral("../")) || normalized == QStringLiteral(".."))
        return false;

    const QStringList parts = normalized.split('/', Qt::SkipEmptyParts);
    for (const QString& part : parts) {
        if (part == QStringLiteral(".") || part == QStringLiteral(".."))
            return false;
    }

    return !parts.isEmpty();
}

/// 枚举待打包目录中的文件并校验相对路径。
QStringList collectRelativeFiles(const QString& sourceDirPath, QString* error)
{
    QDir root(sourceDirPath);
    if (!root.exists()) {
        setError(error, QCoreApplication::tr("工程包源目录不存在：%1").arg(sourceDirPath));
        return {};
    }

    QStringList files;
    QDirIterator it(sourceDirPath, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString absolutePath = it.next();
        QString relativePath = root.relativeFilePath(absolutePath).replace('\\', '/');
        relativePath = QDir::cleanPath(relativePath).replace('\\', '/');
        if (!isSafeRelativePath(relativePath)) {
            setError(error, QCoreApplication::tr("工程包内出现不安全路径：%1").arg(relativePath));
            return {};
        }
        files.append(relativePath);
    }

    files.sort();
    return files;
}

/// 将一个源文件写入指定 ZIP 条目。
bool writeZipFile(QuaZip& zip, const QString& relativePath, const QString& sourcePath, QString* error)
{
    QFile sourceFile(sourcePath);
    if (!sourceFile.open(QIODevice::ReadOnly)) {
        setError(error, QCoreApplication::tr("无法读取待打包文件：%1").arg(sourcePath));
        return false;
    }

    QuaZipFile zipFile(&zip);
    QuaZipNewInfo info(relativePath, sourcePath);
    if (!zipFile.open(QIODevice::WriteOnly, info)) {
        setError(error, QCoreApplication::tr("无法创建 ZIP 条目：%1").arg(relativePath));
        return false;
    }

    QByteArray buffer(kCopyBufferSize, Qt::Uninitialized);
    while (!sourceFile.atEnd()) {
        const qint64 bytesRead = sourceFile.read(buffer.data(), buffer.size());
        if (bytesRead < 0) {
            setError(error, QCoreApplication::tr("无法读取待打包文件：%1").arg(sourcePath));
            return false;
        }
        if (bytesRead > 0 && zipFile.write(buffer.constData(), bytesRead) != bytesRead) {
            setError(error, QCoreApplication::tr("无法写入 ZIP 条目：%1").arg(relativePath));
            return false;
        }
    }

    zipFile.close();
    if (zipFile.getZipError() != ZIP_OK) {
        setError(error, QCoreApplication::tr("无法完成 ZIP 条目：%1").arg(relativePath));
        return false;
    }

    return true;
}

/// 删除旧目标后将临时工程包重命名到目标路径。
bool replaceFile(const QString& tempPath, const QString& targetPath, QString* error)
{
    if (QFile::exists(targetPath) && !QFile::remove(targetPath)) {
        setError(error, QCoreApplication::tr("无法替换已有工程包：%1").arg(targetPath));
        return false;
    }

    if (!QFile::rename(tempPath, targetPath)) {
        setError(error, QCoreApplication::tr("无法落盘工程包：%1").arg(targetPath));
        return false;
    }

    return true;
}
} // namespace

/// 按大小上限读取工程包中的指定文件条目。
bool ProjectPackage::readFile(
    const QString& packagePath,
    const QString& relativePath,
    QByteArray& data,
    qint64 maxBytes,
    QString* error)
{
    data.clear();
    const QString normalizedPath = QDir::cleanPath(relativePath).replace('\\', '/');
    if (!isSafeRelativePath(normalizedPath)) {
        setError(error, QCoreApplication::tr("工程包内出现不安全路径：%1").arg(relativePath));
        return false;
    }
    if (maxBytes < 0) {
        setError(error, QCoreApplication::tr("工程包条目大小上限无效"));
        return false;
    }

    QuaZip zip(packagePath);
    zip.setFileNameCodec("UTF-8");
    if (!zip.open(QuaZip::mdUnzip)) {
        setError(error, QCoreApplication::tr("无法打开工程包：%1").arg(packagePath));
        return false;
    }
    if (!zip.setCurrentFile(normalizedPath, QuaZip::csSensitive)) {
        setError(error, QCoreApplication::tr("工程包内缺少条目：%1").arg(normalizedPath));
        zip.close();
        return false;
    }

    QuaZipFile zipFile(&zip);
    if (!zipFile.open(QIODevice::ReadOnly)) {
        setError(error, QCoreApplication::tr("无法读取 ZIP 条目：%1").arg(normalizedPath));
        zip.close();
        return false;
    }

    QByteArray buffer(kCopyBufferSize, Qt::Uninitialized);
    while (!zipFile.atEnd()) {
        const qint64 remaining = maxBytes - static_cast<qint64>(data.size());
        const qint64 requestSize = remaining >= buffer.size()
            ? static_cast<qint64>(buffer.size())
            : remaining + 1;
        const qint64 bytesRead = zipFile.read(buffer.data(), requestSize);
        if (bytesRead < 0) {
            setError(error, QCoreApplication::tr("无法读取 ZIP 条目：%1").arg(normalizedPath));
            data.clear();
            zipFile.close();
            zip.close();
            return false;
        }
        if (bytesRead > remaining) {
            setError(error, QCoreApplication::tr("工程包条目超出大小上限：%1").arg(normalizedPath));
            data.clear();
            zipFile.close();
            zip.close();
            return false;
        }
        if (bytesRead > 0)
            data.append(buffer.constData(), static_cast<int>(bytesRead));
    }

    zipFile.close();
    zip.close();
    if (zip.getZipError() != UNZ_OK) {
        setError(error, QCoreApplication::tr("工程包读取未正常结束：%1").arg(packagePath));
        data.clear();
        return false;
    }

    return true;
}

/// 将暂存目录中的文件打包为工程文件。
bool ProjectPackage::packDirectory(
    const QString& sourceDirPath,
    const QString& packagePath,
    QString* error)
{
    const QStringList relativeFiles = collectRelativeFiles(sourceDirPath, error);
    if (relativeFiles.isEmpty()) {
        if (error && error->isEmpty())
            setError(error, QCoreApplication::tr("工程包源目录为空"));
        return false;
    }

    QDir root(sourceDirPath);
    QFileInfo packageInfo(packagePath);
    if (!packageInfo.dir().exists() && !packageInfo.dir().mkpath(QStringLiteral("."))) {
        setError(error, QCoreApplication::tr("无法创建工程包目录：%1").arg(packageInfo.absolutePath()));
        return false;
    }

    const QString tempPath = packagePath + QStringLiteral(".tmp");
    QFile::remove(tempPath);

    QuaZip zip(tempPath);
    zip.setFileNameCodec("UTF-8");
    if (!zip.open(QuaZip::mdCreate)) {
        setError(error, QCoreApplication::tr("无法写入工程包：%1").arg(packagePath));
        return false;
    }

    for (const QString& relativePath : relativeFiles) {
        if (!writeZipFile(zip, relativePath, root.filePath(relativePath), error)) {
            zip.close();
            QFile::remove(tempPath);
            return false;
        }
    }

    zip.close();
    if (zip.getZipError() != ZIP_OK) {
        setError(error, QCoreApplication::tr("工程包打包未正常结束：%1").arg(packagePath));
        QFile::remove(tempPath);
        return false;
    }

    return replaceFile(tempPath, packageInfo.absoluteFilePath(), error);
}

/// 校验工程包条目路径并解包到调用方持有的临时目录。
bool ProjectPackage::unpackToTemporaryDirectory(
    const QString& packagePath,
    QTemporaryDir& targetDir,
    QString* error)
{
    if (!targetDir.isValid()) {
        setError(error, QCoreApplication::tr("无法创建工程解包临时目录"));
        return false;
    }

    QuaZip zip(packagePath);
    zip.setFileNameCodec("UTF-8");
    if (!zip.open(QuaZip::mdUnzip)) {
        setError(error, QCoreApplication::tr("无法打开工程包：%1").arg(packagePath));
        return false;
    }

    QDir targetRoot(targetDir.path());
    QSet<QString> seenPaths;
    bool hasAnyFile = false;
    for (bool more = zip.goToFirstFile(); more; more = zip.goToNextFile()) {
        QString relativePath = zip.getCurrentFileName();
        relativePath = QDir::cleanPath(relativePath).replace('\\', '/');
        if (relativePath.endsWith('/'))
            continue;

        if (!isSafeRelativePath(relativePath) || seenPaths.contains(relativePath)) {
            setError(error, QCoreApplication::tr("工程包内包含无效路径"));
            zip.close();
            return false;
        }
        seenPaths.insert(relativePath);

        QuaZipFile zipFile(&zip);
        if (!zipFile.open(QIODevice::ReadOnly)) {
            setError(error, QCoreApplication::tr("无法读取 ZIP 条目：%1").arg(relativePath));
            zip.close();
            return false;
        }

        const QString targetPath = targetRoot.filePath(relativePath);
        const QFileInfo targetInfo(targetPath);
        if (!targetRoot.mkpath(targetInfo.path())) {
            setError(error, QCoreApplication::tr("无法创建解包目录：%1").arg(targetInfo.path()));
            zip.close();
            return false;
        }

        QSaveFile outputFile(targetPath);
        if (!outputFile.open(QIODevice::WriteOnly)) {
            setError(error, QCoreApplication::tr("无法写入解包文件：%1").arg(targetPath));
            zip.close();
            return false;
        }

        QByteArray buffer(kCopyBufferSize, Qt::Uninitialized);
        while (!zipFile.atEnd()) {
            const qint64 bytesRead = zipFile.read(buffer.data(), buffer.size());
            if (bytesRead < 0) {
                setError(error, QCoreApplication::tr("无法读取 ZIP 条目：%1").arg(relativePath));
                zip.close();
                return false;
            }
            if (bytesRead > 0 && outputFile.write(buffer.constData(), bytesRead) != bytesRead) {
                setError(error, QCoreApplication::tr("无法写入解包文件：%1").arg(targetPath));
                zip.close();
                return false;
            }
        }

        zipFile.close();
        if (!outputFile.commit()) {
            setError(error, QCoreApplication::tr("解包文件写入未完成：%1").arg(targetPath));
            zip.close();
            return false;
        }

        hasAnyFile = true;
    }

    zip.close();
    if (zip.getZipError() != UNZ_OK) {
        setError(error, QCoreApplication::tr("工程包解包未正常结束：%1").arg(packagePath));
        return false;
    }

    if (!hasAnyFile) {
        setError(error, QCoreApplication::tr("工程包为空：%1").arg(packagePath));
        return false;
    }

    return true;
}
