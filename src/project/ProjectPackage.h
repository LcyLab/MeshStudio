#pragma once
// 文件职责：ZIP 容器读写：检查包内相对路径，限制条目读取，并通过临时文件原子替换保存结果。

#include <QByteArray>
#include <QString>
#include <QTemporaryDir>

/// .msp 单文件工程包读写器（标准 ZIP 容器，QuaZip 实现，对齐 EMStudio 的 ProjectPackage）
class ProjectPackage
{
public:
    /// 从工程包读取一个指定条目，不解包其他内容
    static bool readFile(
        const QString& packagePath,
        const QString& relativePath,
        QByteArray& data,
        qint64 maxBytes,
        QString* error = nullptr);

    /// 将目录整体打包为 ZIP（QSaveFile 原子替换，失败不破坏旧包）
    static bool packDirectory(
        const QString& sourceDirPath,
        const QString& packagePath,
        QString* error = nullptr);

    /// 解包到临时目录（调用方持有 QTemporaryDir 保活）
    static bool unpackToTemporaryDirectory(
        const QString& packagePath,
        QTemporaryDir& targetDir,
        QString* error = nullptr);
};
