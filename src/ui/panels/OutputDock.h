#pragma once
// 文件职责：日志界面：按级别追加带时间和颜色的消息，供主窗口报告运行结果。

#include <QPlainTextEdit>
#include <QString>

class OutputDock : public QPlainTextEdit
{
    Q_OBJECT

public:
    enum class Level { Info, Ok, Warn, Err };

    /// 创建只读日志窗口并限制保留的日志行数。
    explicit OutputDock(QWidget* parent = nullptr);

    /// 在 GUI 线程追加一条日志；level 控制显示样式，不表示算法或质量状态已通过。
    void appendLog(Level level, const QString& message);

private:
    /// 返回指定日志等级对应的显示颜色。
    static QString levelColor(Level level);
};
