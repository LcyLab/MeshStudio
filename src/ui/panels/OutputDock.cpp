// 文件职责：日志界面：按级别追加带时间和颜色的消息，供主窗口报告运行结果。
#include "OutputDock.h"

#include <QDateTime>
#include <QFont>
#include <QTextBlock>

/// 创建只读日志窗口并限制保留的日志行数。
OutputDock::OutputDock(QWidget* parent)
    : QPlainTextEdit(parent)
{
    setReadOnly(true);
    setFrameShape(QFrame::NoFrame);
    setMaximumBlockCount(2000);
    setLineWrapMode(QPlainTextEdit::WidgetWidth);

    QFont mono(QStringLiteral("Consolas"));
    mono.setStyleHint(QFont::TypeWriter);
    mono.setPointSize(11);
    setFont(mono);
}

/// 返回指定日志等级对应的显示颜色。
QString OutputDock::levelColor(Level level)
{
    switch (level) {
    case Level::Ok:   return QStringLiteral("#2F7D5B");
    case Level::Warn: return QStringLiteral("#B45309");
    case Level::Err:  return QStringLiteral("#C0392B");
    case Level::Info:
    default:          return QStringLiteral("#495057");
    }
}

/// 在日志窗口追加带时间和等级颜色的消息。
void OutputDock::appendLog(Level level, const QString& message)
{
    const QString time = QDateTime::currentDateTime().toString(QStringLiteral("hh:mm:ss"));
    QString text = message;
    text.replace(QLatin1Char('<'), QStringLiteral("&lt;"));
    text.replace(QLatin1Char('>'), QStringLiteral("&gt;"));

    const QString html = QStringLiteral(
                             "<span style=\"color:#868E96\">[%1]</span> "
                             "<span style=\"color:%2\">%3</span>")
                             .arg(time, levelColor(level), text);

    // 追加为只读块
    QTextCursor cursor = textCursor();
    cursor.movePosition(QTextCursor::End);
    cursor.insertBlock();
    cursor.insertHtml(html);
    setTextCursor(cursor);
    ensureCursorVisible();
}
