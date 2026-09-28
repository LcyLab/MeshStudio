#include "OutputDock.h"

#include <QDateTime>
#include <QFont>
#include <QTextBlock>

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
