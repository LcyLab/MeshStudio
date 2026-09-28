#pragma once

#include <QPlainTextEdit>
#include <QString>

class OutputDock : public QPlainTextEdit
{
    Q_OBJECT

public:
    enum class Level { Info, Ok, Warn, Err };

    explicit OutputDock(QWidget* parent = nullptr);

    void appendLog(Level level, const QString& message);

private:
    static QString levelColor(Level level);
};
