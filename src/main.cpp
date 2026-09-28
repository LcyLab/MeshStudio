#include <QApplication>
#include <QElapsedTimer>

#include <QVTKOpenGLNativeWidget.h>
#include <QFile>
#include <QFileInfo>
#include <QIcon>
#include <QPalette>
#include <QProgressBar>
#include <QSplashScreen>
#include <QStyleFactory>
#include <QThread>

#include <vtkFileOutputWindow.h>
#include <vtkOutputWindow.h>

#include "MainWindow.h"

// VTK 的报错默认会弹出 GUI 窗口打断使用，改为写入文件便于排查
static void redirectVtkMessages()
{
    auto* fileOut = vtkFileOutputWindow::New();
    fileOut->SetFileName("meshstudio_vtk.log");
    vtkOutputWindow::SetInstance(fileOut);
    fileOut->Delete();
}

static void applyLightPalette(QApplication& app)
{
    QPalette p;
    p.setColor(QPalette::Window, QColor(0xf5, 0xf4, 0xf6));
    p.setColor(QPalette::WindowText, QColor(0x21, 0x25, 0x29));
    p.setColor(QPalette::Base, QColor(0xff, 0xff, 0xff));
    p.setColor(QPalette::AlternateBase, QColor(0xfa, 0xfb, 0xfc));
    p.setColor(QPalette::Text, QColor(0x21, 0x25, 0x29));
    p.setColor(QPalette::Button, QColor(0xff, 0xff, 0xff));
    p.setColor(QPalette::ButtonText, QColor(0x21, 0x25, 0x29));
    p.setColor(QPalette::Highlight, QColor(0xe7, 0xda, 0xee));
    p.setColor(QPalette::HighlightedText, QColor(0x21, 0x25, 0x29));
    p.setColor(QPalette::ToolTipBase, QColor(0xff, 0xff, 0xff));
    p.setColor(QPalette::ToolTipText, QColor(0x21, 0x25, 0x29));
    p.setColor(QPalette::Link, QColor(0x58, 0x32, 0x66));
    p.setColor(QPalette::Disabled, QPalette::Text, QColor(0x86, 0x8e, 0x96));
    p.setColor(QPalette::Disabled, QPalette::WindowText, QColor(0x86, 0x8e, 0x96));
    p.setColor(QPalette::Disabled, QPalette::ButtonText, QColor(0x86, 0x8e, 0x96));
    app.setPalette(p);
}

int main(int argc, char* argv[])
{
    // VTK 深度剥离要求: 顶层表面格式预配 alpha/深度位面
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);
    redirectVtkMessages();
    QApplication::setOrganizationName("MeshStudio");
    QApplication::setApplicationName("MeshStudio");
    QApplication::setApplicationVersion("0.1.0");

    app.setStyle(QStyleFactory::create("Fusion"));
    applyLightPalette(app);

    QFile qss(":/theme/light.qss");
    if (qss.open(QIODevice::ReadOnly | QIODevice::Text)) {
        app.setStyleSheet(QString::fromUtf8(qss.readAll()));
    }

    // 标题栏为深茄紫色块, 窗口图标用白色剪影版才可见（about 对话框仍用彩色 logo）
    app.setWindowIcon(QIcon(":/icons/logo-light.svg"));

    // 启动画面：真实进度条 + 至少完整展示约 0.9 秒
    QSplashScreen* splash = nullptr;
    QProgressBar* splashProgress = nullptr;
    const QPixmap splashPix(QStringLiteral(":/icons/splash.svg"));
    if (!splashPix.isNull()) {
        splash = new QSplashScreen(splashPix);
        splashProgress = new QProgressBar(splash);
        splashProgress->setGeometry(260, 326, 160, 7);
        splashProgress->setTextVisible(false);
        splashProgress->setRange(0, 100);
        splashProgress->setValue(8);
        splash->show();
        QCoreApplication::processEvents();
    }
    const auto setProgress = [splashProgress](int value) {
        if (splashProgress) {
            splashProgress->setValue(value);
            QCoreApplication::processEvents();
        }
    };

    setProgress(20);
    QElapsedTimer elapsed;
    elapsed.start();
    MainWindow w;
    while (splash && elapsed.elapsed() < 900) {
        QThread::msleep(20);
        QCoreApplication::processEvents();
    }
    setProgress(60);
    w.show();
    setProgress(80);

    // 支持命令行直接打开几何(可多文件)或 .msp 工程: MeshStudio.exe a.step [b.stl] | x.msp
    const QStringList positional = app.arguments().mid(1);
    for (const QString& argument : positional) {
        if (!QFileInfo::exists(argument))
            continue;
        if (argument.endsWith(QStringLiteral(".msp"), Qt::CaseInsensitive))
            w.openProjectFile(argument);
        else
            w.openFile(argument);
    }
    setProgress(100);

    if (splash) {
        splash->finish(&w);
        delete splash;
    }
    return app.exec();
}
