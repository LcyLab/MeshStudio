// 文件职责：窗口外观适配：在支持的 Windows 环境设置 DWM 标题栏颜色，调用失败时保留系统默认外观。
#include "WindowChrome.h"

#include <QWidget>

#ifdef Q_OS_WIN
#include <dwmapi.h>
#include <windows.h>

namespace MeshChrome {

/// 把顶层窗口的标题栏染成主题色; 非 Win11 或失败时静默跳过。
void styleTitleBar(QWidget* window)
{
    if (!window)
        return;
    // 不做版本预判：旧系统上 DwmSetWindowAttribute 返回错误码，静默跳过
    const COLORREF caption = RGB(0x58, 0x32, 0x66); // 主题深茄紫
    const COLORREF white = RGB(0xFF, 0xFF, 0xFF);
    const BOOL dark = TRUE;
    HWND hwnd = reinterpret_cast<HWND>(window->winId());
    DwmSetWindowAttribute(hwnd, 20, &dark, sizeof(dark)); // 深色标题栏按钮
    const HRESULT hr = DwmSetWindowAttribute(hwnd, 35, &caption, sizeof(caption));
    DwmSetWindowAttribute(hwnd, 36, &white, sizeof(white));
    DwmSetWindowAttribute(hwnd, 34, &caption, sizeof(caption));
    if (SUCCEEDED(hr))
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0,
                     SWP_FRAMECHANGED | SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE);
}

} // namespace MeshChrome

#else // !Q_OS_WIN

namespace MeshChrome {
/// 把顶层窗口的标题栏染成主题色; 非 Win11 或失败时静默跳过。
void styleTitleBar(QWidget*) {}
} // namespace MeshChrome

#endif
