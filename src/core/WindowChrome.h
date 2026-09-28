#pragma once

class QWidget;

// 统一窗口标题栏外观（Win11 DWM 染色为深茄紫 + 白字，与主窗口顶栏一致）
namespace MeshChrome {

// 把顶层窗口的标题栏染成主题色; 非 Win11 或失败时静默跳过
void styleTitleBar(QWidget* window);

} // namespace MeshChrome
