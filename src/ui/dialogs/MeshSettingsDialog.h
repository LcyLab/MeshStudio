#pragma once
/** @file MeshSettingsDialog.h
 * @brief 两类网格的独立参数草稿；仅 Accepted 时由主窗口提交。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include <QDialog>
#include "app/meshing/TetMeshParams.h"
#include "mesh/surface/legacy/CadSurfaceMesher.h"
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QLabel;
class QPushButton;
class QTabWidget;
class QComboBox;

class MeshSettingsDialog final : public QDialog {
    Q_OBJECT
public:
    enum class Mode { Surface,Volume };
    /// 创建表面和体网格参数草稿界面。
    MeshSettingsDialog(const QString& modelName,double diagonal,Mode initialMode,
                       const mesher::SurfaceCriteria& surface,const TetMeshParams& volume,
                       QWidget* parent=nullptr);
    /// 返回当前选中的表面或体网格设置页。
    Mode mode() const;
    /// 长度为导入后的模型坐标单位；读取草稿不修改输入或启动算法。
    mesher::SurfaceCriteria surfaceCriteria() const;
    /// 读取当前支持的体网格生成参数。
    TetMeshParams volumeParameters() const;
protected:
    /// 在窗口显示后应用平台标题栏样式。
    void showEvent(QShowEvent*) override;
    /// 提交输入并检查参数，合法后接受对话框。
    void accept() override;
private:
    /// 构造表面算法选择、尺寸和高级参数页。
    QWidget* buildSurfacePage();
    /// 构造当前支持的体网格尺寸和优化设置页。
    QWidget* buildVolumePage();
    /// 将表面参数填入对应控件。
    void fillSurface(const mesher::SurfaceCriteria&);
    /// 将当前参数页恢复为基于模型尺度的默认设置。
    void resetCurrentPage();
    /// 根据当前参数页更新生成按钮和提示状态。
    void updateMode();
    /// 根据选中的算法切换可见参数组。
    void updateSurfaceAlgorithm();
    double modelDiagonal;
    mesher::SurfaceCriteria originalSurface;
    TetMeshParams originalVolume;
    QTabWidget* tabs=nullptr;
    QDoubleSpinBox *minimum=nullptr,*maximum=nullptr,*deviation=nullptr,*angle=nullptr;
    QDoubleSpinBox *quality=nullptr,*growth=nullptr,*volumeTarget=nullptr;
    QSpinBox *vertices=nullptr,*work=nullptr;
    QCheckBox* optimize=nullptr;
    QLabel *errorLabel=nullptr,*legacyHint=nullptr;
    QPushButton* generateButton=nullptr;
    QComboBox* surfaceAlgorithm=nullptr;
    QWidget *legacyBasic=nullptr,*legacyAdvanced=nullptr,*algorithm6Page=nullptr;
    QDoubleSpinBox* algorithm6Target=nullptr;
    QSpinBox *algorithm6Nodes=nullptr,*algorithm6Queries=nullptr,*algorithm6Smoothing=nullptr;
    QCheckBox* algorithm6Extend=nullptr;
};
