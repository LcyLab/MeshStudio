#pragma once

#include <QWidget>

#include "core/TetMeshParams.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QProgressBar;
class QPushButton;
class QTableWidget;

class MeshSettingsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit MeshSettingsPanel(QWidget* parent = nullptr);

    void setGenerating(bool generating);

    /// 读取当前面板参数（geometryFilePath 由调用方填入）
    TetMeshParams parameters() const;

    /// 将参数回填到面板（打开工程时恢复）
    void applyParams(const TetMeshParams& params);

    /// 模型加载后按包围盒对角线自动填入默认尺寸
    void setDefaultTargetSize(double diagonal);

signals:
    void generateRequested();

private:
    QWidget* buildGlobalGroup();
    QWidget* buildQualityGroup();
    QWidget* buildLocalGroup();
    QWidget* buildConformalGroup();

    QDoubleSpinBox* m_targetSize = nullptr;
    QDoubleSpinBox* m_minSize = nullptr;
    QDoubleSpinBox* m_curvature = nullptr;
    QComboBox* m_elementOrder = nullptr;
    QComboBox* m_algo3D = nullptr;
    QCheckBox* m_optimize = nullptr;
    QDoubleSpinBox* m_optimizeThreshold = nullptr;

    QTableWidget* m_localTable = nullptr;
    QPushButton* m_btnGenerate = nullptr;
    QProgressBar* m_progress = nullptr;
};
