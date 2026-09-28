#pragma once

#include <QTreeWidget>

class ModelTreePanel : public QTreeWidget
{
    Q_OBJECT

public:
    explicit ModelTreePanel(QWidget* parent = nullptr);

    /// 新增一个已加载模型节点（几何挂"几何模型"分类，网格挂"网格"分类）
    void addModel(int modelId, const QString& name, bool isMesh = false);
    void removeModel(int modelId);
    void setModelChecked(int modelId, bool checked);

signals:
    void modelNodeClicked(int modelId);
    void modelVisibilityChanged(int modelId, bool visible);
    void removeModelRequested(int modelId);
    void modelColorChangeRequested(int modelId);
    void placeholderClicked(const QString& title);

private slots:
    void onItemClicked(QTreeWidgetItem* item, int column);
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onContextMenuRequested(const QPoint& pos);

private:
    enum Kind { KindCategory = 0, KindModel = 1, KindPlaceholder = 2 };

    QTreeWidgetItem* modelItem(int modelId) const;
    QTreeWidgetItem* addSection(const QString& title, const QString& iconPath,
                                const QStringList& placeholders);
    void updateGeometryRootTitle();
    void updateMeshSectionTitle();

    QTreeWidgetItem* m_geometryRoot = nullptr;
    QTreeWidgetItem* m_meshSection = nullptr;
    QTreeWidgetItem* m_diagSection = nullptr;
    int m_modelCount = 0;
    int m_meshCount = 0;
    bool m_updatingChecks = false;
};
