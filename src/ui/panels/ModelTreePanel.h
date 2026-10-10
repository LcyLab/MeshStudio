#pragma once
// 文件职责：模型树界面：用模型 ID 关联节点和视口，发出选择、显隐、删除和改色请求。

#include <QTreeWidget>

// 树节点只保存模型关联 ID；真正的几何、网格和外观状态由主窗口/视口管理。
class ModelTreePanel : public QTreeWidget
{
    Q_OBJECT

public:
    /// 构造模型树分类及用户操作信号连接。
    explicit ModelTreePanel(QWidget* parent = nullptr);

    /// 新增一个已加载模型节点（几何挂"几何模型"分类，网格挂"网格"分类）
    void addModel(int modelId, const QString& name, bool isMesh = false);
    /// 移除模型树条目并更新分类数量及空态提示。
    void removeModel(int modelId);
    /// 从外部同步勾选状态，使用内部保护标记避免再次发出用户显隐请求。
    void setModelChecked(int modelId, bool checked);

signals:
    /// 通知主窗口用户选择了一个模型。
    void modelNodeClicked(int modelId);
    /// 通知模型可见性发生用户修改。
    void modelVisibilityChanged(int modelId, bool visible);
    /// 通知主窗口删除指定模型。
    void removeModelRequested(int modelId);
    /// 通知主窗口打开模型颜色选择。
    void modelColorChangeRequested(int modelId);
    /// 通知主窗口用户点击了功能占位项。
    void placeholderClicked(const QString& title);

private slots:
    /// 将树节点点击转换为模型选择或占位项通知。
    void onItemClicked(QTreeWidgetItem* item, int column);
    /// 将用户修改的勾选状态转换为可见性通知。
    void onItemChanged(QTreeWidgetItem* item, int column);
    /// 为指定模型显示颜色、变换和删除操作菜单。
    void onContextMenuRequested(const QPoint& pos);

private:
    enum Kind { KindCategory = 0, KindModel = 1, KindPlaceholder = 2 };

    /// 按模型编号查找对应的树节点。
    QTreeWidgetItem* modelItem(int modelId) const;
    /// 添加模型树分类及其空态占位条目。
    QTreeWidgetItem* addSection(const QString& title, const QString& iconPath,
                                const QStringList& placeholders);
    /// 按几何模型数量更新分类标题。
    void updateGeometryRootTitle();
    /// 按网格数量更新网格分类标题。
    void updateMeshSectionTitle();

    QTreeWidgetItem* m_geometryRoot = nullptr;
    QTreeWidgetItem* m_meshSection = nullptr;
    QTreeWidgetItem* m_diagSection = nullptr;
    int m_modelCount = 0;
    int m_meshCount = 0;
    bool m_updatingChecks = false;
};
