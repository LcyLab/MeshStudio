// 文件职责：模型树界面：用模型 ID 关联节点和视口，发出选择、显隐、删除和改色请求。
#include "ModelTreePanel.h"

#include <QBrush>
#include <QColor>
#include <QMenu>

namespace {
const QColor kDimText(0x8a, 0x94, 0x9e);

constexpr int kRoleModelId = Qt::UserRole + 1;
constexpr int kRoleKind = Qt::UserRole + 2;
} // namespace

/// 构造模型树分类及用户操作信号连接。
ModelTreePanel::ModelTreePanel(QWidget* parent)
    : QTreeWidget(parent)
{
    setHeaderHidden(true);
    setIndentation(16);
    setIconSize(QSize(20, 20));
    setFrameShape(QFrame::NoFrame);
    setSelectionMode(QAbstractItemView::NoSelection);
    setFocusPolicy(Qt::NoFocus);
    setContextMenuPolicy(Qt::CustomContextMenu);

    m_geometryRoot = addSection(tr("几何模型"), ":/icons/cube.svg",
                                {tr("导入的模型将显示在这里")});
    m_meshSection = addSection(tr("网格（未生成）"), ":/icons/mesh-grid.svg",
                               {tr("单元"), tr("网格线"), tr("质量统计")});
    m_diagSection = addSection(tr("诊断（0）"), ":/icons/warning.svg",
                               {tr("共形单元列表"), tr("退化单元列表")});
    m_geometryRoot->setExpanded(true);

    connect(this, &QTreeWidget::itemClicked,
            this, &ModelTreePanel::onItemClicked);
    connect(this, &QTreeWidget::itemChanged,
            this, &ModelTreePanel::onItemChanged);
    connect(this, &QTreeWidget::customContextMenuRequested,
            this, &ModelTreePanel::onContextMenuRequested);
}

/// 添加模型树分类及其空态占位条目。
QTreeWidgetItem* ModelTreePanel::addSection(const QString& title,
                                            const QString& iconPath,
                                            const QStringList& placeholders)
{
    auto* section = new QTreeWidgetItem(this);
    section->setText(0, title);
    section->setIcon(0, QIcon(iconPath));
    section->setFlags(Qt::ItemIsEnabled);
    section->setData(0, kRoleKind, KindCategory);

    for (const QString& text : placeholders) {
        auto* child = new QTreeWidgetItem(section);
        child->setText(0, text);
        child->setForeground(0, QBrush(kDimText));
        child->setFlags(Qt::NoItemFlags);
        child->setData(0, kRoleKind, KindPlaceholder);
    }
    return section;
}

/// 按几何模型数量更新分类标题。
void ModelTreePanel::updateGeometryRootTitle()
{
    m_geometryRoot->setText(0, m_modelCount > 0
                                   ? tr("几何模型（%1）").arg(m_modelCount)
                                   : tr("几何模型"));
}

/// 按网格数量更新网格分类标题。
void ModelTreePanel::updateMeshSectionTitle()
{
    m_meshSection->setText(0, m_meshCount > 0
                                  ? tr("网格（%1）").arg(m_meshCount)
                                  : tr("网格（未生成）"));
}

/// 在模型树中添加指定模型的显示条目。
void ModelTreePanel::addModel(int modelId, const QString& name, bool isMesh)
{
    m_updatingChecks = true;

    auto* section = isMesh ? m_meshSection : m_geometryRoot;

    // 移除空态占位（可能存在多个）
    for (int i = section->childCount() - 1; i >= 0; --i) {
        if (section->child(i)->data(0, kRoleKind).toInt() == KindPlaceholder)
            delete section->takeChild(i);
    }

    auto* modelNode = new QTreeWidgetItem(section);
    modelNode->setText(0, name);
    modelNode->setIcon(0, QIcon(isMesh ? ":/icons/mesh-grid.svg" : ":/icons/cube.svg"));
    modelNode->setFlags(Qt::ItemIsEnabled | Qt::ItemIsUserCheckable);
    modelNode->setCheckState(0, Qt::Checked);
    modelNode->setData(0, kRoleModelId, modelId);
    modelNode->setData(0, kRoleKind, KindModel);
    modelNode->setToolTip(0, tr("勾选控制显示/隐藏，右键可移除"));

    if (isMesh) {
        ++m_meshCount;
        updateMeshSectionTitle();
        m_meshSection->setExpanded(true);
    } else {
        ++m_modelCount;
        updateGeometryRootTitle();
        m_geometryRoot->setExpanded(true);
    }
    m_updatingChecks = false;
}

/// 移除模型树条目并更新分类数量及空态提示。
void ModelTreePanel::removeModel(int modelId)
{
    if (QTreeWidgetItem* node = modelItem(modelId)) {
        const bool wasMesh = node->parent() == m_meshSection;
        delete node;
        if (wasMesh) {
            --m_meshCount;
            updateMeshSectionTitle();
            if (m_meshCount == 0 && m_meshSection->childCount() == 0) {
                auto* placeholder = new QTreeWidgetItem(m_meshSection);
                placeholder->setText(0, tr("质量统计"));
                placeholder->setForeground(0, QBrush(kDimText));
                placeholder->setFlags(Qt::NoItemFlags);
                placeholder->setData(0, kRoleKind, KindPlaceholder);
            }
        } else {
            --m_modelCount;
            updateGeometryRootTitle();
        }
    }
    if (m_modelCount == 0 && m_geometryRoot->childCount() == 0) {
        auto* placeholder = new QTreeWidgetItem(m_geometryRoot);
        placeholder->setText(0, tr("导入的模型将显示在这里"));
        placeholder->setForeground(0, QBrush(kDimText));
        placeholder->setFlags(Qt::NoItemFlags);
        placeholder->setData(0, kRoleKind, KindPlaceholder);
        m_geometryRoot->setExpanded(false);
    }
}

/// 更新可见性勾选状态，避免产生重复用户操作信号。
void ModelTreePanel::setModelChecked(int modelId, bool checked)
{
    if (QTreeWidgetItem* node = modelItem(modelId)) {
        m_updatingChecks = true;
        node->setCheckState(0, checked ? Qt::Checked : Qt::Unchecked);
        m_updatingChecks = false;
    }
}

/// 按模型编号查找对应的树节点。
QTreeWidgetItem* ModelTreePanel::modelItem(int modelId) const
{
    for (int pass = 0; pass < 2; ++pass) {
        QTreeWidgetItem* section = pass == 0 ? m_geometryRoot : m_meshSection;
        for (int i = 0; i < section->childCount(); ++i) {
            QTreeWidgetItem* item = section->child(i);
            if (item->data(0, kRoleKind).toInt() == KindModel
                && item->data(0, kRoleModelId).toInt() == modelId)
                return item;
        }
    }
    return nullptr;
}

/// 将树节点点击转换为模型选择或占位项通知。
void ModelTreePanel::onItemClicked(QTreeWidgetItem* item, int column)
{
    Q_UNUSED(column)
    if (item->data(0, kRoleKind).toInt() == KindModel)
        emit modelNodeClicked(item->data(0, kRoleModelId).toInt());
    else
        emit placeholderClicked(item->text(0));
}

/// 将用户修改的勾选状态转换为可见性通知。
void ModelTreePanel::onItemChanged(QTreeWidgetItem* item, int column)
{
    if (m_updatingChecks || column != 0)
        return;
    if (item->data(0, kRoleKind).toInt() == KindModel) {
        emit modelVisibilityChanged(item->data(0, kRoleModelId).toInt(),
                                    item->checkState(0) == Qt::Checked);
    }
}

/// 为指定模型显示颜色、变换和删除操作菜单。
void ModelTreePanel::onContextMenuRequested(const QPoint& pos)
{
    QTreeWidgetItem* item = itemAt(pos);
    while (item && item->data(0, kRoleKind).toInt() != KindModel)
        item = item->parent();
    if (!item)
        return;

    QMenu menu(this);
    QAction* colorAction = menu.addAction(tr("设置颜色..."));
    menu.addSeparator();
    QAction* removeAction = menu.addAction(tr("移除该模型"));
    QAction* chosen = menu.exec(viewport()->mapToGlobal(pos));
    if (chosen == colorAction)
        emit modelColorChangeRequested(item->data(0, kRoleModelId).toInt());
    else if (chosen == removeAction)
        emit removeModelRequested(item->data(0, kRoleModelId).toInt());
}
