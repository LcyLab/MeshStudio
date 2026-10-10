#pragma once
/** @file Mesh2DTransaction.h
 * @brief 对局部三角形、邻接和半边记账；失败逆向恢复，提交公布编辑及搬移。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include "Mesh2D.h"
#include <array>
#include <unordered_set>
namespace mesher {
struct TriangleMove { int from=-1,to=-1; };
struct MeshEditEvent {
    enum class Kind { Created,Removed,Moved };
    Kind kind; int slot=-1,other=-1;
};
// 紧凑删除的编号会立即复用；events 按顺序解释，其他列表仅作摘要。
struct MeshEditDelta {
    size_t initialPoints=0,initialTriangles=0;
    std::vector<MeshEditEvent> events;
    std::vector<int> createdSlots,deletedSlots,changedSlots;
    std::vector<TriangleMove> moves;
    // 按删除顺序保留当时的节点连接（含事务内新建后删除的片），不保存节点属性。
    // 插点先删后建，可据此比较实际替换区域，避免紧凑槽位搬移丢失原片。
    std::vector<std::array<int,3>> removedTriangles;
};
class Mesh2DTransaction {
    friend class Mesh2D;
    Mesh2D& mesh;
    MeshObservation observation;
    MeshEditDelta delta;
    struct TriUndo { Tri2D triangle; std::uint64_t version; };
    std::unordered_map<int,TriUndo> triangles;
    std::unordered_set<int> changed;
    using Node=std::unordered_map<std::uint64_t,int>::node_type;
    struct EdgeUndo { std::uint64_t key; Node original; bool captured=false; };
    std::vector<EdgeUndo> halfedges;
    std::unordered_set<std::uint64_t> seenEdges;
    std::vector<std::pair<double,double>> addedPointKeys;
    bool finished=false;
    /// 记录三角形被修改，供版本更新或事务回滚使用。
    void touchTriangle(int slot);
    /// 在修改前保存半边索引，供事务回滚使用。
    void touchHalfedge(std::uint64_t key);
    /// 记录事务中新建的三角形。
    void created(int slot);
    /// 记录事务中被移除的三角形及其原状态。
    void removed(int slot,int last);
public:
    /// 建立局部网格编辑事务；禁止复制以保持唯一回滚责任。
    explicit Mesh2DTransaction(Mesh2D& mesh,MeshObservation observation={});
    /// 事务未提交时恢复修改前的网格状态。
    ~Mesh2DTransaction();
    /// 建立局部网格编辑事务；禁止复制以保持唯一回滚责任。
    Mesh2DTransaction(const Mesh2DTransaction&)=delete;
    /// 禁止复制事务，避免多个对象重复提交或回滚。
    Mesh2DTransaction& operator=(const Mesh2DTransaction&)=delete;
    /// 编辑前网格须有效，编辑期间只通过 Mesh2D 方法改存储；不嵌套事务。
    bool validate(std::string* error=nullptr,const std::vector<std::pair<int,int>>& protectedEdges={}) const;
    /// 发布网格变化并结束事务，保留已经完成的修改。
    bool commit(std::string* error=nullptr,MeshEditDelta* published=nullptr,
                const std::vector<std::pair<int,int>>& protectedEdges={});
    /// 返回尚未提交的局部网格变化记录。
    const MeshEditDelta& pendingDelta() const { return delta; }
    /// 保留旧 map 节点、向量容量；回滚不重新分配节点，版本序号不倒退。
    void rollback() noexcept;
};
} // namespace mesher
