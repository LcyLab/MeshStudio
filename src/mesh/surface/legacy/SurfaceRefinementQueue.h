#pragma once
/** @file SurfaceRefinementQueue.h
 * @brief 面内坏片的版本化优先队列，局部失效和有界历史压缩。
 * @date 2026-10-02
 * @modified 2026-10-10
 */
#include "mesh/data/Mesh2DTransaction.h"
#include <array>
#include <queue>
namespace mesher {
struct SurfaceTriangleTicket {
    int face=-1,slot=-1;
    std::uint64_t topologyVersion=0,criteriaVersion=0;
    int priorityClass=0; // 2=翻折/硬几何；1=质量软目标。同类按超限比处理。
    double violation=0;
    std::array<int,3> vertexKey{}; // 排序后的稳定顶点编号。
};
class SurfaceRefinementQueue {
    struct LowerPriority {
        /// 按任务类别、违反程度和稳定身份比较细化优先级。
        bool operator()(const SurfaceTriangleTicket&,const SurfaceTriangleTicket&) const;
    };
    int face;
    std::uint64_t criteria;
    MeshObservation observation;
    std::priority_queue<SurfaceTriangleTicket,std::vector<SurfaceTriangleTicket>,LowerPriority> heap;
    std::unordered_map<int,SurfaceTriangleTicket> active;
    size_t peak=0,compressed=0;
    /// 在陈旧条目过多时由有效任务重建队列。
    void compact();
public:
    /// 绑定 CAD 面和准则版本，建立细化任务队列。
    explicit SurfaceRefinementQueue(int face,std::uint64_t criteriaVersion,MeshObservation observation={});
    /// 校验并登记细化任务，同时更新槽位的有效任务。
    bool offer(const SurfaceTriangleTicket&,std::string* error);
    /// 取出最高优先级的有效任务，跳过版本过期条目。
    bool pop(const Mesh2D&,SurfaceTriangleTicket&);
    /// 移除局部拓扑编辑影响的任务并按需压缩队列。
    void invalidate(const MeshEditDelta&);
    /// 更新细化准则版本并清空原有效任务。
    void resetCriteria(std::uint64_t version);
    /// 返回当前有效细化任务数量。
    size_t activeSize() const { return active.size(); }
    /// 返回包含陈旧项在内的队列条目数量。
    size_t heapSize() const { return heap.size(); }
    /// 返回运行期间队列条目数量的峰值。
    size_t peakEntries() const { return peak; }
    /// 返回队列压缩次数。
    size_t compactions() const { return compressed; }
};
} // namespace mesher
