#pragma once
// 表面约束恢复：边细分后检查实际面片覆盖，必要时尝试安全 2→3/3→2 翻转。
// 共面片可使用不同对角线；边存在或另外两边存在不构成面覆盖证明。
// 当前不实现通用 PLC 填腔/任意面 Steiner 恢复，预算耗尽或面仍缺失明确失败。
// 约束边细分也可能因数值尺度/不可插入构型失败，不保证无条件收敛。
// 域提取由 VolumeDomain3D 单独执行，不能把恢复后的点集凸包当实体。

#include <array>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

#include "mesh/data/TetMesh3D.h"

namespace mesher {

struct Constrained3DResult
{
    bool ok = false;
    std::string error;
    int steinerPoints = 0;   ///< 插入的 Steiner 点数
};

/// 约束边恢复参数（默认值与既有行为一致 —— 功能不退步）。
struct EdgeRecoveryParams
{
    /// 是否启用「阶段 1：保护球」（Shewchuk 边界恢复两阶段算法的第一阶段，见
    /// docs/references/shewchuk-cdt-boundary-recovery.pdf p.8）。
    ///
    /// 阶段 1（本开关）：在每个「≥2 条约束段相交」的输入顶点 v_i 处放保护球
    ///   r_i = min{ lfs(v_i), ℓ_i/3, 2·d_i/3 }，对每条相邻段插入点 s ∩ S_i。
    /// 阶段 2（始终启用，原有行为）：对仍断开的约束边插中点二分。
    ///
    /// 启用「阶段 1：保护球」（Shewchuk 边界恢复第一阶段，论文 §3/§5）。
    ///
    /// 论文定位（§3 Theorem 1）：保护球是「让 PLC 达成 edge-protected
    /// （所有**段**强 Delaunay）」，从而使 **CDT 存在**的预处理手段；
    /// 不是"把断边变成网格边"的手段。
    ///
    /// 实现要点（§5 weak edge protection）：
    ///   · **只对 grazeable 段**放保护球 ——
    /// 弱边保护仅考虑可掠段，其他段按不存在处理（译意）。
    ///   · grazeable 判据（§3）：段被 <2 个 facet 包含，或绕段的旋转顺序中
    ///     相邻两面内角 >= 180°。面对角线（同一 facet 内部、两三角面共面）
    ///     **不是 PLC 段**（§1 p.6），一并排除。
    ///   · 半径 r_i = min{ lfs(v_i), ℓ_i/3, 2·d_i/3 }（§5 p.7）。
    ///
    /// 实测（cube.stl / cube10.stl，2026-09-24）：
    ///   `segments=18 grazeable=0` → `candidates=0` → 保护球正确地**不插入
    ///   任何点**，端到端结果与关闭该开关**逐位一致**。
    /// 这与论文原文完全吻合：
    /// 论文示例（译意）：规则立方体单元复形中不存在可掠段，cube/cube10 对应该情形。
    ///
    /// 反例（也已在代码注释中留档）：若**不做** grazeable 过滤，会插 36 个
    /// 多余点（含 6 条面对角线），导致阶段 2 在 (49,115)/(49,135) 上
    /// `cavity volume not conserved` —— 连最简单的 cube.stl 都会失败。
    /// 这是"对非 grazeable 段也做保护"的直接后果，与论文警告一致。
    ///
    /// 默认 false：保护球属于实验能力；以下 cube 结论是历史测量。
    /// 打开后对 cube/cube10 无行为变化（grazeable 为空）；对含小角度的
    /// 真实 PLC 才有实际作用。
    bool edgeProtection = false;
};

/// 恢复表面三角形约束。faces 的顶点索引须已在 mesh.points 中
///（由 delaunay3D 加入），面方向任意（内部归一为无向）。
/// rp 为阶段 1 开关（默认关闭 = 与既有行为一致）；outProtect 为诊断计数。
Constrained3DResult enforceSurface3D(TetMesh3D& mesh,
                                     const std::vector<std::array<int, 3>>& faces,
                                     std::string* error,
                                     const EdgeRecoveryParams& rp = EdgeRecoveryParams{},
                                     int* outProtect = nullptr);

/// 手动 cavity 重建插入点 m（细化阶段复用：分裂约束边/面）。
/// excludeFaceKeys：重建时剔除的面（目标面被细分，不重建）。
/// excludeEdges：重建时剔除含这些边的面（m 恰在这些边上或与边共面，重建会退化）。
bool insertConstrainedPoint(TetMesh3D& mesh, const Point3D& m,
                            const std::unordered_set<uint64_t>& excludeFaceKeys,
                            const std::unordered_set<uint64_t>& excludeEdges,
                            std::string* error);

/// 约束边恢复：对 edges 中每条尚未成为网格边的约束边，插入中点细分，
/// 直到全部成为网格边（或达到迭代上限）。
///
/// ★ §8「back out」（失败退让）★
/// ------------------------------------------------------------
/// `insertConstrainedPoint` 在**修改网格之前**做体积守恒校验，失败时 mesh
/// 保持原样 —— 它天然是一个「安全拒绝」原语。论文 §8 要求：顶点插入不可能
/// 成功时，程序必须**检测并退回**，而不是把整条恢复链拖死：
/// 论文要求（译意）：检查插点是否可行，无法成功时退回。
///
/// 退让的两种形态（均已实现）：
///   · 阶段 1 保护球点不可插入 → 改用**所在段的中点**（§8 原文：
/// 论文退让方法（译意）：保护球插点受阻时二分线段；若中点仍无法插入，则继续后续恢复。
///     仅记录并继续（保护球是"优化"而非"必需"，阶段 2 兜底）。
///   · 阶段 2 中点细分不可插入 → 把该条边移出待处理列表、换下一条继续。
///     由于每轮扫描**从索引 0 全覆盖复查**，被跳过的边下轮会再次被看到；
///     只有**一整轮**都找不到可细分边时才真正退出 → 一次退让不会漏掉可恢复
///     的边。退让总数超过 `|pendingEdges| + 16` 才整体报错（区分「个别构型
///     不可插入」与「整体不可插入」）。
///
/// 诊断开关（只读，默认关闭，不改变算法行为）：
///   MESHSTUDIO_DEBUG_BACKOUT=1  打印每次退让的边与原因
///   MESHSTUDIO_DEBUG_PROTECT=1  打印保护球候选/成功/失败计数与 grazeable 统计
///
/// 为什么细化阶段必须复用：insertConstrainedPoint 的手动 cavity 重建会**删除**
/// 该点周围所有含 m 的四面体并重建。若某条已恢复的约束边 (u,v) 属于被删除的
/// cavity，且 (u,v) 恰好穿过重建后的某条新面，它就不再是网格边 —— 该约束边的
/// 两条子段面片随之消失，表面出现缝隙 → **覆盖域/体积真实改变**。
/// 这是 refine 阶段体积漂移的主因（另一部分来自零体积薄片清理）。
/// 每次分裂后调用本函数，可把被破坏的约束边重新细分恢复，保持边界严格不变。
///
/// **edges 是输入输出参数**：函数返回时被替换为最终的子段列表（细分后的
/// (u,m)(m,v) 等）。调用方必须接收写回结果 —— 否则会继续拿「已被细分过的
/// 旧边」再分裂，导致中点重复插入与真实约束棱永久失同步。
///
/// outSteiner（可空）返回本次插入的 Steiner 点数。
/// outProtect（可空）返回其中来自「阶段 1 保护球」的点数（诊断用）。
/// rp 为阶段 1 开关（默认关闭，见 EdgeRecoveryParams）。
bool recoverConstraintEdges(TetMesh3D& mesh,
                            const std::vector<std::array<int, 3>>& constraintFaces,
                            std::vector<std::pair<int, int>>& edges,
                            int* outSteiner,
                            std::string* error,
                            const EdgeRecoveryParams& rp = EdgeRecoveryParams{},
                            int* outProtect = nullptr);

} // namespace mesher
