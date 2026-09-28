#pragma once
// ============================================================
// Delaunay3D —— Bowyer-Watson 增量 Delaunay 四面体化（M1 核心）
// 职责：
//   1. delaunay3D()：对点集生成 Delaunay 四面体化（super-tet 包裹 + 逐个插入 + 清理）
//   2. insertPoint3D()：单点插入既有四面体网格，供细化阶段（Ruppert3D）复用
// 数值处理：内部先把坐标平移缩放归一化 + 符号扰动（破坏共面/共球退化），
// 完成后还原为原始坐标。四个 super 顶点会残留在 points 中但无四面体引用。
// 流程与 2D 版 Delaunay2D 一一对应（walk 定位 → cavity → 删除重建）。
// ============================================================

#include <cstdint>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "TetMesh3D.h"

namespace mesher {

struct Delaunay3DResult
{
    bool ok = false;
    std::string error;
    std::vector<int> tetIds;     ///< 有效四面体 id（super 相关已剔除）
    long long insertSteps = 0;   ///< 插入循环总步数（性能观测）
};

/// 对 point 集合生成 Delaunay 四面体化。点被追加进 mesh.points。
///
/// ★ verifyEachStep：每次插入后做一次全网格面哈希自检（O(T)）★
/// ------------------------------------------------------------
/// 该自检的唯一价值是**定位**"第几步把拓扑弄坏了"（错误信息带 step 号）。
/// 默认关闭 —— 它是 O(n·T) 的二次方开销，实测在 3850 个点的球面 B-Rep
/// 离散输入上把 delaunay3D 从秒级拖到分钟级（n=3850、T 峰值 ~2 万，
/// 每次 checkFaceHash 要遍历 4T 个面并做哈希查找 → 约 3×10⁸ 次哈希操作）。
///
/// 打开方式：传 verifyEachStep=true，或设环境变量
/// `MESHSTUDIO_DELAUNAY_VERIFY_EACH_STEP=1`（便于不改代码做诊断）。
///
/// 无论开关如何，**插入结束后始终执行 `mesh.validate()`** —— 正确性兜底不变，
/// 只是失去"哪一步坏的"这个定位信息。
Delaunay3DResult delaunay3D(TetMesh3D& mesh, std::vector<Point3D> points,
                            bool verifyEachStep = false);

/// 单点插入既有网格（细化阶段复用，坐标用原始坐标系）。
/// blockedFaces：约束面（无向 key，顶点排序），cavity 收集时不得跨越
///（插入点不得破坏约束面）。
/// startTet 为 walk 定位起点；outNewVertex 输出新顶点索引（失败时为 -1）。
bool insertPoint3D(TetMesh3D& mesh, const Point3D& p,
                   const std::unordered_set<uint64_t>& blockedFaces,
                   int startTet, int* outNewVertex, int* outTet,
                   std::string* error);

/// 清除零体积（共面/共球退化）四面体。
/// 做法：对每个退化四面体，收集其「星形 cavity」= 该四面体 ∪ 所有以它的面为界、
/// 且把这些面「撑开」的邻居，然后删除 cavity 并按 cavity 边界面重新四面体化。
/// 退化四面体体积为 0，删除不改变总体积。
/// 返回清除的退化四面体数；cavity 无法安全重建时保留原状并计数为失败。
int pruneDegenerateTets(TetMesh3D& mesh, std::string* error);

} // namespace mesher
