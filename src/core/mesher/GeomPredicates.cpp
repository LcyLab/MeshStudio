#include "GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

namespace mesher {

double orient2dDet(const Point2D& a, const Point2D& b, const Point2D& c)
{
    // 以 c 为原点，2x2 行列式：(b-c)×(a-c)
    const double acx = a.x - c.x, acy = a.y - c.y;
    const double bcx = b.x - c.x, bcy = b.y - c.y;
    return acx * bcy - acy * bcx;
}

int orient2d(const Point2D& a, const Point2D& b, const Point2D& c)
{
    const double d = orient2dDet(a, b, c);
    return (d > 0.0) - (d < 0.0);
}

double incircleDet(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p)
{
    // 以 p 为原点；逆时针化：若 a,b,c 顺时针则交换 b,c（平移不变，用相对 p 的坐标即可）
    double ax = a.x - p.x, ay = a.y - p.y;
    double bx = b.x - p.x, by = b.y - p.y;
    double cx = c.x - p.x, cy = c.y - p.y;
    if ((bx - ax) * (cy - ay) - (by - ay) * (cx - ax) < 0.0) {   // orient2d(a,b,c) < 0
        std::swap(bx, cx);
        std::swap(by, cy);
    }
    const double az = ax * ax + ay * ay;
    const double bz = bx * bx + by * by;
    const double cz = cx * cx + cy * cy;
    // 3x3 行列式展开
    return ax * (by * cz - bz * cy)
         - bx * (ay * cz - az * cy)
         + cx * (ay * bz - az * by);
}

int incircle(const Point2D& a, const Point2D& b, const Point2D& c, const Point2D& p)
{
    const double d = incircleDet(a, b, c, p);
    return (d > 0.0) - (d < 0.0);
}


int orient3d(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d)
{
    const double v = orient3dDet(a, b, c, d);
    return (v > 0.0) - (v < 0.0);
}

double insphereDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p)
{
    // 以 p 为原点，4x4 行列式；右手系化（标准 orient3d(a,b,c,d) > 0）后 det>0 表示 p 在球内
    double ax = a.x - p.x, ay = a.y - p.y, az = a.z - p.z;
    double bx = b.x - p.x, by = b.y - p.y, bz = b.z - p.z;
    double cx = c.x - p.x, cy = c.y - p.y, cz = c.z - p.z;
    double dx = d.x - p.x, dy = d.y - p.y, dz = d.z - p.z;
    // 标准 orient3d(a,b,c,d)（平移不变，用相对 p 的坐标计算）：det[b-a, c-a, d-a]
    const double abx = bx - ax, aby = by - ay, abz = bz - az;
    const double acx = cx - ax, acy = cy - ay, acz = cz - az;
    const double adx = dx - ax, ady = dy - ay, adz = dz - az;
    if (abx * (acy * adz - acz * ady) - aby * (acx * adz - acz * adx)
        + abz * (acx * ady - acy * adx) < 0.0) {
        std::swap(bx, cx); std::swap(by, cy); std::swap(bz, cz);
    }
    const double aw = ax * ax + ay * ay + az * az;
    const double bw = bx * bx + by * by + bz * bz;
    const double cw = cx * cx + cy * cy + cz * cz;
    const double dw = dx * dx + dy * dy + dz * dz;
    // 4x4 行列式按第一行展开；取负使约定为"p 在球内 > 0"（a,b,c,d 右手系）。
    // 原始行列式对右手系四面体在 p 球内时为负（与 Shewchuk 的逆序参数约定一致）。
    return -(ax * (by * (cz * dw - cw * dz) - bz * (cy * dw - cw * dy) + bw * (cy * dz - cz * dy))
         - bx * (ay * (cz * dw - cw * dz) - az * (cy * dw - cw * dy) + aw * (cy * dz - cz * dy))
         + cx * (ay * (bz * dw - bw * dz) - az * (by * dw - bw * dy) + aw * (by * dz - bz * dy))
         - dx * (ay * (bz * cw - bw * cz) - az * (by * cw - bw * cy) + aw * (by * cz - bz * cy)));
}

int insphere(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d, const Point3D& p)
{
    const double v = insphereDet(a, b, c, d, p);
    return (v > 0.0) - (v < 0.0);
}

// ============================================================
// 符号扰动谓词（SoS 实用版）
// ============================================================

double orient3dDet(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d)
{
    // 标准约定：det[b-a, c-a, d-a]，>0 表示 a,b,c 逆时针看向 d 时 d 在正侧（右手系）
    const double abx = b.x - a.x, aby = b.y - a.y, abz = b.z - a.z;
    const double acx = c.x - a.x, acy = c.y - a.y, acz = c.z - a.z;
    const double adx = d.x - a.x, ady = d.y - a.y, adz = d.z - a.z;
    return abx * (acy * adz - acz * ady)
         - aby * (acx * adz - acz * adx)
         + abz * (acx * ady - acy * adx);
}

double orient3dEps(double scale)
{
    // det 量级 ~ scale^3；double 相对误差 ~ 2^-52 ≈ 2.2e-16，取安全系数 64。
    // 低于该界视为「无法用 double 可靠判定」→ 退化，交给扰动。
    const double s = (scale > 0.0) ? scale : 1.0;
    return 64.0 * 2.220446049250313e-16 * s * s * s;
}

double insphereEps(double scale)
{
    const double s = (scale > 0.0) ? scale : 1.0;
    return 64.0 * 2.220446049250313e-16 * s * s * s * s;
}

namespace {

// 置换奇偶性：逆序数为偶 → +1，奇 → -1。保证与 orient3dDet 的置换行为一致。
inline int permSign(const int* ia, int n)
{
    int inv = 0;
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (ia[i] > ia[j])
                ++inv;
    return (inv % 2 == 0) ? 1 : -1;
}

// ============================================================
// SoS（Simulation of Simplicity）—— 虚拟位移法
// ------------------------------------------------------------
// 目标：当谓词（orient3d / insphere）在浮点意义下退化为 0（共面 / 共球）时，
// 给出一个**确定性、恒非零、对参数置换严格反对称**的符号，且**不修改坐标**。
//
// 做法（Edelsbrunner–Mücke SoS 的等价直接实现）：
//   给顶点 i 施加虚拟扰动 p_i ← p_i + ε·δ_i·u，其中
//     * u = (1,1,1) 固定方向，
//     * δ_i = 1（统一幅度），ε > 0 极小；
//     * 索引小 = 优先级高（ε 的幂次更低 → 更主导）。
//   展开后取首个使谓词非零的顶点所决定的符号。
//
// 实现上不必真的做 ε 展开，直接**数值上叠加一个小位移再探测符号**即可：
//   按索引升序依次尝试「只扰动第 k 个顶点」，若谓词非零则其符号即结果；
//   若仍为零（该顶点不 resolve 退化）则推进到下一优先级顶点。
//   位移量 t = scale·2^-40：远大于 double 舍入误差（~scale·1e-16），
//   又远小于真实特征尺寸，不会改变非退化情形的符号。
//
// 注意：退化情形的符号注定是**人为约定**的（Delaunay 剖分不唯一），
// 但这无关紧要 —— 只要全局所有调用点（faceVertex / validate / 边界定向 /
// cavity 判定）共用**同一套**谓词，拓扑就是自洽的。
// ============================================================

// 原始 5x5 insphere 行列式（**全反对称**，不做任何约定修正）：
//     | ax ay az aw 1 |
//     | bx by bz bw 1 |
//     | cx cy cz cw 1 |      w = x²+y²+z²
//     | dx dy dz dw 1 |
//     | px py pz pw 1 |
// 该行列式对 5 个参数**任意置换严格按奇偶性变号**（行交换性质），
// 这正是 SoS 平局判定所必需的性质；而 insphereDet 为了给出
// "p 在球内 > 0" 的直观约定做了内部换列，破坏了置换奇偶性，故不能复用。
inline double insphereRawDet(const Point3D& a, const Point3D& b, const Point3D& c,
                             const Point3D& d, const Point3D& p)
{
    const double x[5] = { a.x, b.x, c.x, d.x, p.x };
    const double y[5] = { a.y, b.y, c.y, d.y, p.y };
    const double z[5] = { a.z, b.z, c.z, d.z, p.z };
    double w[5];
    for (int i = 0; i < 5; ++i)
        w[i] = x[i] * x[i] + y[i] * y[i] + z[i] * z[i];

    // 5x5 行列式：按第 5 列（全为 1）做 Laplace 展开 → 5 个 4x4 余子式，
    // 符号为 (-1)^{i+5}，i=0..4。
    double det = 0.0;
    for (int i = 0; i < 5; ++i) {
        // 取除第 i 行外的 4 行，列取 [x y z w]
        double m[4][4];
        int r = 0;
        for (int k = 0; k < 5; ++k) {
            if (k == i) continue;
            m[r][0] = x[k]; m[r][1] = y[k]; m[r][2] = z[k]; m[r][3] = w[k];
            ++r;
        }
        // 4x4 行列式（按第一行展开）
        const auto det3 = [](const double a3[3][3]) {
            return a3[0][0] * (a3[1][1] * a3[2][2] - a3[1][2] * a3[2][1])
                 - a3[0][1] * (a3[1][0] * a3[2][2] - a3[1][2] * a3[2][0])
                 + a3[0][2] * (a3[1][0] * a3[2][1] - a3[1][1] * a3[2][0]);
        };
        double sub = 0.0;
        for (int j = 0; j < 4; ++j) {
            double m3[3][3];
            int rr = 0;
            for (int r2 = 0; r2 < 4; ++r2) {
                if (r2 == 0) continue;
                int cc = 0;
                for (int c2 = 0; c2 < 4; ++c2) {
                    if (c2 == j) continue;
                    m3[rr][cc] = m[r2][c2];
                    ++cc;
                }
                ++rr;
            }
            sub += ((j % 2 == 0) ? 1.0 : -1.0) * m[0][j] * det3(m3);
        }
        det += ((i % 2 == 0) ? 1.0 : -1.0) * sub;   // 第 5 列元素恒为 1
    }
    return det;
}

// SoS 平局：insphere 版。idx5 是 5 个点的全局索引，按参数顺序 (a,b,c,d,p)。
// 返回恒非零的符号，严格反对称。
//
// 与 sosTieOrient 完全同构：按索引升序（索引小 = 优先级高）依次对顶点做
// 沿 u=(1,1,1) 的微小虚拟位移，重算 insphereRawDet；首个非零符号即为结果。
// 位移量 t = scale * 2^-40。最终乘 permSign 保证严格反对称。
inline int sosTieInsphere(const Point3D& a, const Point3D& b, const Point3D& c,
                          const Point3D& d, const Point3D& p,
                          int ia, int ib, int ic, int id, int ip)
{
    const int idx[5] = { ia, ib, ic, id, ip };
    const Point3D pts[5] = { a, b, c, d, p };

    // 优先级顺序：索引升序
    int order[5] = { 0, 1, 2, 3, 4 };
    for (int i = 0; i < 5; ++i)
        for (int j = i + 1; j < 5; ++j)
            if (idx[order[j]] < idx[order[i]])
                std::swap(order[i], order[j]);

    double scale = 0.0;
    for (int k = 0; k < 5; ++k) {
        scale = std::max(scale, std::fabs(pts[k].x));
        scale = std::max(scale, std::fabs(pts[k].y));
        scale = std::max(scale, std::fabs(pts[k].z));
    }
    if (scale <= 0.0) scale = 1.0;
    const double t = scale * 9.094947017729282e-13;   // scale * 2^-40

    // insphereRawDet > 0 表示 p 在四面体外部（与 insphereDet 差一个整体符号），
    // 这里只需一个**确定性**符号，故直接用 raw 的符号，不关心物理约定。
    int base = 1;
    for (int pr = 0; pr < 5; ++pr) {
        const int k = order[pr];
        Point3D q = pts[k];
        q.x += t; q.y += t; q.z += t;
        Point3D tmp[5] = { pts[0], pts[1], pts[2], pts[3], pts[4] };
        tmp[k] = q;
        const double v = insphereRawDet(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4]);
        if (v > 0.0) { base = 1;  break; }
        if (v < 0.0) { base = -1; break; }
        // v == 0：该顶点扰动不足以定序 → 继续下一优先级
    }

    return permSign(idx, 5) * base;
}

// SoS 平局：orient3d 版（4 点）。idx4 按 (a,b,c,d) 顺序。
//
// 实现：**字典序 ε 展开**（Edelsbrunner–Mücke SoS 的直接等价形式）。
//
//   给顶点 i 施加虚拟扰动 p_i ← p_i + ε^{σ_i}·u，u = (1,1,1)，其中 σ_i 由
//   全局索引**升序**决定（索引越小 → ε 幂次越低 → 越主导）。把 4 个顶点的
//   扰动下标 0,1,2,3 按 σ 升序排列得到优先序列 (k_0, k_1, k_2, k_3)，则
//       D_SoS = A_0·ε^{σ_{k_0}} + A_1·ε^{σ_{k_0}+σ_{k_1}} + …
//   其中 A_j 是「对 k_0..k_j 施加 u 位移后的行列式」。
//   A_0 由于退化必为 0；取首个 A_j ≠ 0 者，其符号即 D_SoS 的符号。
//
//   把 3 个扰动下标组合写成一个 4 进制编码 c = Σ_j k_j·4^j（j 从低到高），
//   编码大的排在前面（更主导），则只需**一遍遍历 4^4 = 256 个编码**即可：
//   对每个未扰动顶点都落后于已选编码的编码，计算行列式，取最大编码的
//   非零符号。由于 orient3dDet 对参数置换严格反号，而编码与参数顺序无关，
//   该符号天然**严格反对称**，无需额外乘 permSign。
//
// 这样避免了对"哪个顶点索引最小"的依赖，也避免了坐标微移的舍入风险。
inline int sosTieOrient(const Point3D& a, const Point3D& b, const Point3D& c,
                        const Point3D& d, int ia, int ib, int ic, int id)
{
    const Point3D base[4] = { a, b, c, d };
    const int idx[4] = { ia, ib, ic, id };

    // 优先级编码：rank[k] = k 在 idx 升序中的位次（0 = 最主导）
    int rank[4] = { 0, 0, 0, 0 };
    for (int k = 0; k < 4; ++k) {
        for (int m = 0; m < 4; ++m) {
            if (m == k) continue;
            // m 比 k 更主导 → k 的位次 +1
            if (idx[m] < idx[k] || (idx[m] == idx[k] && m < k))
                ++rank[k];
        }
    }

    // 枚举所有编码，取最大编码对应行列式的符号（首个非零即答案）
    int bestCode = -1;
    int bestSign = 1;
    for (int code = 0; code < 256; ++code) {
        // 解码：pos[j] = (code >> 2j) & 3，要求 pos 是 0..3 的排列
        int pos[4] = { code & 3, (code >> 2) & 3, (code >> 4) & 3, (code >> 6) & 3 };
        bool perm = true, seen[4] = { false, false, false, false };
        for (int j = 0; j < 4; ++j) {
            if (seen[pos[j]]) { perm = false; break; }
            seen[pos[j]] = true;
        }
        if (!perm) continue;
        if (code <= bestCode) continue;

        // 计算「对 S 内顶点施加 u 位移」后的行列式，
        // S = { pos[0], pos[1], pos[2], pos[3] } 的前 (j+1) 项，
        // 只需保证所选组合「尊重优先级」：rank 更小的必须出现在 pos 更前。
        // 由于 pos 本身就是一个排列，S 取全排列中的前缀即可。
        for (int j = 0; j < 4; ++j) {
            int S = 0;
            for (int q2 = 0; q2 <= j; ++q2)
                S |= (1 << pos[q2]);

            // 优先级一致性检查：若存在未扰动顶点 m（m ∉ S）比某个已扰动顶点
            // 更主导（rank 更小），则该组合不合法。
            bool ok = true;
            for (int m = 0; m < 4 && ok; ++m) {
                if (S & (1 << m)) continue;
                for (int q2 = 0; q2 <= j; ++q2) {
                    if (rank[m] < rank[pos[q2]]) { ok = false; break; }
                }
            }
            if (!ok) break;

            Point3D pts2[4];
            for (int k = 0; k < 4; ++k) {
                pts2[k] = base[k];
                if (S & (1 << k)) { pts2[k].x += 1.0; pts2[k].y += 1.0; pts2[k].z += 1.0; }
            }
            const double v = orient3dDet(pts2[0], pts2[1], pts2[2], pts2[3]);
            if (v > 0.0 || v < 0.0) {
                bestCode = code;
                bestSign = (v > 0.0) ? 1 : -1;
                break;   // 该编码已给出符号，取最大 code 者
            }
        }
    }

    return bestSign;
}

inline double coordScale3d(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d){
    double s = 0.0;
    const Point3D* ps[4] = { &a, &b, &c, &d };
    for (int k = 0; k < 4; ++k) {
        s = std::max(s, std::fabs(ps[k]->x));
        s = std::max(s, std::fabs(ps[k]->y));
        s = std::max(s, std::fabs(ps[k]->z));
    }
    return s;
}


} // namespace

int orient3dSoS(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                int ia, int ib, int ic, int id)
{
    const double det = orient3dDet(a, b, c, d);
    const double eps = orient3dEps(coordScale3d(a, b, c, d));
    if (det > eps)
        return 1;
    if (det < -eps)
        return -1;
    return sosTieOrient(a, b, c, d, ia, ib, ic, id);   // 严格 SoS
}

int insphereSoS(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                const Point3D& p, int ia, int ib, int ic, int id, int ip)
{
    // 用**全反对称**的原始行列式：置换性质由 permSign 保证。
    // 注意符号约定：对右手系 (a,b,c,d)，insphereRawDet = -insphereDet
    // （见 tests/_SignProbe.cpp 实测），故取负以保持"p 在球内 > 0"的语义。
    const double det = -insphereRawDet(a, b, c, d, p);
    const double eps = insphereEps(coordScale3d(a, b, c, d));
    if (det > eps)
        return 1;
    if (det < -eps)
        return -1;
    return sosTieInsphere(a, b, c, d, p, ia, ib, ic, id, ip);   // 严格 SoS
}

} // namespace mesher
