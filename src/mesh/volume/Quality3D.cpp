// 文件职责：四面体质量度量：统一计算二面角、半径边比、归一化 Jacobian 和退化统计，供细化与验收复用。
#include "Quality3D.h"

#include "geometry/predicates/GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mesher {

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 计算两点之间的三维欧氏距离。
inline double dist3(const Point3D& a, const Point3D& b)
{
    const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 外接球心（克拉默）；共面返回 false
bool circumsphere3D(const Point3D& a, const Point3D& b, const Point3D& c, const Point3D& d,
                    Point3D& o, double& R)
{
    double M[3][3], rhs[3];
    M[0][0] = b.x - a.x; M[0][1] = b.y - a.y; M[0][2] = b.z - a.z;
    rhs[0] = 0.5 * (M[0][0] * M[0][0] + M[0][1] * M[0][1] + M[0][2] * M[0][2]);
    M[1][0] = c.x - a.x; M[1][1] = c.y - a.y; M[1][2] = c.z - a.z;
    rhs[1] = 0.5 * (M[1][0] * M[1][0] + M[1][1] * M[1][1] + M[1][2] * M[1][2]);
    M[2][0] = d.x - a.x; M[2][1] = d.y - a.y; M[2][2] = d.z - a.z;
    rhs[2] = 0.5 * (M[2][0] * M[2][0] + M[2][1] * M[2][1] + M[2][2] * M[2][2]);
    const double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                     - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                     + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    if (std::fabs(det) < 1e-300)
        return false;
    const double detX = rhs[0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
                      - M[0][1] * (rhs[1] * M[2][2] - M[1][2] * rhs[2])
                      + M[0][2] * (rhs[1] * M[2][1] - M[1][1] * rhs[2]);
    const double detY = M[0][0] * (rhs[1] * M[2][2] - M[1][2] * rhs[2])
                      - rhs[0] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
                      + M[0][2] * (M[1][0] * rhs[2] - rhs[1] * M[2][0]);
    const double detZ = M[0][0] * (M[1][1] * rhs[2] - rhs[1] * M[2][1])
                      - M[0][1] * (M[1][0] * rhs[2] - rhs[1] * M[2][0])
                      + rhs[0] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    o.x = detX / det;
    o.y = detY / det;
    o.z = detZ / det;
    R = std::sqrt(o.x * o.x + o.y * o.y + o.z * o.z);
    o.x += a.x; o.y += a.y; o.z += a.z;
    return std::isfinite(R) && R > 0.0;
}

} // namespace

/// 单个四面体质量（共面/退化时 degenerate = true）。
TetQuality tetQuality(const TetMesh3D& mesh, const Tet4& t, double edgeScale)
{
    TetQuality q;
    const Point3D& A = mesh.points[t.v[0]];
    const Point3D& B = mesh.points[t.v[1]];
    const Point3D& C = mesh.points[t.v[2]];
    const Point3D& D = mesh.points[t.v[3]];

    const double vol6 = orient3dDet(A, B, C, D);   // 6 倍有向体积
    q.volume = vol6 / 6.0;
    const double scale3 = edgeScale * edgeScale * edgeScale;
    q.degenerate = std::fabs(vol6) < 1e-6 * scale3;

    // 最短边与半径边比
    double lmin = std::numeric_limits<double>::max();
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j)
        {
            const double length = dist3(mesh.points[t.v[i]], mesh.points[t.v[j]]);
            lmin = std::min(lmin, length);
            q.maxEdgeLength = std::max(q.maxEdgeLength, length);
        }
    Point3D o;
    double R = 0.0;
    q.radiusEdge = std::numeric_limits<double>::infinity();
    if (lmin > 0.0 && circumsphere3D(A, B, C, D, o, R))
        q.radiusEdge = R / lmin;
    else
        q.degenerate = true;

    double scaledJacobian = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 4; ++i) {
        const Point3D& p = mesh.points[t.v[i]];
        double denominator = 1.0;
        for (int j = 0; j < 4; ++j)
            if (j != i) denominator *= dist3(p, mesh.points[t.v[j]]);
        const double value = denominator > 0.0 ? std::sqrt(2.0) * vol6 / denominator : 0.0;
        scaledJacobian = std::min(scaledJacobian, value);
    }
    q.scaledJacobian = scaledJacobian;

    // 最小二面角：4 个面外法向两两夹角取内角
    Point3D n[4];
    double area = 0.0;
    for (int e = 0; e < 4; ++e) {
        const Point3D& P = mesh.points[t.v[(e + 1) % 4]];
        const Point3D& Q = mesh.points[t.v[(e + 2) % 4]];
        const Point3D& S = mesh.points[t.v[(e + 3) % 4]];
        n[e] = { (Q.y - P.y) * (S.z - P.z) - (Q.z - P.z) * (S.y - P.y),
                 (Q.z - P.z) * (S.x - P.x) - (Q.x - P.x) * (S.z - P.z),
                 (Q.x - P.x) * (S.y - P.y) - (Q.y - P.y) * (S.x - P.x) };
        area += 0.5 * std::sqrt(n[e].x * n[e].x + n[e].y * n[e].y + n[e].z * n[e].z);
        const Point3D& opposite = mesh.points[t.v[e]];
        if (n[e].x * (opposite.x - P.x) + n[e].y * (opposite.y - P.y)
            + n[e].z * (opposite.z - P.z) > 0.0) {
            n[e].x = -n[e].x; n[e].y = -n[e].y; n[e].z = -n[e].z;
        }
    }
    if (area > 0.0 && R > 0.0 && std::isfinite(R))
        q.gamma = 9.0 * std::fabs(q.volume) / (area * R);
    double best = 180.0;
    int angleIndex = 0;
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            const double li = std::sqrt(n[i].x * n[i].x + n[i].y * n[i].y + n[i].z * n[i].z);
            const double lj = std::sqrt(n[j].x * n[j].x + n[j].y * n[j].y + n[j].z * n[j].z);
            if (li <= 0.0 || lj <= 0.0) {
                q.degenerate = true;
                best = 0.0;
                ++angleIndex;
                continue;
            }
            const double cosv = (n[i].x * n[j].x + n[i].y * n[j].y + n[i].z * n[j].z) / (li * lj);
            const double ang = std::acos(std::max(-1.0, std::min(1.0, -cosv))) * 180.0 / kPi;
            q.dihedralDeg[angleIndex++] = ang;
            best = std::min(best, ang);
            q.maxDihedralDeg = std::max(q.maxDihedralDeg, ang);
        }
    }
    q.minDihedralDeg = best;
    return q;
}

/// 全网格质量统计。edgeScale 用于退化判定（建议取网格平均边长或 maxEdge）。
MeshQualitySummary summarizeQuality(const TetMesh3D& mesh, double edgeScale,
                                    double sliverAngleDeg)
{
    MeshQualitySummary s;
    s.sliverAngleDeg = sliverAngleDeg;
    s.tetCount = static_cast<int>(mesh.tets.size());
    s.minDihedralDeg = 180.0;
    s.maxDihedralDeg = 0.0;
    s.minVolume = std::numeric_limits<double>::max();
    for (const Tet4& t : mesh.tets) {
        const TetQuality q = tetQuality(mesh, t, edgeScale);
        s.minDihedralDeg = std::min(s.minDihedralDeg, q.minDihedralDeg);
        s.maxDihedralDeg = std::max(s.maxDihedralDeg, q.maxDihedralDeg);
        s.worstRadiusEdge = std::max(s.worstRadiusEdge, q.radiusEdge);
        s.maxEdgeLength = std::max(s.maxEdgeLength, q.maxEdgeLength);
        s.minGamma = std::min(s.minGamma, q.gamma);
        s.avgGamma += q.gamma;
        s.minScaledJacobian = std::min(s.minScaledJacobian, q.scaledJacobian);
        if (q.volume <= 0.0) ++s.invertedCount;
        s.minVolume = std::min(s.minVolume, std::fabs(q.volume));
        if (q.degenerate)
            ++s.degenerateCount;
        if (q.minDihedralDeg < sliverAngleDeg)
            ++s.sliverCount;
    }
    if (mesh.tets.empty()) {
        s.minVolume = 0.0;
        s.minGamma = 0.0;
        s.minScaledJacobian = 0.0;
    } else {
        s.avgGamma /= static_cast<double>(mesh.tets.size());
    }
    return s;
}

} // namespace mesher
