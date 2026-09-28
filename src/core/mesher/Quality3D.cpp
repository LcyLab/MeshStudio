#include "Quality3D.h"

#include "GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mesher {

namespace {

constexpr double kPi = 3.14159265358979323846;

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
    rhs[0] = 0.5 * ((b.x * b.x + b.y * b.y + b.z * b.z) - (a.x * a.x + a.y * a.y + a.z * a.z));
    M[1][0] = c.x - a.x; M[1][1] = c.y - a.y; M[1][2] = c.z - a.z;
    rhs[1] = 0.5 * ((c.x * c.x + c.y * c.y + c.z * c.z) - (a.x * a.x + a.y * a.y + a.z * a.z));
    M[2][0] = d.x - a.x; M[2][1] = d.y - a.y; M[2][2] = d.z - a.z;
    rhs[2] = 0.5 * ((d.x * d.x + d.y * d.y + d.z * d.z) - (a.x * a.x + a.y * a.y + a.z * a.z));
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
    R = dist3(o, a);
    return true;
}

} // namespace

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
            lmin = std::min(lmin, dist3(mesh.points[t.v[i]], mesh.points[t.v[j]]));
    Point3D o;
    double R = 0.0;
    if (lmin > 0.0 && circumsphere3D(A, B, C, D, o, R))
        q.radiusEdge = R / lmin;

    // 最小二面角：4 个面外法向两两夹角取内角
    Point3D n[4];
    for (int e = 0; e < 4; ++e) {
        const Point3D& P = mesh.points[t.v[(e + 1) % 4]];
        const Point3D& Q = mesh.points[t.v[(e + 2) % 4]];
        const Point3D& S = mesh.points[t.v[(e + 3) % 4]];
        n[e] = { (Q.y - P.y) * (S.z - P.z) - (Q.z - P.z) * (S.y - P.y),
                 (Q.z - P.z) * (S.x - P.x) - (Q.x - P.x) * (S.z - P.z),
                 (Q.x - P.x) * (S.y - P.y) - (Q.y - P.y) * (S.x - P.x) };
    }
    double best = 180.0;
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            const double li = std::sqrt(n[i].x * n[i].x + n[i].y * n[i].y + n[i].z * n[i].z);
            const double lj = std::sqrt(n[j].x * n[j].x + n[j].y * n[j].y + n[j].z * n[j].z);
            if (li <= 0.0 || lj <= 0.0)
                continue;
            const double cosv = (n[i].x * n[j].x + n[i].y * n[j].y + n[i].z * n[j].z) / (li * lj);
            const double ang = std::acos(std::max(-1.0, std::min(1.0, cosv))) * 180.0 / kPi;
            best = std::min(best, std::min(ang, 180.0 - ang));
        }
    }
    q.minDihedralDeg = best;
    return q;
}

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
        s.maxDihedralDeg = std::max(s.maxDihedralDeg, 180.0 - q.minDihedralDeg);
        s.worstRadiusEdge = std::max(s.worstRadiusEdge, q.radiusEdge);
        s.minVolume = std::min(s.minVolume, std::fabs(q.volume));
        if (q.degenerate)
            ++s.degenerateCount;
        if (q.minDihedralDeg < sliverAngleDeg)
            ++s.sliverCount;
    }
    if (mesh.tets.empty())
        s.minVolume = 0.0;
    return s;
}

} // namespace mesher
