#include <QCoreApplication>

#include <cmath>
#include <iostream>
#include <vector>

#include "app/meshing/TetMeshEngine.h"
#include "mesh/volume/Quality3D.h"

namespace {
int failures = 0;
void check(bool condition, const char* name)
{
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << name << '\n';
    if (!condition) ++failures;
}

TetMeshResult generate(const std::vector<mesher::Point3D>& points)
{
    TetMeshParams params;
    params.targetSize = 100.0;
    params.optimize = false;
    return TetMeshEngine::run(params, points,
        { {0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3} });
}

void testRegularTet()
{
    std::vector<mesher::Point3D> points = {
        {0, 0, 0}, {1, 0, 0}, {0.5, std::sqrt(3.0) / 2.0, 0},
        {0.5, std::sqrt(3.0) / 6.0, std::sqrt(2.0 / 3.0)}
    };
    const auto regular = generate(points);
    check(regular.ok, "regular tetrahedron generation");
    check(regular.qualityAccepted && regular.warnings.empty(), "regular tetrahedron is accepted");
    check(std::fabs(regular.stats.minGamma - 1.0) < 1e-10,
          "regular tetrahedron gamma equals 1");
    const double angle = std::acos(1.0 / 3.0) * 180.0 / std::acos(-1.0);
    check(std::fabs(regular.stats.maxDihedralDeg - angle) < 1e-10,
          "regular tetrahedron max interior dihedral is 70.53 degrees");
    for (auto& p : points) {
        p.x = 7.0 * p.x + 1000.0;
        p.y = 7.0 * p.y - 2000.0;
        p.z = 7.0 * p.z + 3000.0;
    }
    const auto transformed = generate(points);
    check(transformed.ok, "scaled translated tetrahedron generation");
    check(std::fabs(transformed.stats.minGamma - 1.0) < 1e-9,
          "gamma is invariant under translation and uniform scaling");
}

void testAsymmetricTet()
{
    const auto result = generate({ {0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 4} });
    check(result.ok, "asymmetric tetrahedron generation");
    const double minAngle = std::acos(12.0 / std::sqrt(244.0)) * 180.0 / std::acos(-1.0);
    check(std::fabs(result.stats.minDihedralDeg - minAngle) < 1e-9,
          "asymmetric tetrahedron true minimum interior dihedral");
    check(std::fabs(result.stats.maxDihedralDeg - 90.0) < 1e-9,
          "asymmetric tetrahedron true maximum interior dihedral");
    // V=4, surface area=3+4+6+sqrt(244)/2, R=sqrt(29)/2.
    const double gamma = 36.0 / ((13.0 + std::sqrt(244.0) / 2.0) * std::sqrt(29.0) / 2.0);
    check(std::fabs(result.stats.minGamma - gamma) < 1e-9,
          "gamma uses true triangle areas and the circumradius");
    mesher::TetMesh3D mesh;
    mesh.points = { {0, 0, 0}, {2, 0, 0}, {0, 3, 0}, {0, 0, 4} };
    mesh.addTet(0, 1, 2, 3);
    const auto quality = mesher::tetQuality(mesh, mesh.tets.front(), 4.0);
    const double expected[] = {minAngle,
        std::acos(8.0 / std::sqrt(244.0)) * 180.0 / std::acos(-1.0),
        std::acos(6.0 / std::sqrt(244.0)) * 180.0 / std::acos(-1.0), 90.0, 90.0, 90.0};
    for (int i = 0; i < 6; ++i)
        check(std::fabs(quality.dihedralDeg[i] - expected[i]) < 1e-9,
              "all six asymmetric interior dihedrals are measured");
    const auto summary = mesher::summarizeQuality(mesh, 4.0);
    check(std::fabs(summary.avgGamma - gamma) < 1e-9, "module summary reports the same gamma");
}

void testShortFourthEdge()
{
    const auto result = generate({ {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 0.4} });
    const double gamma = 0.6 / ((0.9 + std::sqrt(1.32) / 2.0) * std::sqrt(2.16) / 2.0);
    check(result.ok, "short fourth edge retains a displayable mesh");
    check(std::fabs(result.stats.minGamma - gamma) < 1e-9,
          "circumradius does not assume shortest edge lies in first triangle");
    TetMeshParams params;
    params.targetSize = 0.1;
    params.optimize = false;
    const auto partial = TetMeshEngine::run(params,
        { {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1} },
        { {0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {2, 0, 3} });
    check(partial.ok && !partial.qualityAccepted && !partial.warnings.empty(),
          "size failure is displayable but not quality accepted");
}
} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    testRegularTet();
    testAsymmetricTet();
    testShortFourthEdge();
    return failures == 0 ? 0 : 1;
}
