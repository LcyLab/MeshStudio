#include "mesh/surface/legacy/SurfaceMesh3D.h"
#include "mesh/validation/VolumeBoundaryValidation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

using namespace mesher;

namespace {
int failures = 0;
void check(bool condition, const char* message)
{
    std::cout << (condition ? "[PASS] " : "[FAIL] ") << message << '\n';
    if (!condition) ++failures;
}

SurfaceMesh3D tetrahedron()
{
    return {{{0,0,0},{1,0,0},{0,1,0},{0,0,1}},
            {{0,2,1},{0,1,3},{0,3,2},{1,2,3}}};
}

SurfaceMesh3D cube(double origin = 0, double side = 1)
{
    SurfaceMesh3D mesh{
        {{0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,0,1},{1,1,1},{0,1,1}},
        {{0,4,7},{0,7,3},{1,2,6},{1,6,5},{0,1,5},{0,5,4},
         {3,7,6},{3,6,2},{0,3,2},{0,2,1},{4,5,6},{4,6,7}}};
    for (auto& p : mesh.vertices) p = {origin+side*p.x,origin+side*p.y,origin+side*p.z};
    return mesh;
}

void reverse(SurfaceMesh3D& mesh)
{
    for (auto& triangle : mesh.triangles) std::swap(triangle[1], triangle[2]);
}

void append(SurfaceMesh3D& mesh, const SurfaceMesh3D& extra)
{
    const int offset = static_cast<int>(mesh.vertices.size());
    mesh.vertices.insert(mesh.vertices.end(), extra.vertices.begin(), extra.vertices.end());
    for (const auto& triangle : extra.triangles)
        mesh.triangles.push_back({triangle[0]+offset,triangle[1]+offset,triangle[2]+offset});
}

void rejected(const SurfaceMesh3D& mesh, const char* description, const char* diagnostic,
              bool intersections = true)
{
    const auto result = validateVolumeBoundary(mesh, intersections);
    check(!result.ok && result.error.find(diagnostic) != std::string::npos, description);
    if (result.ok || result.error.find(diagnostic) == std::string::npos)
        std::cerr << "  diagnostic: " << result.error << '\n';
}

int volumeGateTests()
{
    const auto tet = validateVolumeBoundary(tetrahedron());
    check(tet.ok && tet.shellCount == 1 && std::abs(tet.signedVolume-1.0/6) < 1e-14,
          "closed tetrahedron is accepted");
    auto solid = cube();
    const auto outward = validateVolumeBoundary(solid);
    check(outward.ok && outward.shellCount == 1 && std::abs(outward.signedVolume-1) < 1e-14,
          "closed cube is accepted");
    reverse(solid);
    const auto inward = validateVolumeBoundary(solid);
    check(inward.ok && std::abs(inward.signedVolume+1) < 1e-14,
          "uniformly reversed shell is accepted");
    solid = cube();
    auto cavity = cube(0.25,0.5);
    reverse(cavity);
    append(solid,cavity);
    const auto hollow = validateVolumeBoundary(solid);
    check(hollow.ok && hollow.shellCount == 2 && std::abs(hollow.signedVolume-0.875) < 1e-14,
          "nested closed cavity shells are accepted");
    solid = cube();
    auto second = cube(2);
    reverse(second);
    append(solid,second);
    const auto cancel = validateVolumeBoundary(solid);
    check(cancel.ok && cancel.shellCount == 2 && std::abs(cancel.signedVolume) < 1e-14,
          "opposite disconnected shell volumes may cancel");
    const auto translated = validateVolumeBoundary(cube(1e9));
    check(translated.ok && std::abs(translated.signedVolume-1) < 1e-14,
          "volume calculation survives a large coordinate translation");
    auto thin = tetrahedron();
    thin.vertices[3].z = 1e-12;
    const auto thinResult = validateVolumeBoundary(thin);
    check(thinResult.ok && thinResult.signedVolume > 0,
          "nonzero thin tetrahedron is not rejected by an area or volume cutoff");
    solid = tetrahedron();
    solid.triangles.pop_back();
    rejected(solid,"open shell is rejected","edge");
    solid = tetrahedron();
    std::swap(solid.triangles[0][1],solid.triangles[0][2]);
    rejected(solid,"one reversed face is rejected","triangle");
    SurfaceMesh3D duplicate{{{0,0,0},{1,0,0},{0,1,0}},{{0,1,2},{2,1,0}}};
    rejected(duplicate,"duplicate face reports both triangle indices","duplicate");
    solid = tetrahedron();
    solid.vertices[2] = {2,0,0};
    rejected(solid,"zero-area face is rejected","triangle 0");
    solid = tetrahedron();
    solid.triangles[0][2] = -1;
    rejected(solid,"negative vertex index is rejected safely","triangle 0");
    solid.triangles[0][2] = 4;
    rejected(solid,"out-of-range vertex index is rejected safely","vertex 4");
    solid = tetrahedron();
    solid.vertices[2].z = std::numeric_limits<double>::quiet_NaN();
    rejected(solid,"NaN vertex is rejected","vertex 2");
    solid.vertices[2].z = std::numeric_limits<double>::infinity();
    rejected(solid,"infinite vertex is rejected","vertex 2");
    solid = tetrahedron();
    solid.triangles[0][2] = solid.triangles[0][1];
    rejected(solid,"repeated triangle node is rejected","triangle 0");
    solid = tetrahedron();
    // Two independently closed fans share only node 0: every edge still has two faces.
    solid.vertices.insert(solid.vertices.end(),{{-1,0,0},{0,-1,0},{0,0,-1}});
    solid.triangles.insert(solid.triangles.end(),{{0,4,5},{0,6,4},{0,5,6},{4,6,5}});
    rejected(solid,"pinched vertex with two link cycles is rejected","vertex 0");
    solid = tetrahedron();
    solid.vertices.insert(solid.vertices.end(),{{0,-1,0},{0,0,-1}});
    solid.triangles.insert(solid.triangles.end(),{{0,4,1},{0,1,5},{0,5,4},{1,4,5}});
    rejected(solid,"non-manifold edge shared by four faces is rejected","incident triangles");
    solid = cube();
    append(solid,cube(0.5));
    rejected(solid,"crossing closed shells are rejected","intersect");
    check(validateVolumeBoundary(solid,false).ok,
          "intersection toggle changes only geometric crossing validation");
    solid = cube();
    auto nearby = cube();
    for (auto& p : nearby.vertices) p.x += 1+1e-9;
    append(solid,nearby);
    const auto savedVertices = solid.vertices;
    const auto savedTriangles = solid.triangles;
    const auto distinct = validateVolumeBoundary(solid);
    bool unchanged = solid.triangles == savedTriangles && solid.vertices.size() == savedVertices.size();
    for (size_t i = 0; i < solid.vertices.size(); ++i)
        unchanged = unchanged && solid.vertices[i].x == savedVertices[i].x &&
                    solid.vertices[i].y == savedVertices[i].y && solid.vertices[i].z == savedVertices[i].z;
    check(distinct.ok && distinct.shellCount == 2 && unchanged,
          "nearby distinct nodes remain distinct and the input is unchanged");
    solid = tetrahedron();
    auto splitNode = solid.vertices[0];
    splitNode.x += 1e-16;
    solid.vertices.push_back(splitNode);
    solid.triangles[0][0] = 4;
    rejected(solid,"nearby seam nodes cannot be silently welded into a closed boundary","edge");
    solid = tetrahedron();
    solid.vertices[3] = {0.25,0.25,0};
    rejected(solid,"each flat closed shell must have nonzero volume","volume",false);
    rejected({},"empty boundary is rejected","empty");
    return failures ? 1 : 0;
}

// Diagnostic first: the legacy edge count alone cannot certify a volume boundary.
int legacyRed()
{
    std::string error;
    SurfaceMesh3D duplicate{{{0,0,0},{1,0,0},{0,1,0}}, {{0,1,2},{2,1,0}}};
    check(!isWatertight(duplicate, &error), "volume input must reject duplicate faces");
    auto flat = tetrahedron();
    flat.vertices = {{0,0,0},{1,0,0},{2,0,0},{3,0,0}};
    check(!isWatertight(flat, &error), "volume input must reject zero-area faces");
    auto inconsistent = tetrahedron();
    std::swap(inconsistent.triangles[0][1], inconsistent.triangles[0][2]);
    check(!isWatertight(inconsistent, &error), "volume input must reject inconsistent winding");
    return failures ? 1 : 0;
}
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::string(argv[1]) == "legacy-red") return legacyRed();
    return volumeGateTests();
}
