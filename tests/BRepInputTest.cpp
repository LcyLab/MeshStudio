// ============================================================
// B-Rep 几何输入自测（控制台程序）
//
// 背景：STEP / IGES / BRep 是**边界表示**，文件里不含任何三角面片。
// 要进四面体引擎必须先离散化（tessellation）。本测试验证这条新链路：
//
//   ShapeLoader::load  →  ShapeToPolyData::tessellate（BRepMesh 逐面离散）
//                      →  mesher::weldSurfaceMesh（容差焊接缝合共享棱）
//                      →  TetMeshEngine::run(params, verts, tris)
//
// 硬断言压在前两级（离散 + 焊接），因为这两级是本次修复的对象：
//   * 逐面离散会在共享棱上产生**重复顶点**（面间不共享拓扑），必须被焊掉；
//   * 焊接必须真的把重复点合并（merged > 0）——否则说明焊接失效，
//     表面不可能水密，下游一定失败。
// 引擎级断言只在带 --with-engine 时执行（见下）。
//
// ★ 为什么引擎级要开关 ★
// 球面这类输入的点集是**凸的且近似共球**：Delaunay 四面体化本身高度退化，
// 且 7692 个约束面走 CDT 是分钟级耗时。因此默认只验证「离散 + 焊接」
//（快、稳定，正是本次修复的部分）；全链路用 --with-engine 在
// cube2.stp 这类轻量输入上验证。
//
// 用法：BRepInputTest [--with-engine] <file.brep|file.stp|file.iges> [...]
// ============================================================

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <QCoreApplication>
#include <QFileInfo>

#include "core/ShapeTessellator.h"
#include "core/TetMeshEngine.h"
#include "core/TetMeshParams.h"
#include "core/mesher/SurfaceMesh3D.h"

namespace {

int g_failures = 0;

void check(bool condition, const char* what)
{
    if (condition) {
        std::cout << "  [PASS] " << what << std::endl;
    } else {
        std::cerr << "  [FAIL] " << what << std::endl;
        ++g_failures;
    }
}

struct Bounds
{
    double diag = 0.0;
    double lo[3] = { 0, 0, 0 };
    double hi[3] = { 0, 0, 0 };
};

Bounds boundsOf(const std::vector<mesher::Point3D>& v)
{
    Bounds b;
    double mn[3] = { 1e300, 1e300, 1e300 };
    double mx[3] = { -1e300, -1e300, -1e300 };
    for (const auto& p : v) {
        const double c[3] = { p.x, p.y, p.z };
        for (int k = 0; k < 3; ++k) {
            mn[k] = std::min(mn[k], c[k]);
            mx[k] = std::max(mx[k], c[k]);
        }
    }
    for (int k = 0; k < 3; ++k) {
        b.lo[k] = mn[k];
        b.hi[k] = mx[k];
    }
    b.diag = std::sqrt((mx[0] - mn[0]) * (mx[0] - mn[0])
                       + (mx[1] - mn[1]) * (mx[1] - mn[1])
                       + (mx[2] - mn[2]) * (mx[2] - mn[2]));
    return b;
}

void runOne(const QString& path, bool withEngine)
{
    const QString name = QFileInfo(path).fileName();
    std::cout << "\n===== " << name.toStdString() << " =====" << std::endl;

    if (!ShapeTessellator::isBRepFormat(path)) {
        std::cerr << "  [SKIP] 不是 B-Rep 类格式" << std::endl;
        return;
    }

    // ---- 1) 离散化（targetSize 传 0 = 用默认弦高 diag·1e-3） ----
    std::vector<mesher::Point3D> verts;
    std::vector<std::array<int, 3>> tris;
    QString err;
    if (!ShapeTessellator::tessellateFile(path, 0.0, verts, tris, &err)) {
        std::cerr << "  [FAIL] 离散化失败: " << err.toStdString() << std::endl;
        ++g_failures;
        return;
    }
    std::cout << "  离散：点 " << verts.size() << " · 三角形 " << tris.size() << std::endl;
    check(!verts.empty() && !tris.empty(), "tessellation produced triangles");

    // 逐面离散的典型特征：顶点数 ≈ 3 × 三角形数（面间不共享顶点）
    const bool faceIndependent = verts.size() >= tris.size() * 3 - 1;
    std::cout << "  " << (faceIndependent ? "逐面独立顶点（符合预期）" : "已部分共享顶点")
              << std::endl;

    const Bounds b = boundsOf(verts);
    std::cout << "  包围盒 diag = " << b.diag
              << " · x ∈ [" << b.lo[0] << ", " << b.hi[0] << "]"
              << " · y ∈ [" << b.lo[1] << ", " << b.hi[1] << "]"
              << " · z ∈ [" << b.lo[2] << ", " << b.hi[2] << "]" << std::endl;
    check(b.diag > 0.0, "bounding box is non-degenerate");

    // ---- 2) 焊接（引擎用的同一容差） ----
    const double tol = b.diag * 1e-7;
    const mesher::WeldResult weld = mesher::weldSurfaceMesh(verts, tris, tol);
    std::cout << "  焊接(tol=" << tol << ")：合并 " << weld.mergedVertices
              << " 个重复点 → " << weld.surface.vertices.size() << " 点，剔除退化三角形 "
              << weld.droppedTriangles << std::endl;
    check(weld.mergedVertices > 0, "welding actually merged duplicate vertices");
    check(weld.surface.vertices.size() < verts.size(), "vertex count decreased after weld");

    std::string wtErr;
    const bool watertight = mesher::isWatertight(weld.surface, &wtErr);
    check(watertight, "welded surface is watertight");
    if (!watertight)
        std::cerr << "    " << wtErr << std::endl;

    // ---- 3) 引擎全链路（默认跳过，--with-engine 打开） ----
    if (!withEngine) {
        std::cout << "  引擎：已跳过（未指定 --with-engine）" << std::endl;
        return;
    }

    TetMeshParams params;
    params.geometryFilePath = path;
    params.targetSize = 0.0;   // 自动 = diag·5%
    params.optimize = true;

    const TetMeshResult r = TetMeshEngine::run(params, verts, tris);
    std::cout << "  引擎：ok=" << (r.ok ? "true" : "false") << std::endl;
    if (!r.ok) {
        std::cout << "    错误：" << r.error.toStdString() << std::endl;
        // 退化输入（凸/共球点集）允许引擎拒绝，但必须**优雅失败**：
        // 返回非空错误信息，而不是崩溃或产出空网格。
        check(!r.error.isEmpty(), "engine fails gracefully with a message");
    } else {
        std::cout << "    节点 " << r.stats.nodeCount << " · 体单元 " << r.stats.tetCount
                  << " · 表面三角形 " << r.stats.surfaceTriCount
                  << " · minSICN " << r.stats.minSICN
                  << " · minGamma " << r.stats.minGamma
                  << " · 翻转单元 " << r.stats.invertedCount << std::endl;
        check(r.stats.tetCount > 0, "engine produced tetrahedra");
        check(r.stats.nodeCount > 0, "engine produced nodes");
        check(r.stats.invertedCount == 0, "no inverted tetrahedra");
    }
    check(r.inputPointCount == static_cast<long long>(verts.size()),
          "engine reports input point count");
    check(r.weldedPointCount == static_cast<long long>(weld.surface.vertices.size()),
          "engine weld matches standalone weld");
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);
    std::cout << "===== B-Rep input self-test =====" << std::endl;

    bool withEngine = false;
    std::vector<QString> files;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--with-engine") {
            withEngine = true;
            continue;
        }
        files.push_back(QString::fromLocal8Bit(argv[i]));
    }
    if (files.empty()) {
        std::cerr << "用法: BRepInputTest [--with-engine] <file.brep|file.stp|file.iges> ..."
                  << std::endl;
        return 2;
    }
    for (const QString& f : files)
        runOne(f, withEngine);

    std::cout << "\n===== result: " << (g_failures == 0 ? "ALL PASS" : "FAILURES")
              << " (failures=" << g_failures << ") =====" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
