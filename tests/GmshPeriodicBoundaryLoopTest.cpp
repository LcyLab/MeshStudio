#include "mesh/surface/algorithm6/chart/FaceChart.h"
#include <algorithm>
#include <iostream>
#include <limits>
using namespace mesher;
using namespace mesher::gmshReference;
int main()
{
    int failures = 0;
    auto check = [&](bool ok, const char* name) {
        if(!ok) { ++failures; std::cerr << "FAIL " << name << '\n'; }
    };
    std::vector<PeriodicCurveChart> square = {
        {1,false,{{1,{0.,0.}},{2,{1.,0.}}},{}},
        {2,false,{{2,{1.,0.}},{3,{1.,1.}}},{}},
        {3,false,{{3,{1.,1.}},{4,{0.,1.}}},{}},
        {4,false,{{4,{0.,1.}},{1,{0.,0.}}},{}}
    };
    PeriodicBoundaryLoop loop;
    auto status = assemblePeriodicBoundaryLoop(square,1.e-12,false,loop);
    check(status.complete() && loop.vertices.size() == 4 && loop.initialDirection == 0,
          "ordered loop removes closing duplicate");
    if(loop.vertices.size() == 4) check(loop.vertices[0].physicalId == 1 && loop.vertices[3].physicalId == 4,
          "native loop order and physical identities");
    auto reversed = square;
    std::reverse(reversed[1].vertices.begin(),reversed[1].vertices.end());
    status = assemblePeriodicBoundaryLoop(reversed,1.e-12,false,loop);
    check(status.complete() && loop.vertices.size() == 4 && loop.vertices[1].physicalId == 2,
          "selects reverse next curve by unique nearest UV");
    auto retry = square;
    retry[1] = square[3]; retry[2] = square[2]; retry[3] = square[1];
    status = assemblePeriodicBoundaryLoop(retry,1.e-12,false,loop);
    check(status.complete() && loop.initialDirection == 1 && loop.vertices[0].physicalId == 2,
          "failed continuation retries first curve in reverse");
    auto replaced = square; replaced[1].vertices[0].physicalId = 99;
    status = assemblePeriodicBoundaryLoop(replaced,1.e-12,false,loop);
    check(status.complete() && loop.vertices[1].physicalId == 99,
          "join keeps next curve identity without coordinate or identity averaging");
    auto seam = square;
    seam[1].seam = true;
    seam[1].alternateCoordinates = {{2.,0.},{2.,1.}};
    std::swap(seam[1].vertices[0].uv,seam[1].alternateCoordinates[0]);
    std::swap(seam[1].vertices[1].uv,seam[1].alternateCoordinates[1]);
    status = assemblePeriodicBoundaryLoop(seam,1.e-12,false,loop);
    check(status.complete() && loop.vertices.size() == 4 && loop.vertices[1].uv.x == 1.,
          "selects alternate seam branch");
    std::reverse(seam[1].vertices.begin(),seam[1].vertices.end());
    std::reverse(seam[1].alternateCoordinates.begin(),seam[1].alternateCoordinates.end());
    status = assemblePeriodicBoundaryLoop(seam,1.e-12,false,loop);
    check(status.complete() && loop.vertices.size() == 4 && loop.vertices[1].physicalId == 2,
          "selects reversed alternate seam branch");
    auto firstSeam = square;
    firstSeam[0].seam = true;
    firstSeam[0].alternateCoordinates = {{0.,0.},{1.,0.}};
    firstSeam[0].vertices[0].uv.x = 5.; firstSeam[0].vertices[1].uv.x = 6.;
    status = assemblePeriodicBoundaryLoop(firstSeam,1.e-12,true,loop);
    check(status.complete() && loop.vertices[0].uv.x == 0., "first seam branch is caller controlled");
    PeriodicBoundaryLoop sentinel; sentinel.initialDirection = 77;
    auto tied = square;
    tied[1].vertices.back().uv = tied[1].vertices.front().uv;
    status = assemblePeriodicBoundaryLoop(tied,1.e-12,false,sentinel);
    check(status.code == MeshingStatus::NonConverged && sentinel.initialDirection == 77,
          "equal nearest distances fail explicitly and retain output");
    seam = square; seam[1].seam = true;
    seam[1].alternateCoordinates = {{1.,0.},{1.,1.}};
    status = assemblePeriodicBoundaryLoop(seam,1.e-12,false,sentinel);
    check(status.code == MeshingStatus::NonConverged, "identical seam branches are not arbitrarily selected");
    std::vector<PeriodicCurveChart> single = {{1,false,{{1,{0.,0.}},{2,{1.,0.}},{1,{.125,0.}}},{}}};
    status = assemblePeriodicBoundaryLoop(single,.125,false,sentinel);
    check(status.code == MeshingStatus::NonConverged, "closing tolerance is strict");
    status = assemblePeriodicBoundaryLoop(single,.126,false,loop);
    check(status.complete() && loop.vertices.size() == 2 && loop.vertices[0].uv.x == 0.,
          "closure removes last point without averaging coordinates");
    auto bad = square; bad[0].vertices[0].uv.x = std::numeric_limits<double>::quiet_NaN();
    status = assemblePeriodicBoundaryLoop(bad,1.e-12,false,sentinel);
    check(status.code == MeshingStatus::InvalidInput && sentinel.initialDirection == 77, "nonfinite UV input rejected atomically");
    bad = square; bad[1].seam = true; bad[1].alternateCoordinates = {{0.,0.}};
    status = assemblePeriodicBoundaryLoop(bad,1.e-12,false,sentinel);
    check(status.code == MeshingStatus::InvalidInput, "seam branch length mismatch rejected");
    status = assemblePeriodicBoundaryLoop(square,0.,false,sentinel);
    check(status.code == MeshingStatus::InvalidInput, "nonpositive tolerance rejected");
    RunControl control; control.maximumNodes = 3;
    status = assemblePeriodicBoundaryLoop(square,1.e-12,false,sentinel,control);
    check(status.code == MeshingStatus::BudgetExceeded && sentinel.initialDirection == 77, "loop node budget retains output");
    control = {}; control.keepGoing = [] { return false; };
    status = assemblePeriodicBoundaryLoop(square,1.e-12,false,sentinel,control);
    check(status.code == MeshingStatus::Cancelled && sentinel.initialDirection == 77, "loop cancellation retains output");
    status = assemblePeriodicBoundaryLoop({},1.e-12,false,loop);
    check(status.complete() && loop.vertices.empty(), "empty edge loop matches native success");
    std::cout << "periodic boundary loop failures=" << failures << '\n';
    return failures ? 1 : 0;
}
