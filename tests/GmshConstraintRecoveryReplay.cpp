/** @file GmshConstraintRecoveryReplay.cpp
 * @brief 严格相交及初始恢复重放；输入不包含参考连通性。
 * @date 2026-10-07
 * @modified 2026-10-08
 */
#include "mesh/surface/algorithm6/initial/ConstraintRecovery.h"
#include "mesh/surface/algorithm6/initial/InitialTriangulation.h"
#include "mesh/surface/algorithm6/initial/DomainClassification.h"
#include "mesh/surface/algorithm6/initial/InitialUvLegalization.h"
#include "GmshInitialPoolJson.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <map>
#include <stdexcept>
using namespace mesher;
using namespace mesher::gmshReference;
int main(int argc,char** argv)
{
    if(argc != 3) return 2;
    try {
        std::ifstream in(argv[1]); in.imbue(std::locale::classic()); std::size_t count; std::string mode;
        if(!(in >> mode >> count) || count > 1000000) throw std::runtime_error("invalid recovery count");
        std::ofstream out(argv[2]); out.imbue(std::locale::classic()); out << std::setprecision(17);
        if(mode == "intersection") {
            out << "{\"intersections\":[";
            for(std::size_t i = 0; i < count; ++i) {
                Point2D a,b,c,d;
                if(!(in >> a.x >> a.y >> b.x >> b.y >> c.x >> c.y >> d.x >> d.y)) throw std::runtime_error("truncated segment input");
                if(i) out << ','; out << int(intersectRecoverySegments(a,b,c,d));
            }
            out << "]}\n";
        }
        else {
            const bool pipeline = mode == "domain" || mode == "uv";
            const bool chartMode = mode == "chart" || pipeline;
            int faceEntity = -1;
            if(pipeline && !(in >> faceEntity)) throw std::runtime_error("invalid pipeline face entity");
            Point2D lower,upper;
            if(mode == "walk" && !(in >> lower.x >> lower.y >> upper.x >> upper.y)) throw std::runtime_error("invalid recovery rectangle");
            std::vector<InitialChartVertex> points(count);
            for(auto& p : points) if(!(in >> p.physicalId >> p.uv.x >> p.uv.y)) throw std::runtime_error("truncated recovery points");
            std::size_t constraintsCount;
            if(!(in >> constraintsCount) || constraintsCount > 1000000) throw std::runtime_error("invalid constraint count");
            std::vector<InitialConstraint> constraints;
            FaceChart chart; chart.initialVertices = points; std::map<std::uint64_t,int> physicalToLocal;
            for(std::size_t i = 0; i < points.size(); ++i) {
                FaceChartVertex v; v.localId = int(i); v.physicalId = points[i].physicalId; chart.vertices.push_back(v);
                physicalToLocal.emplace(v.physicalId,v.localId);
            }
            for(std::size_t i = 0; i < constraintsCount; ++i) {
                if(chartMode) {
                    std::uint64_t from,to; int curve;
                    if(!(in >> from >> to >> curve)) throw std::runtime_error("truncated physical constraints");
                    chart.constraints.push_back({curve,physicalToLocal.at(from),physicalToLocal.at(to),false});
                }
                else {
                    InitialConstraint c;
                    if(!(in >> c.fromVertex >> c.toVertex >> c.curveEntity)) throw std::runtime_error("truncated constraints");
                    constraints.push_back(c);
                }
            }
            WalkTriangulation mesh; OperationStatus status;
            if(mode == "walk") status = buildWalkTriangulation(lower,upper,points,mesh);
            else if(mode == "initial" || chartMode) {
                OrdinaryInitialTriangulation initial; status = buildInitialTriangulation(chart,initial); mesh = std::move(initial.mesh);
            }
            else throw std::runtime_error("unknown recovery mode");
            if(!status.complete()) throw std::runtime_error(status.message);
            if(chartMode) {
                status = prepareInitialConstraints(chart,mesh,constraints);
                if(!status.complete()) throw std::runtime_error(status.message);
            }
            WalkTriangulation recovered; const auto result = recoverConstraintEdges(mesh,constraints,recovered);
            if(!result.status.complete()) throw std::runtime_error(result.status.message);
            if(pipeline) {
                WalkTriangulation colored; const auto domain = classifyFaceDomain(recovered,faceEntity,colored);
                if(!domain.status.complete()) throw std::runtime_error(domain.status.message);
                InitialUvLegalizationResult uv;
                if(mode == "uv") {
                    WalkTriangulation legalized; uv = legalizeInitialUvMesh(colored,faceEntity,legalized);
                    if(!uv.status.complete()) throw std::runtime_error(uv.status.message);
                    colored = std::move(legalized);
                }
                out << '{'; writeInitialPoolFields(out,colored);
                out << ",\"domain\":{\"seeds\":[" << domain.interiorSeedHalfEdge << ',' << domain.finalOppositeHalfEdge << "],\"visits\":[";
                for(const auto* visits : {&domain.exteriorVisits,&domain.interiorVisits}) {
                    if(visits == &domain.interiorVisits) out << ',';
                    out << '['; for(std::size_t i = 0; i < visits->size(); ++i) { if(i) out << ','; out << (*visits)[i]; } out << ']';
                }
                out << "]},\"uv\":{\"passes\":[";
                for(std::size_t i = 0; i < uv.flippedHalfEdges.size(); ++i) {
                    if(i) out << ','; out << '[';
                    for(std::size_t j = 0; j < uv.flippedHalfEdges[i].size(); ++j) { if(j) out << ','; out << uv.flippedHalfEdges[i][j]; }
                    out << ']';
                }
                out << "],\"limit\":" << (uv.passLimitReached ? "true" : "false") << "}}\n";
            }
            else {
            out << '{'; writeInitialPoolFields(out,recovered); out << ",\"decisions\":[";
            for(std::size_t i = 0; i < result.decisions.size(); ++i) {
                if(i) out << ','; const auto& d = result.decisions[i];
                out << '[' << d.constraint.fromVertex << ',' << d.constraint.toVertex << ',' << d.constraint.curveEntity << ','
                    << d.nativeResult << ',' << d.recoveredHalfEdge << ",[";
                for(std::size_t k = 0; k < d.crossedEdges.size(); ++k) { if(k) out << ','; out << d.crossedEdges[k]; }
                out << "]]";
            }
            out << "]}\n";
            }
        }
        std::string tail; if(in >> tail) throw std::runtime_error("unexpected recovery input tail");
        if(!out) throw std::runtime_error("recovery output failed"); return 0;
    } catch(const OperationFailure& error) { std::cerr << error.status.message << '\n'; return 1; }
      catch(const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
