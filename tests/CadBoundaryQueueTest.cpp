#include "mesh/surface/legacy/CadBoundaryRegistry.h"
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <limits>
using namespace mesher;
void require(bool ok,const char* why) { if (!ok) throw std::runtime_error(why); }
int main() { try {
    CadBoundaryRegistry registry(1); auto& edge=registry.edge(0);
    edge.samples={{0,0},{1,1},{2,2}}; edge.nodeIds={{0,0},{1,1},{2,2}}; edge.ready=true;
    std::string error; uint64_t first=0,duplicate=0,second=0; double parameter=0;
    require(registry.enqueueSplit(0,0,1,4,"first",first,&error),"queue first interval");
    require(registry.enqueueSplit(0,0,1,7,"duplicate",duplicate,&error) && first==duplicate,"shared request deduplicated across faces");
    require(registry.enqueueSplit(0,1,2,4,"second",second,&error),"queue second interval");
    CadBoundaryRegistry::SplitRequest request;
    require(registry.popSplitRequest(request) && request.id==first && request.sourceFace==4,"stable request metadata");
    require(registry.applySplit(request,parameter,&error) && parameter==0.5 && edge.nodeIds.size()==4 && !edge.ready,"one shared parameter registered");
    require(!registry.popSplitRequest(request),"old boundary version ignored");
    edge.samples={{0,0},{0.5,3},{1,1},{2,2}}; edge.nodeIds[0.5]=3; edge.ready=true;
    require(registry.enqueueSplit(0,1,2,7,"new",second,&error),"new version request");
    require(registry.popSplitRequest(request),"take new request"); registry.invalidate(0);
    const auto nodes=edge.nodeIds;
    require(!registry.applySplit(request,parameter,&error) && nodes==edge.nodeIds,"taken old request cannot mutate new boundary");
    edge.ready=true;
    require(!registry.enqueueSplit(0,0,1,4,"not adjacent",first,&error),"non-adjacent interval rejected");
    const double adjacent=std::nextafter(2.0,3.0); edge.samples={{2,2},{adjacent,4}};
    edge.nodeIds[adjacent]=4;
    require(registry.enqueueSplit(0,2,adjacent,4,"precision",first,&error) && registry.popSplitRequest(request),"precision request staged");
    const auto before=edge.nodeIds;
    require(!registry.applySplit(request,parameter,&error) && before==edge.nodeIds,"precision failure leaves nodes unchanged");
    CadBoundaryRegistry batchRegistry(1);auto& batchEdge=batchRegistry.edge(0);
    batchEdge.samples={{0,0},{1,1},{2,2}};batchEdge.nodeIds={{0,0},{1,1},{2,2}};batchEdge.ready=true;
    require(batchRegistry.enqueueSplit(0,0,1,4,"batch first",first,&error),"batch first request");
    require(batchRegistry.enqueueSplit(0,1,2,4,"batch second",second,&error),"batch second request");
    std::vector<CadBoundaryRegistry::SplitRequest> requests;
    while(batchRegistry.popSplitRequest(request))requests.push_back(request);
    const auto version=batchEdge.version;std::vector<double> parameters;
    require(batchRegistry.applySplitBatch(requests,parameters,&error) && parameters==std::vector<double>{.5,1.5}
        && batchEdge.nodeIds.size()==5 && batchEdge.version==version+1,"all same-version intervals applied before one invalidation");
    const auto batchNodes=batchEdge.nodeIds;
    require(!batchRegistry.applySplitBatch(requests,parameters,&error) && batchNodes==batchEdge.nodeIds,
        "stale batch leaves boundary unchanged");
    CadBoundaryRegistry shifted(1);auto& shiftedEdge=shifted.edge(0);
    shiftedEdge.samples={{0,0},{1,1},{2,2}};shiftedEdge.nodeIds={{0,0},{1,1},{2,2}};shiftedEdge.ready=true;
    require(shifted.enqueueSplit(0,0,1,4,"quality offset",first,&error,.45),"queue offset parameter");
    require(shifted.enqueueSplit(0,0,1,7,"other face offset",duplicate,&error,.55)&&duplicate==first&&shifted.popSplitRequest(request),
        "shared offset request retains the first accepted parameter");
    require(shifted.applySplit(request,parameter,&error)&&parameter==.45&&shiftedEdge.nodeIds.count(.45)&&!shiftedEdge.nodeIds.count(.5),
        "shared quality split must use the accepted CAD parameter");
    shiftedEdge.samples={{0,0},{.45,3},{1,1},{2,2}};shiftedEdge.nodeIds[.45]=3;shiftedEdge.ready=true;
    require(shifted.enqueueSplit(0,1,2,4,"quality",first,&error,{},CadBoundarySplitPurpose::Quality),"quality purpose queued");
    require(shifted.enqueueSplit(0,1,2,7,"geometry",duplicate,&error)&&first==duplicate&&shifted.popSplitRequest(request)&&
        request.purpose==CadBoundarySplitPurpose::Geometry,"shared geometry request overrides quality purpose");
    require(shifted.enqueueSplit(0,1,2,4,"quality only",first,&error,{},CadBoundarySplitPurpose::Quality)&&
        shifted.popSplitRequest(request)&&request.purpose==CadBoundarySplitPurpose::Quality,"quality purpose retained");
    require(!shifted.enqueueSplit(0,1,2,4,"invalid purpose",first,&error,{},static_cast<CadBoundarySplitPurpose>(99)),"invalid purpose rejected");
    for(double invalid:{1.,2.,-1.,std::numeric_limits<double>::quiet_NaN()})
        require(!shifted.enqueueSplit(0,1,2,4,"invalid offset",first,&error,invalid),"invalid explicit split parameter rejected");
    require(shifted.enqueueSplit(0,0,.45,4,"batch offset first",first,&error,.2),"queue batch offset first");
    require(shifted.enqueueSplit(0,1,2,4,"batch offset second",second,&error,1.55),"queue batch offset second");
    requests.clear();while(shifted.popSplitRequest(request))requests.push_back(request);
    auto malformed=requests;malformed[1].parameter=malformed[1].right;const auto savedNodes=shiftedEdge.nodeIds;
    require(!shifted.applySplitBatch(malformed,parameters,&error)&&savedNodes==shiftedEdge.nodeIds,"invalid offset batch is atomic");
    malformed=requests;malformed[1].purpose=static_cast<CadBoundarySplitPurpose>(99);
    require(!shifted.applySplitBatch(malformed,parameters,&error)&&savedNodes==shiftedEdge.nodeIds,"invalid purpose batch is atomic");
    require(shifted.applySplitBatch(requests,parameters,&error)&&parameters==std::vector<double>{.2,1.55},"offset batch preserves both parameters");
    std::cout<<"PASS shared request deduplication, boundary versions and precision failure\n";
    return 0;
} catch (const std::exception& e) { std::cerr<<"FAIL "<<e.what()<<'\n'; return 1; } }
