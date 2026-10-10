/** @file CadSurfaceBenchmark.cpp
 * @brief 独立性能工具：固定参数、可关闭观察、唯一目录及可复核结果指纹。
 * @date 2026-10-02
 */
#include "CadBenchmarkOptions.h"
#include "CadSurfaceQualityReport.h"
#include "cad/occ/OccCadGeometry.h"
#include "cad/import/ShapeLoader.h"
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include <Psapi.h>
#endif
#include <Standard_Failure.hxx>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDirIterator>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QSysInfo>
#include <QUuid>
#include <QtEndian>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <map>
#include <tuple>

namespace {
using Clock=std::chrono::steady_clock;
using namespace mesher;
double seconds(Clock::time_point a,Clock::time_point b) { return std::chrono::duration<double>(b-a).count(); }
const char* stages[]={"generation","boundary_sampling","initial_triangulation","constraint_recovery",
    "domain_classification","refinement","geometry_evaluation","topology_update","legalization","mesh_assembly","validation",
    "criterion_measurement","intersection_validation","physical_quality"};
const char* counters[]={"geometry_queries","whole_mesh_copies","copied_points","copied_triangles",
    "triangle_visits","point_visits","cavity_triangles","whole_domain_classifications","geometry_cache_hits","geometry_cache_samples",
    "local_mesh_transactions","transaction_triangles_logged","transaction_halfedges_logged","rolled_back_transactions",
    "whole_mesh_validations","local_mesh_validations","cavity_stamp_resets",
    "refinement_queue_pushes","refinement_queue_stale_entries","refinement_queue_peak_entries",
    "refinement_queue_compactions","full_criterion_audits",
    "boundary_queue_requests","boundary_queue_stale_requests","boundary_queue_peak_entries",
    "quality_candidate_trials","quality_rejected_domain","quality_rejected_duplicate","quality_rejected_floor",
    "quality_rejected_shape","quality_rejected_geometry","quality_rejected_topology","quality_rejected_metric",
    "quality_candidate_comparisons","quality_reused_failures","shape_candidate_trials","shape_reused_neighborhoods","shape_rejected_floor","quality_reused_faces","narrow_boundary_alignment_requests"};
static_assert(sizeof(counters)/sizeof(counters[0])==size_t(MeshCounter::NarrowBoundaryAlignmentRequests)+1,
              "benchmark counter names must cover the observation enum");
using Key=std::tuple<int,int,int>; // stage/counter, entity dimension, entity ID
struct Times { double inclusive=0,exclusive=0; std::uint64_t calls=0; };
class Observation final : public MeshingObserver {
    struct Frame { Key key; Clock::time_point start; double children=0; };
    std::vector<Frame> stack;
public:
    std::map<Key,Times> times;
    std::map<Key,std::uint64_t> counts;
    std::map<std::string,std::uint64_t> queries;
    std::map<std::uint64_t,std::uint64_t> cavitySizes;
    bool valid=true;
    void stageBegin(MeshStage s,MeshEntityRef e) noexcept override { stack.push_back({{int(s),e.dimension,e.id},Clock::now(),0}); }
    void stageEnd(MeshStage s,MeshEntityRef e) noexcept override {
        const auto end=Clock::now();
        if (stack.empty() || stack.back().key!=Key{int(s),e.dimension,e.id}) { valid=false; return; }
        const auto frame=stack.back(); stack.pop_back();
        const double elapsed=seconds(frame.start,end);
        auto& t=times[frame.key]; ++t.calls; t.inclusive+=elapsed; t.exclusive+=elapsed-frame.children;
        if (!stack.empty()) stack.back().children+=elapsed;
    }
    void addCount(MeshCounter c,MeshEntityRef e,std::uint64_t n) noexcept override {
        counts[{int(c),e.dimension,e.id}]+=n;
        if (c==MeshCounter::CavityTriangles) ++cavitySizes[n];
    }
    bool balanced() const { return valid && stack.empty(); }
    double partition() const { double sum=0; for (const auto& p:times) sum+=p.second.exclusive; return sum; }
};
// 全部能力原样转发，关闭观察时直接用 OccCadGeometry，不引入代理/逐查询时钟。
class ObservedGeometry final : public GeometrySource {
    const GeometrySource& g; Observation& o;
    template<class F> auto query(const char* kind,MeshEntityRef e,F fn) const -> decltype(fn()) {
        MeshStageScope scope({&o,e},MeshStage::GeometryEvaluation);
        ++o.queries[kind]; o.addCount(MeshCounter::GeometryQueries,e,1); return fn();
    }
public:
    ObservedGeometry(const GeometrySource& source,Observation& observer):g(source),o(observer) {}
    int vertexCount() const override { return query("vertexCount",{},[&]{return g.vertexCount();}); }
    Point3D vertexPoint(int v) const override { return query("vertexPoint",{0,v},[&]{return g.vertexPoint(v);}); }
    int edgeCount() const override { return query("edgeCount",{},[&]{return g.edgeCount();}); }
    std::pair<int,int> edgeVertices(int e) const override { return query("edgeVertices",{1,e},[&]{return g.edgeVertices(e);}); }
    std::pair<double,double> edgeRange(int e) const override { return query("edgeRange",{1,e},[&]{return g.edgeRange(e);}); }
    Point3D edgePoint(int e,double t) const override { return query("edgePoint",{1,e},[&]{return g.edgePoint(e,t);}); }
    Point3D edgeDerivative(int e,double t) const override { return query("edgeDerivative",{1,e},[&]{return g.edgeDerivative(e,t);}); }
    bool edgeDegenerate(int e) const override { return query("edgeDegenerate",{1,e},[&]{return g.edgeDegenerate(e);}); }
    int faceCount() const override { return query("faceCount",{},[&]{return g.faceCount();}); }
    FaceTopology faceTopology(int f) const override { return query("faceTopology",{2,f},[&]{return g.faceTopology(f);}); }
    Point2D edgeUV(int f,const EdgeUse& u,double t) const override { return query("edgeUV",{2,f},[&]{return g.edgeUV(f,u,t);}); }
    Point3D facePoint(int f,Point2D uv) const override { return query("facePoint",{2,f},[&]{return g.facePoint(f,uv);}); }
    Point3D faceNormal(int f,Point2D uv) const override { return query("faceNormal",{2,f},[&]{return g.faceNormal(f,uv);}); }
    void faceDerivatives(int f,Point2D uv,Point3D& du,Point3D& dv) const override {
        query("faceDerivatives",{2,f},[&]{g.faceDerivatives(f,uv,du,dv);});
    }
    bool faceParameterBounds(int f,SurfaceParameterBounds& b) const override { return query("faceParameterBounds",{2,f},[&]{return g.faceParameterBounds(f,b);}); }
    bool faceReversed(int f) const override { return query("faceReversed",{2,f},[&]{return g.faceReversed(f);}); }
    bool normalUsesOrientedDerivatives() const override { return g.normalUsesOrientedDerivatives(); }
    std::string validationError() const override { return query("validationError",{},[&]{return g.validationError();}); }
    bool bounds(Point3D& lo,Point3D& hi) const override { return query("bounds",{},[&]{return g.bounds(lo,hi);}); }
    bool expectsClosedSurface() const override { return query("expectsClosedSurface",{},[&]{return g.expectsClosedSurface();}); }
    int faceBodyId(int f) const override {return g.faceBodyId(f);}
    CadBodyContact bodyContact(int a,int b) const override {return query("bodyContact",{2,a},[&]{return g.bodyContact(a,b);});}
    bool commonSurfaceChart(int a,int b) const override {return query("commonSurfaceChart",{2,a},[&]{return g.commonSurfaceChart(a,b);});}
    bool faceCoordinates(int f,Point3D p,Point2D seed,Point2D& uv) const override {return query("faceCoordinates",{2,f},[&]{return g.faceCoordinates(f,p,seed,uv);});}
    Point2D facePeriods(int f) const override {return query("facePeriods",{2,f},[&]{return g.facePeriods(f);});}
    CadTrimLocation faceTrimLocation(int f,Point2D uv) const override {return query("faceTrimLocation",{2,f},[&]{return g.faceTrimLocation(f,uv);});}
    bool closestTrimCoordinates(int f,Point3D p,Point2D seed,Point2D& uv,double& distance) const override {return query("closestTrimCoordinates",{2,f},[&]{return g.closestTrimCoordinates(f,p,seed,uv,distance);});}
};
void save(const QString& path,const QByteArray& data) {
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly|QIODevice::NewOnly) || file.write(data)!=data.size() || !file.flush())
        throw std::runtime_error("cannot create output: "+path.toStdString());
}
QString fileHash(const QString& path) {
    QFile file(path); QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!file.open(QIODevice::ReadOnly) || !hash.addData(&file)) throw std::runtime_error("cannot hash: "+path.toStdString());
    return QString::fromLatin1(hash.result().toHex());
}
QString fingerprint(const SurfaceMeshResult& r) {
    QCryptographicHash hash(QCryptographicHash::Sha256);
    const auto integer=[&](std::uint64_t n) { const auto le=qToLittleEndian<quint64>(n); hash.addData(reinterpret_cast<const char*>(&le),8); };
    const auto real=[&](double d) { if (!std::isfinite(d)) throw std::runtime_error("non-finite result coordinate");
        std::uint64_t bits; std::memcpy(&bits,&d,8); integer(bits); };
    integer(r.mesh.vertices.size()); for (auto p:r.mesh.vertices) { real(p.x); real(p.y); real(p.z); }
    integer(r.mesh.triangles.size()); for (auto t:r.mesh.triangles) for (int v:t) integer(v);
    integer(r.triangleFaceIds.size()); for (int f:r.triangleFaceIds) integer(f);
    integer(r.triangleBodyIds.size()); for(int b:r.triangleBodyIds)integer(b);
    integer(int(r.assemblyPolicy));integer(r.bodyInteractions.size());for(auto i:r.bodyInteractions){integer(i.firstBody);integer(i.secondBody);integer(i.firstFace);integer(i.secondFace);integer(int(i.contact));integer(i.firstPartition);integer(i.secondPartition);}
    integer(r.triangleCornerUV.size());
    for(size_t t=0;t<r.triangleCornerUV.size();++t)for(auto p:r.triangleCornerUV[t]) {
        if(t<r.triangleFaceIds.size()&&r.triangleFaceIds[t]==-1&&!r.trianglePartitionIds.empty()) {
            if(!std::isnan(p.x)||!std::isnan(p.y))throw std::runtime_error("cross-face legacy UV must be explicitly absent");
            integer(0);integer(0);
        }else {real(p.x);real(p.y);}
    }
    integer(r.trianglePartitionIds.size());for(int p:r.trianglePartitionIds)integer(p);
    integer(r.partitionSourceFaces.size());for(const auto& faces:r.partitionSourceFaces){integer(faces.size());for(int f:faces)integer(f);}
    integer(r.trianglePartitionUV.size());for(auto t:r.trianglePartitionUV)for(auto p:t){real(p.x);real(p.y);}
    integer(r.triangleCadSupportSamples.size());for(auto t:r.triangleCadSupportSamples)for(auto p:t){integer(p.face);real(p.uv.x);real(p.uv.y);real(p.distance);integer(p.projectedToTrim);}
    integer(r.boundaryEdges.size()); for (auto e:r.boundaryEdges) { integer(e.a); integer(e.b); integer(e.cadEdge); }
    return QString::fromLatin1(hash.result().toHex());
}
QJsonObject sourceMetadata() {
    const QString root=QString::fromUtf8(CAD_BENCH_SOURCE_ROOT);
    QProcess git; git.setWorkingDirectory(root);
    git.start("git",{"rev-parse","HEAD"});
    if (!git.waitForFinished(10000) || git.exitCode()) throw std::runtime_error("cannot identify checkout HEAD");
    const QString head=QString::fromUtf8(git.readAllStandardOutput()).trimmed();
    git.start("git",{"status","--porcelain=v1","--untracked-files=all"});
    if (!git.waitForFinished(10000) || git.exitCode()) throw std::runtime_error("cannot identify dirty checkout");
    const QByteArray status=git.readAllStandardOutput();
    QStringList files; files<<root+"/CMakeLists.txt";
    for (const auto& folder:{"src","tests"}) {
        QDirIterator it(root+"/"+folder,QDir::Files,QDirIterator::Subdirectories);
        while (it.hasNext()) files<<it.next();
    }
    files.sort(); QCryptographicHash hash(QCryptographicHash::Sha256); QJsonArray inputs;
    for (const auto& path:files) {
        const QString relative=QDir(root).relativeFilePath(path),digest=fileHash(path);
        hash.addData(relative.toUtf8()); hash.addData("\0",1); hash.addData(digest.toLatin1());
        inputs.append(QJsonObject{{"path",relative},{"sha256",digest}});
    }
    return {{"root",root},{"head",head},{"dirtyStatus",QString::fromUtf8(status)},
        {"sourceFingerprint",QString::fromLatin1(hash.result().toHex())},{"sourceInputs",inputs},
        {"buildConfiguration",QString::fromUtf8(CAD_BENCH_CONFIG)},{"compiler",QString::fromUtf8(CAD_BENCH_COMPILER)}};
}
QJsonObject criteriaJson(const SurfaceCriteria& c) {
    return {{"target",c.targetSize},{"min",c.minSize},{"max",c.maxSize},{"deviation",c.maxDeviation},{"angle",c.maxNormalAngleDeg},
        {"radiusEdgeRatio",c.maxRadiusEdgeRatio},{"growth",c.growthRatio},{"work",c.maxIterations},{"vertices",c.maxVertices},
        {"shapeSweeps",c.shapeOptimizationSweeps},{"qualityReuse",c.reuseFailedQualityPasses},
        {"placement",surfacePointPlacementName(c.pointPlacement)},
        {"crossSmoothCadSplits",c.crossSmoothCadSplits}};
}
QJsonObject shapeJson(const cadquality::Statistics& s){
    return {{"bad",double(s.bad)},{"invalid",double(s.invalid)},{"triangles",double(s.total)},
        {"minAngle",s.minAngle},{"worst",s.worst},{"p95",s.p95},{"p99",s.p99},
        {"below25",double(s.below25)},{"below30",double(s.below30)},
        {"nearEquilateral",double(s.nearEquilateral)},{"nearEquilateralFraction",s.total?double(s.nearEquilateral)/s.total:0.0},
        {"p1Angle",s.p1Angle},{"p5Angle",s.p5Angle},{"p50Angle",s.p50Angle},
        {"p1Shape",s.p1Shape},{"p5Shape",s.p5Shape},{"p50Shape",s.p50Shape}};
}
QJsonObject workJson(const SurfaceWorkStatistics& w) {
    return {{"total",w.totalWork},{"boundarySamplingSplits",w.boundarySamplingSplits},{"sharedEdgeSplits",w.sharedEdgeSplits},
        {"intersectionRefinements",w.intersectionRefinements},{"insertions",w.insertedSurfacePoints},{"flips",w.delaunayFlips},
        {"frontalCandidates",w.frontalCandidates},{"frontalInserted",w.frontalInserted},{"frontalRejected",w.frontalRejected},
        {"refinementEdgeViolations",w.refinementEdgeViolations},{"refinementDeviationViolations",w.refinementDeviationViolations},
        {"refinementNormalViolations",w.refinementNormalViolations},{"refinementQualityViolations",w.refinementQualityViolations},
        {"faceBuilds",w.faceBuilds},{"passes",w.schedulingPasses},{"allocatedVertices",w.allocatedVertices},
        {"violationsAfterLegalization",w.violationsAfterLegalization}};
}
}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    QString output;
    bool ownsOutput=false;
    QJsonObject manifest;
    Observation observer;
    const auto started=Clock::now();
    try {
        auto options=cadbench::parse(cadbench::arguments(argc,argv));
        const auto source=sourceMetadata();
        const QString cad=QFileInfo(QString::fromLocal8Bit(options.cad.c_str())).absoluteFilePath();
        const QString inputHash=fileHash(cad);
        output=options.output.empty()?QDir::tempPath()+"/MeshStudio-surface-perf/"+
            QDateTime::currentDateTimeUtc().toString("yyyyMMdd-HHmmss-zzz")+"-"+QUuid::createUuid().toString(QUuid::WithoutBraces):
            QFileInfo(QString::fromLocal8Bit(options.output.c_str())).absoluteFilePath();
        if (QFileInfo::exists(output) || !QDir().mkpath(output)) throw std::runtime_error("output directory must be new: "+output.toStdString());
        ownsOutput=true;
        QProcess cpu; cpu.start("powershell",{"-NoProfile","-Command","(Get-CimInstance Win32_Processor).Name"});
        const bool cpuOk=cpu.waitForFinished(10000) && cpu.exitCode()==0;
        manifest={{"schema",1},{"cad",cad},{"inputSha256",inputHash},{"source",source},{"observe",options.observe},
            {"executableSha256",fileHash(QCoreApplication::applicationFilePath())},
            {"timeoutSeconds",options.timeout},{"utc",QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs)},
            {"cpu",cpuOk?QString::fromLocal8Bit(cpu.readAllStandardOutput()).trimmed():QString("unavailable")},
            {"architecture",QSysInfo::currentCpuArchitecture()},{"mesherThreads",1},{"warmup",false},
            {"powerAndOtherLoad","not controlled; inspect repeated runs"}};
        std::cout<<"output="<<output.toStdString()<<std::endl;
        const auto loadStarted=Clock::now(); TopoDS_Shape shape; QString error;
        if (!ShapeLoader::load(cad,shape,&error)) throw std::runtime_error(error.toStdString());
        const auto imported=Clock::now(); OccCadGeometry geo(shape,true); const auto adapted=Clock::now();
        if (!geo.validationError().empty()) throw std::runtime_error(geo.validationError());
        cadbench::resolve(options,geo);
        manifest["criteria"]=criteriaJson(options.criteria);
        const auto prepared=geo.preparationStatistics();
        manifest["geometryPreparation"]=QJsonObject{{"surfaceAdaptors",double(prepared.surfaceAdaptors)},
            {"curveAdaptors",double(prepared.curveAdaptors)},{"pcurves",double(prepared.pcurves)}};
        manifest["cadTopology"]=QJsonObject{{"faces",geo.faceCount()},{"edges",geo.edgeCount()},{"vertices",geo.vertexCount()},{"closed",geo.expectsClosedSurface()}};
        ObservedGeometry traced(geo,observer);
        const GeometrySource& input=options.observe?static_cast<const GeometrySource&>(traced):static_cast<const GeometrySource&>(geo);
        const auto meshStarted=Clock::now(); auto reported=meshStarted; double firstFace=-1; int lastFace=0;
        const auto result=generateCadSurfaceMesh(input,options.criteria,nullptr,[&](int done,int total) {
            const auto now=Clock::now(); lastFace=done;
            // Partition preparation/support callbacks also carry work counts;
            // they do not identify completion of an original CAD face.
            if (!options.criteria.crossSmoothCadSplits && done>0 && firstFace<0) firstFace=seconds(meshStarted,now);
            if (seconds(reported,now)>10) { std::cout<<"face="<<done<<'/'<<total<<" seconds="<<seconds(meshStarted,now)<<std::endl; reported=now; }
            return seconds(meshStarted,now)<options.timeout;
        },options.observe?&observer:nullptr,AssemblySurfacePolicy::IndependentBodies);
        const auto meshed=Clock::now();
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS memory{};
        if (GetProcessMemoryInfo(GetCurrentProcess(),&memory,sizeof(memory)))
            manifest["memory"]=QJsonObject{{"peakWorkingSetBytes",double(memory.PeakWorkingSetSize)},
                {"workingSetBytesAfterGeneration",double(memory.WorkingSetSize)},
                {"scope","whole benchmark process, including CAD import and evaluators"}};
#endif
        if (!observer.balanced()) throw std::runtime_error("unbalanced observation events");
        manifest["ok"]=result.ok; manifest["error"]=QString::fromStdString(result.error);
        manifest["assemblyPolicy"]="independent_body_surfaces";
        QJsonArray interactions;for(const auto& i:result.bodyInteractions)interactions.append(QJsonObject{{"firstBody",i.firstBody},{"secondBody",i.secondBody},{"firstFace",i.firstFace},{"secondFace",i.secondFace},{"contact",int(i.contact)}});
        manifest["bodyInteractions"]=interactions;
        manifest["resultFingerprint"]=fingerprint(result); manifest["work"]=workJson(result.work);
        if(result.ok) {
            const auto quality=cadquality::analyze(result,options.criteria.maxRadiusEdgeRatio,options.criteria.minSize);
            manifest["quality"]=shapeJson(quality);
            QJsonArray perFace;
            for(const auto& item:quality.byFace){auto entry=shapeJson(item.second);entry["face"]=item.first;perFace.append(entry);}
            manifest["qualityByFace"]=perFace;
            QJsonArray perBody;
            for(const auto& item:quality.byBody){auto entry=shapeJson(item.second);entry["body"]=item.first;perBody.append(entry);}
            manifest["qualityByBody"]=perBody;
            std::ostringstream csv;cadquality::writeCsv(csv,quality,result);save(output+"/quality.csv",QByteArray::fromStdString(csv.str()));
            if(options.snapshotFace>=0){
                if(options.snapshotFace>=geo.faceCount())throw std::invalid_argument("snapshot face out of range");
                for(const auto& sources:result.partitionSourceFaces)if(sources.size()>1&&
                    std::find(sources.begin(),sources.end(),options.snapshotFace)!=sources.end())
                    throw std::invalid_argument("original-face snapshot is unsupported for a multi-face patch; use patch UV and support metadata");
                std::ostringstream snapshot;snapshot<<std::setprecision(17)<<"triangle";
                for(int k=0;k<3;++k)snapshot<<",id"<<k<<",x"<<k<<",y"<<k<<",z"<<k<<",u"<<k<<",v"<<k;snapshot<<'\n';
                for(size_t t=0;t<result.mesh.triangles.size();++t)if(result.triangleFaceIds[t]==options.snapshotFace){snapshot<<t;
                    for(int k=0;k<3;++k){int id=result.mesh.triangles[t][k];auto p=result.mesh.vertices[id];auto uv=result.triangleCornerUV[t][k];
                        snapshot<<','<<id<<','<<p.x<<','<<p.y<<','<<p.z<<','<<uv.x<<','<<uv.y;}snapshot<<'\n';}
                save(output+"/face-"+QString::number(options.snapshotFace)+".csv",QByteArray::fromStdString(snapshot.str()));
            }
            if(quality.invalid)throw std::runtime_error("independent quality detected a degenerate output facet");
        }
        manifest["outputMesh"]=QJsonObject{{"points",double(result.mesh.vertices.size())},{"triangles",double(result.mesh.triangles.size())},
            {"maxEdge",result.measuredMaxEdgeLength},{"sampledDeviation",result.measuredMaxDeviation},
            {"normalAngle",result.measuredMaxNormalAngleDeg},{"radiusEdgeRatio",result.measuredWorstRadiusEdgeRatio}};
        QJsonArray patches;for(const auto& sources:result.partitionSourceFaces){QJsonArray ids;for(int f:sources)ids.append(f);patches.append(ids);}
        manifest["partitionSourceFaces"]=patches;
        manifest["cadSupportContract"]="v2: facet corners, edge midpoints, centroid: actual trimmed CAD support, physical distance and closest-trim flag; point support only";
        double maximumSupportDistance=0;size_t projectedSupportSamples=0;
        for(const auto& triangle:result.triangleCadSupportSamples)for(const auto& support:triangle) {
            maximumSupportDistance=std::max(maximumSupportDistance,support.distance);projectedSupportSamples+=support.projectedToTrim;
        }
        manifest["cadSupport"]=QJsonObject{{"maximumDistance",maximumSupportDistance},{"projectedToTrimSamples",double(projectedSupportSamples)}};
        manifest["timing"]=QJsonObject{{"import",seconds(loadStarted,imported)},{"adapter",seconds(imported,adapted)},
            {"generationIncludingVerification",seconds(meshStarted,meshed)},{"importAdapterGeneration",seconds(loadStarted,meshed)},
            {"firstCompletedFace",firstFace},{"lastFace",lastFace},{"exclusivePartition",observer.partition()},
            {"setup",seconds(started,loadStarted)}};
        QJsonObject totals,queryKinds,cavities;
        for (const auto& p:observer.counts) { const auto name=QString(counters[std::get<0>(p.first)]); totals[name]=totals[name].toDouble()+double(p.second); }
        for (const auto& p:observer.queries) queryKinds[QString::fromStdString(p.first)]=double(p.second);
        for (const auto& p:observer.cavitySizes) cavities[QString::number(qulonglong(p.first))]=double(p.second);
        manifest["counters"]=totals; manifest["geometryQueryKinds"]=queryKinds;
        manifest["cavitySizeHistogram"]=cavities;
        QByteArray stageLines,faceLines="face,stage,calls,inclusive_seconds,exclusive_seconds\n";
        for (const auto& p:observer.times) {
            const auto [stage,dimension,id]=p.first; const auto t=p.second;
            stageLines+=QJsonDocument(QJsonObject{{"stage",stages[stage]},{"dimension",dimension},{"entity",id},
                {"calls",double(t.calls)},{"inclusiveSeconds",t.inclusive},{"exclusiveSeconds",t.exclusive}}).toJson(QJsonDocument::Compact)+'\n';
            if (dimension==2) faceLines+=QByteArray::number(id)+','+stages[stage]+','+QByteArray::number(qulonglong(t.calls))+','+
                QByteArray::number(t.inclusive,'g',17)+','+QByteArray::number(t.exclusive,'g',17)+'\n';
        }
        for (const auto& p:observer.counts) {
            const auto [counter,dimension,id]=p.first;
            stageLines+=QJsonDocument(QJsonObject{{"counter",counters[counter]},{"dimension",dimension},{"entity",id},
                {"count",double(p.second)}}).toJson(QJsonDocument::Compact)+'\n';
        }
        QByteArray diagnostics; for (const auto& d:result.diagnostics) diagnostics+=QByteArray::fromStdString(d)+'\n';
        save(output+"/stages.jsonl",stageLines); save(output+"/faces.csv",faceLines); save(output+"/diagnostics.txt",diagnostics);
        if (fileHash(cad)!=inputHash || sourceMetadata()["sourceFingerprint"]!=source["sourceFingerprint"])
            throw std::runtime_error("input or source changed during benchmark");
        auto timing=manifest["timing"].toObject(); timing["postprocessing"]=seconds(meshed,Clock::now()); manifest["timing"]=timing;
        save(output+"/run.json",QJsonDocument(manifest).toJson());
        std::cout<<std::setprecision(17)<<"ok="<<result.ok<<" generation_seconds="<<seconds(meshStarted,meshed)
            <<" points="<<result.mesh.vertices.size()<<" triangles="<<result.mesh.triangles.size()
            <<" fingerprint="<<manifest["resultFingerprint"].toString().toStdString()<<'\n';
        if (!result.ok) std::cerr<<result.error<<'\n';
        return result.ok?0:1;
    } catch (const Standard_Failure& e) {
        manifest["ok"]=false; manifest["error"]=QString::fromLocal8Bit(e.GetMessageString());
    } catch (const std::exception& e) {
        manifest["ok"]=false; manifest["error"]=QString::fromLocal8Bit(e.what());
    }
    std::cerr<<manifest["error"].toString().toStdString()<<'\n';
    // Import/exception failures retain a failed manifest when a new directory exists.
    if (ownsOutput && !QFileInfo::exists(output+"/run.json")) {
        try {
            manifest["observationBalanced"]=observer.balanced();
            QByteArray rows;
            for (const auto& p:observer.times) {
                const auto [stage,dimension,id]=p.first;
                rows+=QJsonDocument(QJsonObject{{"stage",stages[stage]},{"dimension",dimension},{"entity",id},
                    {"calls",double(p.second.calls)},{"inclusiveSeconds",p.second.inclusive},
                    {"exclusiveSeconds",p.second.exclusive}}).toJson(QJsonDocument::Compact)+'\n';
            }
            if (!QFileInfo::exists(output+"/stages.jsonl")) save(output+"/stages.jsonl",rows);
            save(output+"/run.json",QJsonDocument(manifest).toJson());
        }
        catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; }
    }
    return 1;
}
