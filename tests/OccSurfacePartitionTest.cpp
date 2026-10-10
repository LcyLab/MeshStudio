#include "cad/occ/OccCadGeometry.h"
#include "OccSplitSurfaceFixtures.h"
#include "mesh/surface/legacy/SurfaceMeshingPartition.h"
#include "mesh/validation/SurfaceMeshValidation.h"
#include "CadSurfaceQualityReport.h"
#include <TopExp.hxx>
#include <BRepTools.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <iostream>
#include <cmath>
#include <sstream>
#include <filesystem>
#include <fstream>

int main(int argc,char** argv) {
    try {
        for(bool cylinder:{false,true}) {
            const auto shape=cylinder?occsplit::cylinder(true):occsplit::plane(true);
            TopTools_IndexedMapOfShape solids;TopExp::MapShapes(shape,TopAbs_SOLID,solids);
            OccCadGeometry source(shape);
            if(!source.validationError().empty() || solids.Extent()!=1)throw std::runtime_error("fixture must be one valid solid");
            bool found=false;
            for(int a=0;a<source.faceCount();++a)for(int b=a+1;b<source.faceCount();++b) {
                if(!source.commonSurfaceChart(a,b))continue;
                found=true;mesher::SurfaceParameterBounds bounds;source.faceParameterBounds(b,bounds);
                const mesher::Point2D native{(bounds.uMin+bounds.uMax)*.5,(bounds.vMin+bounds.vMax)*.5};
                const auto p=source.facePoint(b,native);mesher::Point2D projected;
                if(!source.faceCoordinates(a,p,native,projected))throw std::runtime_error("common surface projection failed");
                const auto q=source.facePoint(a,projected);
                if(std::hypot(p.x-q.x,p.y-q.y,p.z-q.z)>1e-9)throw std::runtime_error("common surface residual");
                if(source.faceTrimLocation(b,native)!=mesher::CadTrimLocation::Inside)throw std::runtime_error("source trim classification");
                if(source.faceBodyId(a)!=source.faceBodyId(b))throw std::runtime_error("split changed body");
            }
            if(!found)throw std::runtime_error(cylinder?"split cylinder common chart not recognized":"split plane common chart not recognized");
            std::cout<<"PASS same-solid "<<(cylinder?"cylinder":"plane")<<" common chart faces="<<source.faceCount()<<'\n';
            std::ostringstream before;BRepTools::Write(source.acceptedShape(),before);
            mesher::SurfacePartitionPolicy policy;policy.enabled=true;
            mesher::SurfaceMeshingPartition partition(source,policy);
            if(partition.faceCount()!=source.faceCount()-1||partition.removedCadEdges().size()!=1) {
                for(const auto& diagnostic:partition.diagnostics())std::cerr<<diagnostic<<'\n';
                throw std::runtime_error("actual OCC split did not become one virtual patch");
            }
            const int splitEdge=partition.removedCadEdges().front();
            auto protectedPolicy=policy;protectedPolicy.protectedCadEdges={splitEdge};
            mesher::SurfaceMeshingPartition protectedPartition(source,protectedPolicy);
            if(protectedPartition.faceCount()!=source.faceCount()||!protectedPartition.removedCadEdges().empty())
                throw std::runtime_error("explicit semantic edge was removed");
            mesher::SurfaceCriteria criteria;criteria.targetSize=cylinder?.5:1;
            criteria.maxSize=criteria.targetSize*1.05;criteria.minSize=criteria.targetSize*.1;
            criteria.maxDeviation=cylinder?.005:.01;criteria.maxNormalAngleDeg=20;
            criteria.shapeOptimizationSweeps=0;criteria.pointPlacement=mesher::SurfacePointPlacement::FrontalDelaunay;
            const auto result=mesher::generatePartitionedCadSurfaceMesh(source,criteria,policy);
            if(!result.ok)throw std::runtime_error(result.error);
            std::string water;if(!mesher::isWatertight(result.mesh,&water))throw std::runtime_error(water);
            bool crosses=false;int patchTriangles=0;
            for(size_t t=0;t<result.mesh.triangles.size();++t) {
                const int patch=result.trianglePartitionIds.at(t);
                if(result.partitionSourceFaces.at(patch).size()<2)continue;
                ++patchTriangles;double lo=1e9,hi=-1e9;
                for(int v:result.mesh.triangles[t]) {
                    const auto p=result.mesh.vertices[v];const double coordinate=cylinder?p.z:p.x;
                    lo=std::min(lo,coordinate);hi=std::max(hi,coordinate);
                }
                const double splitCoordinate=cylinder?4:8;crosses|=lo<splitCoordinate-1e-8&&hi>splitCoordinate+1e-8;
                if(result.triangleFaceIds[t]!=-1||!std::isnan(result.triangleCornerUV[t][0].x))throw std::runtime_error("cross-face patch mislabeled as original CAD face");
                const auto& uv=result.trianglePartitionUV[t];
                const mesher::Point2D samples[7]={uv[0],uv[1],uv[2],{(uv[0].x+uv[1].x)/2,(uv[0].y+uv[1].y)/2},
                    {(uv[1].x+uv[2].x)/2,(uv[1].y+uv[2].y)/2},{(uv[2].x+uv[0].x)/2,(uv[2].y+uv[0].y)/2},
                    {(uv[0].x+uv[1].x+uv[2].x)/3,(uv[0].y+uv[1].y+uv[2].y)/3}};
                for(int k=0;k<7;++k) {
                    const auto support=result.triangleCadSupportSamples[t][k];
                    const auto p=source.facePoint(support.face,support.uv),q=partition.facePoint(patch,samples[k]);
                    const auto location=source.faceTrimLocation(support.face,support.uv);
                    if(std::hypot(p.x-q.x,p.y-q.y,p.z-q.z)>1e-9||
                       (location!=mesher::CadTrimLocation::Inside&&location!=mesher::CadTrimLocation::Boundary))
                        throw std::runtime_error("support sample not on recorded original trim");
                }
            }
            if(!crosses)throw std::runtime_error("triangles still constrained by removable CAD split");
            for(auto edge:result.boundaryEdges)if(edge.cadEdge==splitEdge)throw std::runtime_error("removed split persisted as mesh constraint");
            const auto unsplitShape=cylinder?occsplit::cylinder(false):occsplit::plane(false);
            OccCadGeometry unsplitSource(unsplitShape);
            const auto unsplitResult=mesher::generatePartitionedCadSurfaceMesh(unsplitSource,criteria,policy);
            if(!unsplitResult.ok)throw std::runtime_error(unsplitResult.error);
            TopTools_IndexedMapOfShape unsplitFaces;TopExp::MapShapes(unsplitShape,TopAbs_FACE,unsplitFaces);
            int targetFace=-1;
            for(int f=0;f<unsplitSource.faceCount();++f) {
                const BRepAdaptor_Surface surface(TopoDS::Face(unsplitFaces(f+1)));
                if((cylinder&&surface.GetType()==GeomAbs_Cylinder)||(!cylinder&&surface.GetType()==GeomAbs_Plane&&std::abs(surface.Plane().Location().Z()-4)<1e-9))targetFace=f;
            }
            if(targetFace<0)throw std::runtime_error("unsplit comparison face missing");
            const auto patchQuality=[&](const mesher::SurfaceMeshResult& mesh,bool split) {
                mesher::SurfaceMeshResult subset;subset.mesh.vertices=mesh.mesh.vertices;
                for(size_t t=0;t<mesh.mesh.triangles.size();++t) {
                    const bool select=split?mesh.partitionSourceFaces.at(mesh.trianglePartitionIds[t]).size()>1:mesh.triangleFaceIds[t]==targetFace;
                    if(select){subset.mesh.triangles.push_back(mesh.mesh.triangles[t]);subset.triangleFaceIds.push_back(0);}
                }
                if(subset.mesh.triangles.empty())throw std::runtime_error("empty segmentation comparison");
                return cadquality::analyze(subset,criteria.maxRadiusEdgeRatio,criteria.minSize);
            };
            const auto splitQuality=patchQuality(result,true),unsplitQuality=patchQuality(unsplitResult,false);
            const double splitFraction=double(splitQuality.nearEquilateral)/splitQuality.total,unsplitFraction=double(unsplitQuality.nearEquilateral)/unsplitQuality.total;
            std::cout<<"segmentation comparison "<<(cylinder?"cylinder":"plane")<<" triangles="<<splitQuality.total<<'/'<<unsplitQuality.total
                <<" nearEquilateral="<<splitFraction<<'/'<<unsplitFraction<<" angleP5="<<splitQuality.p5Angle<<'/'<<unsplitQuality.p5Angle<<'\n';
            // Compare shape distributions, not node identity: retained edge sampling
            // and the initial boundary order can differ after CAD segmentation.
            if(splitQuality.invalid||unsplitQuality.invalid||std::abs(splitFraction-unsplitFraction)>.1||
               std::abs(splitQuality.p5Angle-unsplitQuality.p5Angle)>5||splitQuality.total<.75*unsplitQuality.total||splitQuality.total>1.25*unsplitQuality.total)
                throw std::runtime_error("smooth CAD segmentation changed shape distribution substantially");
            std::ostringstream after;BRepTools::Write(source.acceptedShape(),after);
            if(before.str()!=after.str())throw std::runtime_error("partition generation mutated original CAD");
            std::cout<<"PASS cross-split "<<(cylinder?"cylinder":"plane")<<" patch triangles="<<patchTriangles<<" total="<<result.mesh.triangles.size()<<'\n';
            if(argc==2) {
                const std::filesystem::path directory(argv[1]);
                const auto cad=directory/(cylinder?"split-cylinder.brep":"split-plane.brep");
                if(!BRepTools::Write(shape,cad.string().c_str()))throw std::runtime_error("fixture export failed");
                std::ofstream mesh(directory/(cylinder?"split-cylinder.obj":"split-plane.obj"));mesh.precision(17);
                for(auto p:result.mesh.vertices)mesh<<"v "<<p.x<<' '<<p.y<<' '<<p.z<<'\n';
                for(auto t:result.mesh.triangles)mesh<<"f "<<t[0]+1<<' '<<t[1]+1<<' '<<t[2]+1<<'\n';
                if(!mesh)throw std::runtime_error("mesh fixture export failed");
            }
        }
    } catch(const Standard_Failure& e) {std::cerr<<e.GetMessageString()<<'\n';return 1;}
      catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
