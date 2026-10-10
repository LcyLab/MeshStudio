#pragma once
// Test-only Gmsh exchange. Face IDs are mapped by unique area/centroid signatures
// and independently checked against OCC trimmed faces; numeric IDs are not reused.
#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "mesh/validation/SurfaceMeshValidation.h"
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <vector>
#include <iostream>

inline mesher::SurfaceMeshResult readExternalSurface(const std::string& path,
        const TopTools_IndexedMapOfShape& faces,double diagonal) {
    std::ifstream input(path); std::string magic;
    int nf=0,np=0,nt=0; input>>magic>>nf>>np>>nt;
    if (!input || magic!="MESHCAD1" || nf!=faces.Extent() || np<3 || nt<1 ||
        np>10000000 || nt>20000000) throw std::runtime_error("invalid external mesh header");
    std::vector<GProp_GProps> properties(nf);
    for (int f=0;f<nf;++f) BRepGProp::SurfaceProperties(TopoDS::Face(faces(f+1)),properties[f]);
    std::map<int,int> mapping; std::set<int> used;
    for (int i=0;i<nf;++i) {
        int tag; double area,x,y,z; input>>tag>>area>>x>>y>>z;
        if (!input || !std::isfinite(area)||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))
            throw std::runtime_error("invalid external face signature");
        std::vector<int> candidates;
        for (int f=0;f<nf;++f)
            if (std::abs(properties[f].Mass()-area)<=1e-7*std::max(1.0,std::abs(area)) &&
                properties[f].CentreOfMass().Distance(gp_Pnt(x,y,z))<=diagonal*1e-7)
                candidates.push_back(f);
        if (candidates.size()!=1 || used.count(candidates.front()) || mapping.count(tag))
            throw std::runtime_error("ambiguous/non-bijective external CAD face mapping at tag "+std::to_string(tag));
        mapping.emplace(tag,candidates.front()); used.insert(candidates.front());
    }
    mesher::SurfaceMeshResult result;
    result.mesh.vertices.resize(np); result.mesh.triangles.resize(nt);
    result.triangleFaceIds.resize(nt); result.triangleCornerUV.resize(nt);
    std::vector<std::set<int>> faceVertices(nf);
    std::map<std::pair<int,int>,std::pair<int,int>> edges;
    for (auto& p:result.mesh.vertices) {
        input>>p.x>>p.y>>p.z;
        if (!input || !std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))
            throw std::runtime_error("invalid external point");
    }
    for (int t=0;t<nt;++t) {
        int tag; auto& tri=result.mesh.triangles[t]; input>>tag>>tri[0]>>tri[1]>>tri[2];
        if (!input || !mapping.count(tag)) throw std::runtime_error("invalid external triangle label");
        result.triangleFaceIds[t]=mapping.at(tag);
        for (int k=0;k<3;++k) {
            int a=tri[k],b=tri[(k+1)%3];
            if (a<0||a>=np||b<0||b>=np||a==b) throw std::runtime_error("invalid external triangle index");
            faceVertices[mapping.at(tag)].insert(a);
            auto& e=edges[std::minmax(a,b)]; ++e.first; e.second+=a<b?1:-1;
        }
        const auto a=result.mesh.vertices[tri[0]],b=result.mesh.vertices[tri[1]],c=result.mesh.vertices[tri[2]];
        const double ux=b.x-a.x,uy=b.y-a.y,uz=b.z-a.z,vx=c.x-a.x,vy=c.y-a.y,vz=c.z-a.z;
        const double area=std::hypot(uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx);
        if (!(area>0)||!std::isfinite(area)) throw std::runtime_error("degenerate external triangle");
    }
    std::string extra; if (input>>extra) throw std::runtime_error("trailing external mesh data");
    for (const auto& e:edges) if (e.second.first!=2 || e.second.second!=0)
        throw std::runtime_error("external surface is not closed/oriented at edge "+std::to_string(e.first.first));
    double maxResidual=0; int samples=0;
    for (int f=0;f<nf;++f) {
        if (faceVertices[f].empty()) throw std::runtime_error("external face has no triangles");
        const std::vector<int> nodes(faceVertices[f].begin(),faceVertices[f].end());
        const size_t count=std::min<size_t>(32,nodes.size());
        BRepExtrema_DistShapeShape distance; distance.LoadS2(faces(f+1));
        for (size_t k=0;k<count;++k) {
            const auto p=result.mesh.vertices[nodes[k*nodes.size()/count]];
            distance.LoadS1(BRepBuilderAPI_MakeVertex(gp_Pnt(p.x,p.y,p.z)).Vertex());
            if (!distance.Perform()||!distance.IsDone()||!std::isfinite(distance.Value()))
                throw std::runtime_error("external face mapping OCC distance failed");
            maxResidual=std::max(maxResidual,distance.Value()); ++samples;
            if (distance.Value()>diagonal*5e-7)
                throw std::runtime_error("external node does not match mapped trimmed CAD face "+std::to_string(f));
        }
    }
    std::string error;
    if (!mesher::validateSurfaceIntersections(result.mesh,&error))
        throw std::runtime_error("external mesh intersection check: "+error);
    std::cout<<"externalMapping faces="<<nf<<" sampledVertices="<<samples<<" maxResidual="<<maxResidual
             <<" closedOriented=1 intersections=0\n";
    result.ok=true; return result;
}
