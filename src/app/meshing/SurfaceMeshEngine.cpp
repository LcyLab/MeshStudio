// 文件职责：应用层表面生成入口：读取 CAD、选择生成路线，再将网格与 CAD 标签转换为 VTK 数据。
#include "SurfaceMeshEngine.h"

#include "cad/occ/OccCadGeometry.h"
#include "cad/occ/OccAlgorithm6Adapter.h"
#include "cad/import/ShapeLoader.h"

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkIntArray.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkFieldData.h>
#include <vtkStringArray.h>

#include <TopoDS_Shape.hxx>
#include <Standard_Failure.hxx>
#include <BRepTools.hxx>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <unordered_map>
#include <sstream>
#include <memory>
#include <cmath>

/// 从 CAD 文件生成表面网格，并保留 CAD 面、边、部件和逐角 UV。
SurfaceGenerateResult SurfaceMeshEngine::run(const QString& filePath,
                                              const mesher::SurfaceCriteria& criteria,
                                              mesher::MeshingProgress progress,
                                              const mesher::SurfacePartitionPolicy& partitionPolicy) {
    SurfaceGenerateResult result;
    try {
    // 先读取几何并保活适配器；后续算法回调借用这些对象，不能提前释放。
    TopoDS_Shape shape;
    if (!ShapeLoader::load(filePath,shape,&result.error)) return result;
    const bool algorithm6=criteria.pointPlacement==mesher::SurfacePointPlacement::Algorithm6Experimental;
    std::unique_ptr<OccAlgorithm6Adapter> adapter;
    std::unique_ptr<OccCadGeometry> legacyGeometry;
    if(algorithm6) adapter=std::make_unique<OccAlgorithm6Adapter>(shape);
    else legacyGeometry=std::make_unique<OccCadGeometry>(shape,true);
    const auto& geometry=adapter?adapter->geometry():*legacyGeometry;
    if(algorithm6) {
        // 实验路线只接受原 CAD 面和明确预算；失败返回原因，不自动改走其他算法。
        const auto& settings=criteria.algorithm6;
        if(criteria.crossSmoothCadSplits || partitionPolicy.enabled || !std::isfinite(settings.targetSize) || settings.targetSize<=0 ||
           (settings.extendBoundarySize!=0 && settings.extendBoundarySize!=1) || settings.smoothingIterations<0 ||
           settings.maximumNodes<=0 || settings.maximumGeometryEvaluations<=0) {
            result.error=QStringLiteral("Algorithm 6 requires explicit positive h and budgets, boundary extension 0/1, and original CAD faces");return result;
        }
        mesher::gmshReference::SurfaceMeshingOptions options;
        options.sizeMax=settings.targetSize;options.sizeFromPoints=false;options.elementsPerCurvatureCircle=0;
        options.extendBoundarySize=settings.extendBoundarySize;options.smoothingIterations=settings.smoothingIterations;options.switchOnFailure=false;
        mesher::gmshReference::RunControl control;
        control.maximumNodes=static_cast<std::size_t>(settings.maximumNodes);
        control.maximumGeometryEvaluations=static_cast<std::size_t>(settings.maximumGeometryEvaluations);
        // 此回调是取消心跳；不开阶段网格观察，工作总量未知。
        control.keepGoing=[progress]{return !progress || progress(0,0);};
        auto generated=mesher::gmshReference::generateSurfaceMesh(adapter->buildModelInput(),options,control);
        result.algorithm6Faces=std::move(generated.faces);
        if(!generated.status.complete()) {
            result.detail.error="Algorithm 6 face "+std::to_string(generated.failedFace)+" at "+generated.failedStage+": "+generated.status.message;
            result.error=QString::fromStdString(result.detail.error);return result;
        }
        result.detail=std::move(generated.surface);
        result.detail.assemblyPolicy=mesher::AssemblySurfacePolicy::IndependentBodies;
        result.detail.diagnostics.push_back("Algorithm 6 generation completed; shape, CAD distances and body contacts require independent audit.");
    } else if(criteria.crossSmoothCadSplits || partitionPolicy.enabled) {
        auto policy=partitionPolicy;policy.enabled=true;
        result.detail=mesher::generatePartitionedCadSurfaceMesh(geometry,criteria,policy,nullptr,progress,nullptr,mesher::AssemblySurfacePolicy::IndependentBodies);
    } else result.detail=mesher::generateCadSurfaceMesh(geometry,criteria,nullptr,progress,nullptr,mesher::AssemblySurfacePolicy::IndependentBodies);
    result.detail.diagnostics.insert(result.detail.diagnostics.begin(),geometry.inputDiagnostics().begin(),geometry.inputDiagnostics().end());
    if (!result.detail.ok) {
        result.error=QString::fromStdString(result.detail.error);
        return result;
    }
    // 算法结果通过后才创建 VTK 数据；节点索引保持一致，避免展示转换重新焊接身份。
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    points->Allocate(static_cast<vtkIdType>(result.detail.mesh.vertices.size()));
    for (const auto& p:result.detail.mesh.vertices)
        points->InsertNextPoint(p.x,p.y,p.z);
    vtkNew<vtkCellArray> polygons;
    vtkNew<vtkIntArray> faceIds;
    faceIds->SetName("CadFaceId");
    vtkNew<vtkIntArray> bodyIds;bodyIds->SetName("CadBodyId");
    vtkNew<vtkDoubleArray> cornerUV;
    cornerUV->SetName("CadCornerUV");
    cornerUV->SetNumberOfComponents(6);
    vtkNew<vtkIntArray> edgeIds;
    edgeIds->SetName("CadEdgeId");
    edgeIds->SetNumberOfComponents(3);
    auto key=[](int a,int b) {
        if (a>b) std::swap(a,b);
        return (uint64_t(uint32_t(a))<<32)|uint32_t(b);
    };
    std::unordered_map<uint64_t,int> cadEdges;
    for (const auto& edge:result.detail.boundaryEdges)
        cadEdges[key(edge.a,edge.b)]=edge.cadEdge;
    // UV 按三角形的三个角保存：共享物理节点在周期缝或不同 CAD 面上可以有不同参数。
    for (size_t i=0;i<result.detail.mesh.triangles.size();++i) {
        const auto& t=result.detail.mesh.triangles[i];
        const auto& uv=result.detail.triangleCornerUV[i];
        const vtkIdType ids[3]={t[0],t[1],t[2]};
        polygons->InsertNextCell(3,ids);
        faceIds->InsertNextValue(result.detail.triangleFaceIds[i]);
        bodyIds->InsertNextValue(result.detail.triangleBodyIds[i]);
        const double corners[6]={uv[0].x,uv[0].y,uv[1].x,uv[1].y,uv[2].x,uv[2].y};
        cornerUV->InsertNextTuple(corners);
        int edgeLabels[3]={-1,-1,-1};
        for (int k=0;k<3;++k) {
            const auto found=cadEdges.find(key(t[k],t[(k+1)%3]));
            if (found!=cadEdges.end()) edgeLabels[k]=found->second;
        }
        edgeIds->InsertNextTuple3(edgeLabels[0],edgeLabels[1],edgeLabels[2]);
    }
    result.surface=vtkSmartPointer<vtkPolyData>::New();
    result.surface->SetPoints(points);
    result.surface->SetPolys(polygons);
    result.surface->GetCellData()->AddArray(faceIds);
    result.surface->GetCellData()->AddArray(bodyIds);
    result.surface->GetCellData()->AddArray(cornerUV);
    result.surface->GetCellData()->AddArray(edgeIds);
    if(algorithm6) {
        // 把有效配置和逐面状态随 VTP 保存；生成完成标记不替代独立质量审计。
        vtkNew<vtkStringArray> algorithm;algorithm->SetName("SurfaceGenerationAlgorithm");
        algorithm->InsertNextValue("algorithm6-experimental; first_order; serial; switch_on_failure=0; quality_not_audited");
        result.surface->GetFieldData()->AddArray(algorithm);
        vtkNew<vtkDoubleArray> configuration;configuration->SetName("Algorithm6RunConfiguration");configuration->SetNumberOfComponents(5);
        const double values[5]={criteria.algorithm6.targetSize,double(criteria.algorithm6.extendBoundarySize),double(criteria.algorithm6.smoothingIterations),
            double(criteria.algorithm6.maximumNodes),double(criteria.algorithm6.maximumGeometryEvaluations)};
        configuration->InsertNextTuple(values);
        result.surface->GetFieldData()->AddArray(configuration);
        vtkNew<vtkStringArray> faces;faces->SetName("Algorithm6FaceStatus");
        for(const auto& face:result.algorithm6Faces)faces->InsertNextValue("face="+std::to_string(face.face)+", body="+std::to_string(face.bodyId)+
            ", attempted="+std::to_string(face.attempted)+", current="+std::to_string(face.current)+", status="+std::to_string(int(face.status.code))+", stage="+face.failedStage);
        result.surface->GetFieldData()->AddArray(faces);
    }
    if(!result.detail.trianglePartitionIds.empty()) {
        // 分区结果额外保存原 CAD 支撑面及采样偏差，区分生成图表与实际 CAD 来源。
        vtkNew<vtkIntArray> patchIds;patchIds->SetName("SurfacePatchId");
        vtkNew<vtkDoubleArray> patchUV;patchUV->SetName("SurfacePatchCornerUV");patchUV->SetNumberOfComponents(6);
        vtkNew<vtkIntArray> supportFaces;supportFaces->SetName("CadSupportFaceIds");supportFaces->SetNumberOfComponents(7);
        vtkNew<vtkDoubleArray> supportUV;supportUV->SetName("CadSupportUV");supportUV->SetNumberOfComponents(14);
        vtkNew<vtkDoubleArray> supportDistance;supportDistance->SetName("CadSupportDistance");supportDistance->SetNumberOfComponents(7);
        vtkNew<vtkIntArray> supportProjected;supportProjected->SetName("CadSupportProjectedToTrim");supportProjected->SetNumberOfComponents(7);
        for(size_t t=0;t<result.detail.mesh.triangles.size();++t) {
            patchIds->InsertNextValue(result.detail.trianglePartitionIds[t]);
            const auto& uv=result.detail.trianglePartitionUV[t];
            const double corners[6]={uv[0].x,uv[0].y,uv[1].x,uv[1].y,uv[2].x,uv[2].y};patchUV->InsertNextTuple(corners);
            int faces[7],projected[7];double coordinates[14],distances[7];
            for(int k=0;k<7;++k){const auto sample=result.detail.triangleCadSupportSamples[t][k];
                faces[k]=sample.face;coordinates[2*k]=sample.uv.x;coordinates[2*k+1]=sample.uv.y;
                distances[k]=sample.distance;projected[k]=sample.projectedToTrim?1:0;}
            supportFaces->InsertNextTypedTuple(faces);supportUV->InsertNextTuple(coordinates);
            supportDistance->InsertNextTuple(distances);supportProjected->InsertNextTypedTuple(projected);
        }
        result.surface->GetCellData()->AddArray(patchIds);result.surface->GetCellData()->AddArray(patchUV);
        result.surface->GetCellData()->AddArray(supportFaces);result.surface->GetCellData()->AddArray(supportUV);
        result.surface->GetCellData()->AddArray(supportDistance);result.surface->GetCellData()->AddArray(supportProjected);
        vtkNew<vtkStringArray> patchSources;patchSources->SetName("SurfacePatchSourceFaces");
        for(const auto& sources:result.detail.partitionSourceFaces) {
            std::ostringstream list;for(size_t i=0;i<sources.size();++i){if(i)list<<',';list<<sources[i];}
            patchSources->InsertNextValue(list.str());
        }
        result.surface->GetFieldData()->AddArray(patchSources);
        vtkNew<vtkStringArray> supportContract;supportContract->SetName("CadSupportSamplingContract");
        supportContract->InsertNextValue("v2: facet corners, edge midpoints, centroid; actual trimmed CAD support UV, physical distance and closest-trim flag; point support, not exact triangle footprint coverage");
        result.surface->GetFieldData()->AddArray(supportContract);
    }
    // 将实测残差和候选拒绝诊断写入 VTP，显示和工程保存可继续读取同一份证据。
    vtkNew<vtkStringArray> diagnostics;diagnostics->SetName("CadSurfaceDiagnostics");
    for(const auto& text:result.detail.diagnostics)diagnostics->InsertNextValue(text);
    result.surface->GetFieldData()->AddArray(diagnostics);
    vtkNew<vtkStringArray> semantics;semantics->SetName("CadAssemblyPolicy");semantics->InsertNextValue("independent_body_surfaces");
    result.surface->GetFieldData()->AddArray(semantics);
    vtkNew<vtkIntArray> interactions;interactions->SetName("CadBodyInteractions");interactions->SetNumberOfComponents(5);
    vtkNew<vtkIntArray> patchInteractions;patchInteractions->SetName("SurfacePatchBodyInteractions");patchInteractions->SetNumberOfComponents(7);
    for(const auto& i:result.detail.bodyInteractions) {
        const int tuple[5]={i.firstBody,i.secondBody,i.firstFace,i.secondFace,int(i.contact)};interactions->InsertNextTypedTuple(tuple);
        const int partitionTuple[7]={i.firstBody,i.secondBody,i.firstFace,i.secondFace,int(i.contact),i.firstPartition,i.secondPartition};
        patchInteractions->InsertNextTypedTuple(partitionTuple);
    }
    result.surface->GetFieldData()->AddArray(interactions);
    if(!result.detail.trianglePartitionIds.empty())result.surface->GetFieldData()->AddArray(patchInteractions);
    if(geometry.inputWasReconciled()) {
        // 工程包继续保存用户原始 CAD 文件；VTP 同时记录本次接受的仅容差修正 BREP 及修复说明，适配器已在生成前核对原标签顺序。
        std::ostringstream brep;BRepTools::Write(geometry.acceptedShape(),brep);
        vtkNew<vtkStringArray> accepted;accepted->SetName("CadAcceptedBRep");accepted->InsertNextValue(brep.str());
        vtkNew<vtkStringArray> changes;changes->SetName("CadInputReconciliation");
        for(const auto& d:geometry.inputDiagnostics())changes->InsertNextValue(d);
        result.surface->GetFieldData()->AddArray(accepted);result.surface->GetFieldData()->AddArray(changes);
    }
    result.ok=true;
    } catch (const Standard_Failure& failure) {
        result.error=QStringLiteral("CAD evaluation failed: ")+QString::fromUtf8(failure.GetMessageString());
    } catch (const std::exception& failure) {
        result.error=QStringLiteral("Surface meshing failed: ")+QString::fromUtf8(failure.what());
    }
    return result;
}
