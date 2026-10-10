cmake_minimum_required(VERSION 3.16)

project(MeshStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 日常方案只生成产品；验证工程在独立目录显式开启。
option(BUILD_TESTING "Build MeshStudio tests, replay tools and benchmarks" OFF)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMake")

# 本机依赖路径（可用 -DCMAKE_PREFIX_PATH=... 覆盖）
list(APPEND CMAKE_PREFIX_PATH
    "F:/Qt/Qt5.14.2/5.14.2/msvc2017_64"
    "F:/ProgramData/opencascade-7.6.0/products/VTK-9.3.1/release"
)

if(MSVC)
    add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:/utf-8;/W3;/EHsc;/fp:precise>")
endif()

find_package(Qt5 COMPONENTS Widgets Svg Concurrent REQUIRED)
find_package(VTK COMPONENTS
    CommonCore
    CommonDataModel
    CommonMath
    FiltersCore
    FiltersSources
    GUISupportQt
    IOGeometry
    IOXML
    InteractionStyle
    RenderingAnnotation
    RenderingCore
    RenderingFreeType
    RenderingOpenGL2
    REQUIRED)

set(MESHSTUDIO_SOURCES
    src/app/main.cpp
    src/ui/window/MainWindow.h
    src/ui/window/MainWindow.cpp
    src/ui/viewport/ViewportWidget.h
    src/ui/viewport/ViewportWidget.cpp
    src/cad/import/ShapeLoader.h
    src/cad/import/ShapeLoader.cpp
    src/cad/import/ShapeToPolyData.h
    src/cad/import/ShapeToPolyData.cpp
    src/cad/import/ShapeTessellator.h
    src/cad/import/ShapeTessellator.cpp
    src/cad/occ/OccCadGeometry.h
    src/cad/occ/OccCadGeometry.cpp
    src/app/meshing/SurfaceMeshEngine.h
    src/app/meshing/SurfaceMeshEngine.cpp
    src/cad/occ/OccAlgorithm6Adapter.cpp
    src/cad/occ/OccMeshVertexParameters.cpp
    src/app/meshing/TetMeshParams.h
    src/app/meshing/TetMeshEngine.h
    src/app/meshing/TetMeshEngine.cpp
    src/project/MeshProject.h
    src/project/ProjectPackage.h
    src/project/ProjectPackage.cpp
    src/project/ProjectArchive.h
    src/project/ProjectArchive.cpp
    src/ui/platform/WindowChrome.h
    src/ui/platform/WindowChrome.cpp
    src/ui/panels/ModelTreePanel.h
    src/ui/panels/ModelTreePanel.cpp
    src/ui/panels/OutputDock.h
    src/ui/panels/OutputDock.cpp
    src/ui/panels/PropertiesPanel.h
    src/ui/panels/PropertiesPanel.cpp
    src/ui/dialogs/ModelTransformDialog.h
    src/ui/dialogs/ModelTransformDialog.cpp
    src/ui/dialogs/MeshSettingsDialog.h
    src/ui/dialogs/MeshSettingsDialog.cpp
    src/ui/forms/DialogFormFactory.h
    src/ui/forms/DialogFormFactory.cpp
)

add_executable(MeshStudio WIN32
    ${MESHSTUDIO_SOURCES}
    resources/resources.qrc
)

# OpenCASCADE 7.6.0（本机从源码构建：头文件在 build/inc，库在 install）
set(OCC_INCLUDE_DIR "F:/ProgramData/opencascade-7.6.0/build/inc")
set(OCC_LIB_DIR "F:/ProgramData/opencascade-7.6.0/install/win64/vc14/lib")
set(OCC_LIB_DIR_DEBUG "F:/ProgramData/opencascade-7.6.0/install/win64/vc14/libd")

# QuaZip/zlib：复用 EMStudio 3rdparty 预编译静态库（工程包 .msp 的 ZIP 容器）
set(QUAZIP_INC_DIR "F:/Code/EMStudio/EMStudio/3rdparty/QuaZip/include")
set(QUAZIP_LIB_DIR "F:/Code/EMStudio/EMStudio/3rdparty/QuaZip/lib")
set(ZLIB_INC_DIR "F:/Code/EMStudio/EMStudio/3rdparty/zlib/include")
set(ZLIB_LIB_DIR "F:/Code/EMStudio/EMStudio/3rdparty/zlib/lib")

target_include_directories(MeshStudio PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OCC_INCLUDE_DIR}
    ${QUAZIP_INC_DIR}
    ${ZLIB_INC_DIR})

target_compile_definitions(MeshStudio PRIVATE QUAZIP_STATIC)

target_link_directories(MeshStudio PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>"
    "$<$<CONFIG:Release>:${OCC_LIB_DIR}>"
    ${QUAZIP_LIB_DIR}
    ${ZLIB_LIB_DIR})

set(OCC_LIBS
    TKernel TKMath TKG2d TKG3d TKGeomBase TKGeomAlgo
    TKBRep TKTopAlgo TKPrim TKBO TKShHealing TKMesh
    TKXSBase TKSTEPBase TKSTEPAttr TKSTEP209 TKSTEP TKIGES)
set(OCC_LIBS_D "")
foreach(occLib ${OCC_LIBS})
    list(APPEND OCC_LIBS_D "${occLib}d")
endforeach()

target_link_libraries(MeshStudio PRIVATE
    Qt5::Widgets
    Qt5::Svg
    Qt5::Concurrent
    ${VTK_LIBRARIES}
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>"
    "$<$<CONFIG:Release>:${OCC_LIBS}>"
    "$<$<CONFIG:Debug>:quazip1-qt5d.lib>"
    "$<$<CONFIG:Release>:quazip1-qt5.lib>"
    "$<$<CONFIG:Debug>:zsd.lib>"
    "$<$<CONFIG:Release>:zs.lib>"
    dwmapi   # Win11 标题栏着色（DwmSetWindowAttribute）
)

vtk_module_autoinit(TARGETS MeshStudio MODULES ${VTK_LIBRARIES})

# ---------- 自研网格算法库（纯 C++17，不依赖 Qt/VTK/Gmsh；3D 阶段沿用） ----------
add_library(mesher STATIC
    src/geometry/types/MesherTypes.h
    src/geometry/types/MesherTypes.cpp
    src/geometry/predicates/GeomPredicates.h
    src/geometry/predicates/GeomPredicates.cpp
    src/mesh/data/Mesh2D.h
    src/mesh/runtime/MeshingObserver.h
    src/mesh/data/Mesh2D.cpp
    src/mesh/data/Mesh2DTransaction.h
    src/mesh/data/Mesh2DTransaction.cpp
    src/mesh/surface/legacy/SurfaceDomainState.h
    src/mesh/surface/legacy/SurfaceDomainState.cpp
    src/mesh/surface/legacy/SurfaceRefinementQueue.h
    src/mesh/surface/legacy/SurfaceRefinementQueue.cpp
    src/mesh/planar/Delaunay2D.h
    src/mesh/planar/Delaunay2D.cpp
    src/mesh/planar/Constrained2D.h
    src/mesh/planar/Constrained2D.cpp
    src/mesh/surface/legacy/CadBoundaryRegistry.h
    src/mesh/surface/legacy/CadBoundaryRegistry.cpp
    src/mesh/surface/legacy/SurfaceChart.h
    src/mesh/surface/legacy/SurfaceChart.cpp
    src/mesh/surface/legacy/SurfaceTangentChart.h
    src/mesh/surface/legacy/SurfaceTangentChart.cpp
    src/mesh/surface/legacy/SurfaceLocalRemesh.h
    src/mesh/surface/legacy/SurfaceLocalRemesh.cpp
    src/mesh/surface/legacy/SurfaceBoundarySizeField.h
    src/mesh/surface/legacy/SurfaceBoundarySizeField.cpp
    src/mesh/surface/legacy/SurfaceSizingPolicy.h
    src/mesh/surface/legacy/SurfaceSizingPolicy.cpp
    src/mesh/surface/legacy/SurfaceQualityPolicy.h
    src/mesh/surface/legacy/SurfaceQualityPolicy.cpp
    src/mesh/surface/legacy/SurfaceRefinementCandidates.h
    src/mesh/surface/legacy/SurfaceRefinementCandidates.cpp
    src/mesh/surface/legacy/SurfaceFrontalDelaunay.h
    src/mesh/surface/legacy/SurfaceFrontalDelaunay.cpp
    src/mesh/surface/legacy/SurfaceReferenceFront.h
    src/mesh/surface/legacy/SurfaceReferenceFront.cpp
    src/mesh/surface/legacy/SurfaceReferenceBoundary.h
    src/mesh/surface/legacy/SurfaceReferenceBoundary.cpp
    src/mesh/surface/legacy/SurfaceMeshingPartition.h
    src/mesh/surface/legacy/SurfaceMeshingPartition.cpp
    src/mesh/surface/legacy/SurfaceEvaluationCache.h
    src/mesh/surface/legacy/SurfaceEvaluationCache.cpp
    src/mesh/planar/Ruppert2D.h
    src/mesh/planar/Ruppert2D.cpp
    src/mesh/data/TetMesh3D.h
    src/mesh/data/TetMesh3D.cpp
    src/mesh/volume/Delaunay3D.h
    src/mesh/volume/Delaunay3D.cpp
    src/mesh/volume/Constrained3D.h
    src/mesh/volume/Constrained3D.cpp
    src/mesh/volume/VolumeDomain3D.h
    src/mesh/volume/VolumeDomain3D.cpp
    src/mesh/validation/VolumeBoundaryValidation.h
    src/mesh/validation/VolumeBoundaryValidation.cpp
    src/mesh/volume/Ruppert3D.h
    src/mesh/volume/Ruppert3D.cpp
    src/mesh/volume/Flips3D.h
    src/mesh/volume/Flips3D.cpp
    src/mesh/volume/Quality3D.h
    src/mesh/volume/Quality3D.cpp
    src/mesh/volume/Smooth3D.h
    src/mesh/volume/Smooth3D.cpp
    src/mesh/surface/legacy/SurfaceMesh3D.h
    src/mesh/surface/legacy/SurfaceMesh3D.cpp
    src/mesh/validation/SurfaceMeshValidation.h
    src/mesh/validation/SurfaceMeshValidation.cpp
    src/mesh/surface/legacy/CadSurfaceMesher.h
    src/mesh/surface/legacy/CadSurfaceMesher.cpp)
target_include_directories(mesher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 固定 Gmsh 流程的自研复现模块：核心不链接 Gmsh。
target_sources(mesher PRIVATE src/mesh/surface/algorithm6/pipeline/SurfaceGenerator.cpp
    src/mesh/surface/algorithm6/pipeline/PeriodicSurfaceGeneration.cpp
    src/mesh/surface/algorithm6/pipeline/MeshAssembly.cpp)
target_sources(mesher PRIVATE
    src/mesh/surface/algorithm6/data/SurfaceTypes.h
    src/geometry/numerics/NativeSinCos.h
    src/geometry/numerics/NativeSinCos.cpp
    src/mesh/surface/algorithm6/boundary/MeshSizeEvaluator.h
    src/mesh/surface/algorithm6/boundary/MeshSizeEvaluator.cpp
    src/mesh/surface/algorithm6/boundary/CurveDiscretization.h
    src/mesh/surface/algorithm6/boundary/CurveDiscretization.cpp
    src/mesh/surface/algorithm6/boundary/BoundaryRegistry.h
    src/mesh/surface/algorithm6/boundary/BoundaryRegistry.cpp
    src/mesh/surface/algorithm6/chart/FaceChart.h
    src/mesh/surface/algorithm6/chart/FaceChart.cpp
    src/mesh/surface/algorithm6/chart/PeriodicFaceChart.h
    src/mesh/surface/algorithm6/chart/PeriodicFaceChart.cpp
    src/mesh/surface/algorithm6/initial/DivideAndConquerTriangulation.h
    src/mesh/surface/algorithm6/initial/DivideAndConquerTriangulation.cpp
    src/mesh/surface/algorithm6/initial/WalkTriangulation.h
    src/mesh/surface/algorithm6/initial/WalkTriangulation.cpp
    src/mesh/surface/algorithm6/data/HalfEdgeTopology.h
    src/mesh/surface/algorithm6/data/HalfEdgeTopology.cpp
    src/mesh/surface/algorithm6/initial/ConstraintRecovery.h
    src/mesh/surface/algorithm6/initial/ConstraintRecovery.cpp
    src/mesh/surface/algorithm6/initial/DomainClassification.h
    src/mesh/surface/algorithm6/initial/DomainClassification.cpp
    src/mesh/surface/algorithm6/initial/InitialUvLegalization.h
    src/mesh/surface/algorithm6/initial/InitialUvLegalization.cpp
    src/mesh/surface/algorithm6/data/BdsTopology.h
    src/mesh/surface/algorithm6/data/BdsTypes.h
    src/mesh/surface/algorithm6/data/BdsTopology.cpp
    src/mesh/surface/algorithm6/bds/BdsMeshConversion.h
    src/mesh/surface/algorithm6/bds/BdsMeshConversion.cpp
    src/mesh/surface/algorithm6/data/FaceMeshTypes.h
    src/mesh/surface/algorithm6/postprocess/FaceMeshConversion.h
    src/mesh/surface/algorithm6/postprocess/FaceMeshConversion.cpp
    src/mesh/surface/algorithm6/postprocess/PeriodicSeamReconstruction.h
    src/mesh/surface/algorithm6/postprocess/PeriodicSeamReconstruction.cpp
    src/mesh/surface/algorithm6/front/NodeSizeInitialization.h
    src/mesh/surface/algorithm6/front/NodeSizeInitialization.cpp
    src/mesh/surface/algorithm6/data/FrontMeshTypes.h
    src/mesh/surface/algorithm6/front/FrontTriangleInitialization.h
    src/mesh/surface/algorithm6/front/FrontTriangleInitialization.cpp
    src/mesh/surface/algorithm6/front/FrontTriangleOrdering.h
    src/mesh/surface/algorithm6/front/FrontTriangleOrdering.cpp
    src/mesh/surface/algorithm6/front/FrontQueue.h
    src/mesh/surface/algorithm6/front/FrontQueue.cpp
    src/mesh/surface/algorithm6/front/FrontPointPlacement.h
    src/mesh/surface/algorithm6/front/FrontPointPlacement.cpp
    src/mesh/surface/algorithm6/front/FrontPointLocation.h
    src/mesh/surface/algorithm6/front/FrontPointLocation.cpp
    src/mesh/surface/algorithm6/front/FrontCavity.h
    src/mesh/surface/algorithm6/front/FrontCavity.cpp
    src/mesh/surface/algorithm6/front/FrontInsertion.h
    src/mesh/surface/algorithm6/front/FrontInsertion.cpp
    src/mesh/surface/algorithm6/chart/ParametricDomain.h
    src/mesh/surface/algorithm6/chart/ParametricDomain.cpp
    src/mesh/surface/algorithm6/front/FrontGeneration.h
    src/mesh/surface/algorithm6/front/FrontGeneration.cpp
    src/mesh/surface/algorithm6/postprocess/FrontMeshTransfer.h
    src/mesh/surface/algorithm6/postprocess/FrontMeshTransfer.cpp
    src/mesh/surface/algorithm6/postprocess/SurfaceSmoothing.h
    src/mesh/surface/algorithm6/postprocess/SurfaceSmoothing.cpp
    src/mesh/surface/algorithm6/geometry/MeshVertexParameters.h
    src/mesh/surface/algorithm6/geometry/MeshVertexParameters.cpp
    src/mesh/surface/algorithm6/bds/BdsConstraintRecovery.h
    src/mesh/surface/algorithm6/bds/BdsConstraintRecovery.cpp
    src/mesh/surface/algorithm6/bds/BdsCurveRecovery.h
    src/mesh/surface/algorithm6/bds/BdsCurveRecovery.cpp
    src/mesh/surface/algorithm6/bds/PeriodicBdsRecovery.h
    src/mesh/surface/algorithm6/bds/PeriodicBdsRecovery.cpp
    src/mesh/surface/algorithm6/bds/BdsDomainClassification.h
    src/mesh/surface/algorithm6/bds/BdsDomainClassification.cpp
    src/mesh/surface/algorithm6/bds/BdsNodeSizeInitialization.h
    src/mesh/surface/algorithm6/bds/BdsNodeSizeInitialization.cpp
    src/mesh/surface/algorithm6/bds/BdsInteriorPreparation.h
    src/mesh/surface/algorithm6/bds/BdsInteriorPreparation.cpp
    src/mesh/surface/algorithm6/bds/BdsSwapQualification.h
    src/mesh/surface/algorithm6/bds/BdsSwapQualification.cpp
    src/mesh/surface/algorithm6/bds/InitialMeshLegalization.h
    src/mesh/surface/algorithm6/bds/InitialMeshLegalization.cpp
    src/mesh/surface/algorithm6/initial/InitialTriangulation.h
    src/mesh/surface/algorithm6/initial/InitialTriangulation.cpp
    src/mesh/surface/algorithm6/geometry/SurfaceMetric.h
    src/mesh/surface/algorithm6/geometry/SurfaceMetric.cpp
    src/geometry/numerics/LinearSystem3.h
    src/geometry/numerics/LinearSystem3.cpp
    src/geometry/numerics/SingularMatrixInverse3.h
    src/geometry/numerics/SingularMatrixInverse3.cpp
    src/mesh/surface/algorithm6/geometry/SurfaceParameterInverse.h
    src/mesh/surface/algorithm6/geometry/SurfaceParameterInverse.cpp
    src/mesh/surface/algorithm6/geometry/CurveSurfaceIntersection.h
    src/mesh/surface/algorithm6/geometry/CurveSurfaceIntersection.cpp)
if(MSVC AND CMAKE_SIZEOF_VOID_P EQUAL 8 AND CMAKE_SYSTEM_PROCESSOR MATCHES "AMD64|amd64|x86_64")
    enable_language(ASM_MASM)
    target_sources(mesher PRIVATE src/geometry/numerics/NativeSinCos_x64.asm)
    target_compile_definitions(mesher PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:MESHSTUDIO_NATIVE_SINCOS_X64>")
endif()

# 主程序使用自研引擎（取代 Gmsh）
target_link_libraries(MeshStudio PRIVATE mesher)

include(cmake/MeshStudioSourceGroups.cmake)

if(BUILD_TESTING)
    enable_testing()
    include(cmake/MeshStudioTests.cmake)
else()
    # 同一目录曾启用测试时，覆盖旧注册清单，避免 ctest 继续列出过期工程。
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/CTestTestfile.cmake"
        CONTENT "# MeshStudio: BUILD_TESTING=OFF; no tests registered.\n")
endif()
