# 测试、CAD 重放及基准工具；由根 CMake 在 BUILD_TESTING=ON 时包含。
# 相对源码路径按根工程目录解析；本文件不增删产品库源码。

# ---------- 工程存储层自测（控制台，tests/ProjectArchiveTest.cpp） ----------
add_executable(ProjectArchiveTest tests/ProjectArchiveTest.cpp
    src/project/MeshProject.h
    src/app/meshing/TetMeshParams.h
    src/app/meshing/TetMeshEngine.h
    src/project/ProjectPackage.h
    src/project/ProjectPackage.cpp
    src/project/ProjectArchive.h
    src/project/ProjectArchive.cpp)

target_include_directories(ProjectArchiveTest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${QUAZIP_INC_DIR}
    ${ZLIB_INC_DIR})

target_compile_definitions(ProjectArchiveTest PRIVATE QUAZIP_STATIC)

target_link_directories(ProjectArchiveTest PRIVATE
    ${QUAZIP_LIB_DIR}
    ${ZLIB_LIB_DIR})

target_link_libraries(ProjectArchiveTest PRIVATE
    Qt5::Core
    ${VTK_LIBRARIES}
    quazip1-qt5.lib
    zs.lib)

vtk_module_autoinit(TARGETS ProjectArchiveTest MODULES ${VTK_LIBRARIES})
add_test(NAME ProjectArchiveReferenceTest COMMAND ProjectArchiveTest reference)
add_test(NAME ProjectArchiveReferencePhysicalTest COMMAND ProjectArchiveTest reference-physical)
set_tests_properties(ProjectArchiveReferenceTest ProjectArchiveReferencePhysicalTest PROPERTIES LABELS unit TIMEOUT 60)

add_executable(GmshSurfacePipelineTest tests/GmshSurfacePipelineTest.cpp
    src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(GmshSurfacePipelineTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshSurfacePipelineTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshSurfacePipelineTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME GmshSurfacePipelineTest COMMAND GmshSurfacePipelineTest)
set_tests_properties(GmshSurfacePipelineTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 120)

add_executable(GmshMeshAssemblyTest tests/GmshMeshAssemblyTest.cpp)
target_link_libraries(GmshMeshAssemblyTest PRIVATE mesher)
add_test(NAME GmshMeshAssemblyTest COMMAND GmshMeshAssemblyTest)
set_tests_properties(GmshMeshAssemblyTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshSurfaceCadAuditQueryTest tests/GmshSurfaceCadAuditQueryTest.cpp)
target_include_directories(GmshSurfaceCadAuditQueryTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshSurfaceCadAuditQueryTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshSurfaceCadAuditQueryTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME GmshSurfaceCadAuditQueryTest COMMAND GmshSurfaceCadAuditQueryTest)
add_test(NAME GmshSurfaceCadAuditDistanceTest COMMAND GmshSurfaceCadAuditQueryTest distance)
add_test(NAME GmshSurfaceCadAuditRoundedTest COMMAND GmshSurfaceCadAuditQueryTest rounded)
set_tests_properties(GmshSurfaceCadAuditQueryTest GmshSurfaceCadAuditDistanceTest GmshSurfaceCadAuditRoundedTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshSurfaceReproductionBenchmark tests/GmshSurfaceReproductionBenchmark.cpp
    src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(GmshSurfaceReproductionBenchmark PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshSurfaceReproductionBenchmark PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshSurfaceReproductionBenchmark PRIVATE mesher VTK::CommonCore VTK::CommonDataModel VTK::IOXML
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(Algorithm6AcceptanceProbe tests/Algorithm6AcceptanceProbe.cpp
    src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(Algorithm6AcceptanceProbe PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(Algorithm6AcceptanceProbe PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(Algorithm6AcceptanceProbe PRIVATE mesher VTK::CommonCore VTK::CommonDataModel VTK::IOXML
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME Algorithm6AcceptanceProbeSelfTest COMMAND Algorithm6AcceptanceProbe self-test)
set_tests_properties(Algorithm6AcceptanceProbeSelfTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshSurfaceInitializationProfile tests/GmshSurfaceInitializationProfile.cpp
    src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(GmshSurfaceInitializationProfile PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshSurfaceInitializationProfile PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshSurfaceInitializationProfile PRIVATE mesher VTK::CommonCore VTK::CommonDataModel VTK::IOXML
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(OccAlgorithm6SphereEvaluationTest tests/OccAlgorithm6SphereEvaluationTest.cpp
    src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(OccAlgorithm6SphereEvaluationTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(OccAlgorithm6SphereEvaluationTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(OccAlgorithm6SphereEvaluationTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME OccAlgorithm6SphereEvaluationTest COMMAND OccAlgorithm6SphereEvaluationTest)
set_tests_properties(OccAlgorithm6SphereEvaluationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshMeshSizeEvaluatorTest tests/GmshMeshSizeEvaluatorTest.cpp)
target_link_libraries(GmshMeshSizeEvaluatorTest PRIVATE mesher)
add_test(NAME GmshMeshSizeEvaluatorTest COMMAND GmshMeshSizeEvaluatorTest)
set_tests_properties(GmshMeshSizeEvaluatorTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshCurveDiscretizationTest tests/GmshCurveDiscretizationTest.cpp)
target_link_libraries(GmshCurveDiscretizationTest PRIVATE mesher)
add_test(NAME GmshCurveDiscretizationTest COMMAND GmshCurveDiscretizationTest)
set_tests_properties(GmshCurveDiscretizationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshCurveNodeFilteringTest tests/GmshCurveNodeFilteringTest.cpp)
target_link_libraries(GmshCurveNodeFilteringTest PRIVATE mesher)
add_test(NAME GmshCurveNodeFilteringTest COMMAND GmshCurveNodeFilteringTest)
set_tests_properties(GmshCurveNodeFilteringTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshBoundaryRegistryTest tests/GmshBoundaryRegistryTest.cpp)
target_link_libraries(GmshBoundaryRegistryTest PRIVATE mesher)
add_test(NAME GmshBoundaryRegistryTest COMMAND GmshBoundaryRegistryTest)
set_tests_properties(GmshBoundaryRegistryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshBoundaryRecoveryTest tests/GmshBoundaryRecoveryTest.cpp)
target_link_libraries(GmshBoundaryRecoveryTest PRIVATE mesher)
add_test(NAME GmshBoundaryRecoveryTest COMMAND GmshBoundaryRecoveryTest)
set_tests_properties(GmshBoundaryRecoveryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshFaceChartTest tests/GmshFaceChartTest.cpp)
target_link_libraries(GmshFaceChartTest PRIVATE mesher)
add_test(NAME GmshFaceChartTest COMMAND GmshFaceChartTest)
set_tests_properties(GmshFaceChartTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshFaceChartReplay tests/GmshFaceChartReplay.cpp)
target_link_libraries(GmshFaceChartReplay PRIVATE mesher)

add_executable(GmshPeriodicBoundaryLoopTest tests/GmshPeriodicBoundaryLoopTest.cpp)
target_link_libraries(GmshPeriodicBoundaryLoopTest PRIVATE mesher)
add_test(NAME GmshPeriodicBoundaryLoopTest COMMAND GmshPeriodicBoundaryLoopTest)
set_tests_properties(GmshPeriodicBoundaryLoopTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshPeriodicBoundaryLoopReplay tests/GmshPeriodicBoundaryLoopReplay.cpp)
target_link_libraries(GmshPeriodicBoundaryLoopReplay PRIVATE mesher)

add_executable(GmshPeriodicBoundaryChartTest tests/GmshPeriodicBoundaryChartTest.cpp)
target_link_libraries(GmshPeriodicBoundaryChartTest PRIVATE mesher)
add_test(NAME GmshPeriodicBoundaryChartTest COMMAND GmshPeriodicBoundaryChartTest)
set_tests_properties(GmshPeriodicBoundaryChartTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshPeriodicBoundaryChartReplay tests/GmshPeriodicBoundaryChartReplay.cpp)
target_link_libraries(GmshPeriodicBoundaryChartReplay PRIVATE mesher)

add_executable(GmshPeriodicEmbeddedChartTest tests/GmshPeriodicEmbeddedChartTest.cpp)
target_link_libraries(GmshPeriodicEmbeddedChartTest PRIVATE mesher)
add_test(NAME GmshPeriodicEmbeddedChartTest COMMAND GmshPeriodicEmbeddedChartTest)
set_tests_properties(GmshPeriodicEmbeddedChartTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshTrueBoundaryTest tests/GmshTrueBoundaryTest.cpp)
add_executable(GmshDivideAndConquerTest tests/GmshDivideAndConquerTest.cpp)
target_link_libraries(GmshDivideAndConquerTest PRIVATE mesher)
add_test(NAME GmshDivideAndConquerTest COMMAND GmshDivideAndConquerTest)
set_tests_properties(GmshDivideAndConquerTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshDivideAndConquerReplay tests/GmshDivideAndConquerReplay.cpp)
target_link_libraries(GmshDivideAndConquerReplay PRIVATE mesher)
add_executable(GmshPeriodicInitialTriangulationTest tests/GmshPeriodicInitialTriangulationTest.cpp)
target_link_libraries(GmshPeriodicInitialTriangulationTest PRIVATE mesher)
add_test(NAME GmshPeriodicInitialTriangulationTest COMMAND GmshPeriodicInitialTriangulationTest)
set_tests_properties(GmshPeriodicInitialTriangulationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshWalkTriangulationTest tests/GmshWalkTriangulationTest.cpp)
target_link_libraries(GmshWalkTriangulationTest PRIVATE mesher)
add_test(NAME GmshWalkTriangulationTest COMMAND GmshWalkTriangulationTest)
set_tests_properties(GmshWalkTriangulationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshWalkTriangulationReplay tests/GmshWalkTriangulationReplay.cpp)
target_link_libraries(GmshWalkTriangulationReplay PRIVATE mesher)
add_executable(GmshConstraintRecoveryTest tests/GmshConstraintRecoveryTest.cpp)
target_link_libraries(GmshConstraintRecoveryTest PRIVATE mesher)
add_test(NAME GmshConstraintRecoveryTest COMMAND GmshConstraintRecoveryTest)
set_tests_properties(GmshConstraintRecoveryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshConstraintRecoveryReplay tests/GmshConstraintRecoveryReplay.cpp)
target_link_libraries(GmshConstraintRecoveryReplay PRIVATE mesher)
add_executable(GmshInitialDomainTest tests/GmshInitialDomainTest.cpp)
target_link_libraries(GmshInitialDomainTest PRIVATE mesher)
add_test(NAME GmshInitialDomainTest COMMAND GmshInitialDomainTest)
set_tests_properties(GmshInitialDomainTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshBdsMeshConversionTest tests/GmshBdsMeshConversionTest.cpp)
target_link_libraries(GmshBdsMeshConversionTest PRIVATE mesher)
add_test(NAME GmshBdsMeshConversionTest COMMAND GmshBdsMeshConversionTest)
set_tests_properties(GmshBdsMeshConversionTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshBdsMeshConversionReplay tests/GmshBdsMeshConversionReplay.cpp)
target_link_libraries(GmshBdsMeshConversionReplay PRIVATE mesher)

# 同一前缀重放的CAD版本独立求导；纯算法版本仍不链接OCC/Gmsh。
add_executable(GmshInitialMeshLegalizationCadReplay tests/GmshBdsMeshConversionReplay.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_compile_definitions(GmshInitialMeshLegalizationCadReplay PRIVATE MESHSTUDIO_REFERENCE_CAD_METRIC)
target_include_directories(GmshInitialMeshLegalizationCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshInitialMeshLegalizationCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshInitialMeshLegalizationCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(GmshBdsConstraintRecoveryTest tests/GmshBdsConstraintRecoveryTest.cpp)
target_link_libraries(GmshBdsConstraintRecoveryTest PRIVATE mesher)
add_test(NAME GmshBdsConstraintRecoveryTest COMMAND GmshBdsConstraintRecoveryTest)
set_tests_properties(GmshBdsConstraintRecoveryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshBdsConstraintRecoveryReplay tests/GmshBdsConstraintRecoveryReplay.cpp)
target_link_libraries(GmshBdsConstraintRecoveryReplay PRIVATE mesher)
add_executable(GmshBdsCurveRecoveryTest tests/GmshBdsCurveRecoveryTest.cpp)
target_link_libraries(GmshBdsCurveRecoveryTest PRIVATE mesher)
add_test(NAME GmshBdsCurveRecoveryTest COMMAND GmshBdsCurveRecoveryTest)
set_tests_properties(GmshBdsCurveRecoveryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshPeriodicBdsRecoveryTest tests/GmshPeriodicBdsRecoveryTest.cpp)
target_link_libraries(GmshPeriodicBdsRecoveryTest PRIVATE mesher)
add_test(NAME GmshPeriodicBdsRecoveryTest COMMAND GmshPeriodicBdsRecoveryTest)
set_tests_properties(GmshPeriodicBdsRecoveryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshBdsDomainClassificationTest tests/GmshBdsDomainClassificationTest.cpp)
target_link_libraries(GmshBdsDomainClassificationTest PRIVATE mesher)
add_test(NAME GmshBdsDomainClassificationTest COMMAND GmshBdsDomainClassificationTest)
set_tests_properties(GmshBdsDomainClassificationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
  add_executable(GmshPeriodicBdsDomainTest tests/GmshPeriodicBdsDomainTest.cpp)
  target_link_libraries(GmshPeriodicBdsDomainTest PRIVATE mesher)
  add_test(NAME GmshPeriodicBdsDomainTest COMMAND GmshPeriodicBdsDomainTest)
  set_tests_properties(GmshPeriodicBdsDomainTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
  add_executable(GmshFaceMeshConversionTest tests/GmshFaceMeshConversionTest.cpp)
  target_link_libraries(GmshFaceMeshConversionTest PRIVATE mesher)
  add_test(NAME GmshFaceMeshConversionTest COMMAND GmshFaceMeshConversionTest)
  set_tests_properties(GmshFaceMeshConversionTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
  add_executable(GmshFaceMeshConversionReplay tests/GmshFaceMeshConversionReplay.cpp)
  target_link_libraries(GmshFaceMeshConversionReplay PRIVATE mesher)
add_executable(GmshNodeSizeInitializationTest tests/GmshNodeSizeInitializationTest.cpp)
target_link_libraries(GmshNodeSizeInitializationTest PRIVATE mesher)
add_test(NAME GmshNodeSizeInitializationTest COMMAND GmshNodeSizeInitializationTest)
set_tests_properties(GmshNodeSizeInitializationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontTriangleInitializationTest tests/GmshFrontTriangleInitializationTest.cpp)
target_link_libraries(GmshFrontTriangleInitializationTest PRIVATE mesher)
add_test(NAME GmshFrontTriangleInitializationTest COMMAND GmshFrontTriangleInitializationTest)
set_tests_properties(GmshFrontTriangleInitializationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontQueueTest tests/GmshFrontQueueTest.cpp)
target_link_libraries(GmshFrontQueueTest PRIVATE mesher)
add_test(NAME GmshFrontQueueTest COMMAND GmshFrontQueueTest)
set_tests_properties(GmshFrontQueueTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontQueueReplay tests/GmshFrontQueueReplay.cpp)
target_link_libraries(GmshFrontQueueReplay PRIVATE mesher)
add_executable(GmshFrontPointPlacementTest tests/GmshFrontPointPlacementTest.cpp)
target_link_libraries(GmshFrontPointPlacementTest PRIVATE mesher)
add_test(NAME GmshFrontPointPlacementTest COMMAND GmshFrontPointPlacementTest)
set_tests_properties(GmshFrontPointPlacementTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontPointPlacementCadReplay tests/GmshFrontPointPlacementCadReplay.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshFrontPointPlacementCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshFrontPointPlacementCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshFrontPointPlacementCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(GmshFrontPointLocationTest tests/GmshFrontPointLocationTest.cpp)
target_link_libraries(GmshFrontPointLocationTest PRIVATE mesher)
add_test(NAME GmshFrontPointLocationTest COMMAND GmshFrontPointLocationTest)
set_tests_properties(GmshFrontPointLocationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontCavityTest tests/GmshFrontCavityTest.cpp)
target_link_libraries(GmshFrontCavityTest PRIVATE mesher)
add_test(NAME GmshFrontCavityTest COMMAND GmshFrontCavityTest)
set_tests_properties(GmshFrontCavityTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontCavityReplay tests/GmshFrontCavityReplay.cpp)
target_link_libraries(GmshFrontCavityReplay PRIVATE mesher)
add_executable(GmshFrontInsertionTest tests/GmshFrontInsertionTest.cpp)
target_link_libraries(GmshFrontInsertionTest PRIVATE mesher)
add_test(NAME GmshFrontInsertionTest COMMAND GmshFrontInsertionTest)
set_tests_properties(GmshFrontInsertionTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontInsertionReplay tests/GmshFrontInsertionReplay.cpp)
target_link_libraries(GmshFrontInsertionReplay PRIVATE mesher)
add_executable(GmshFrontPointInsertionTest tests/GmshFrontPointInsertionTest.cpp)
target_link_libraries(GmshFrontPointInsertionTest PRIVATE mesher)
add_test(NAME GmshFrontPointInsertionTest COMMAND GmshFrontPointInsertionTest)
set_tests_properties(GmshFrontPointInsertionTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontPointInsertionCadReplay tests/GmshFrontPointInsertionCadReplay.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshFrontPointInsertionCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshFrontPointInsertionCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshFrontPointInsertionCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(GmshParametricDomainTest tests/GmshParametricDomainTest.cpp)
target_link_libraries(GmshParametricDomainTest PRIVATE mesher)
add_test(NAME GmshParametricDomainTest COMMAND GmshParametricDomainTest)
set_tests_properties(GmshParametricDomainTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontGenerationTest tests/GmshFrontGenerationTest.cpp)
target_link_libraries(GmshFrontGenerationTest PRIVATE mesher)
add_test(NAME GmshFrontGenerationTest COMMAND GmshFrontGenerationTest)
set_tests_properties(GmshFrontGenerationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontMeshTransferTest tests/GmshFrontMeshTransferTest.cpp)
target_link_libraries(GmshFrontMeshTransferTest PRIVATE mesher)
add_test(NAME GmshFrontMeshTransferTest COMMAND GmshFrontMeshTransferTest)
set_tests_properties(GmshFrontMeshTransferTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshFrontMeshTransferReplay tests/GmshFrontMeshTransferReplay.cpp)
target_link_libraries(GmshFrontMeshTransferReplay PRIVATE mesher)
add_executable(GmshSurfaceSmoothingTest tests/GmshSurfaceSmoothingTest.cpp)
add_executable(GmshMeshVertexParametersTest tests/GmshMeshVertexParametersTest.cpp)
target_link_libraries(GmshMeshVertexParametersTest PRIVATE mesher)
add_test(NAME GmshMeshVertexParametersTest COMMAND GmshMeshVertexParametersTest)
set_tests_properties(GmshMeshVertexParametersTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshMeshVertexParametersCadReplay tests/GmshMeshVertexParametersCadReplay.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(GmshMeshVertexParametersCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshMeshVertexParametersCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshMeshVertexParametersCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
target_link_libraries(GmshSurfaceSmoothingTest PRIVATE mesher)
add_test(NAME GmshSurfaceSmoothingTest COMMAND GmshSurfaceSmoothingTest)
set_tests_properties(GmshSurfaceSmoothingTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshSurfaceSmoothingCadReplay tests/GmshSurfaceSmoothingCadReplay.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(GmshSurfaceSmoothingCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshSurfaceSmoothingCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshSurfaceSmoothingCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(GmshFrontGenerationCadReplay tests/GmshFrontGenerationCadReplay.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshFrontGenerationCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshFrontGenerationCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshFrontGenerationCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_executable(GmshParametricDomainReplay tests/GmshParametricDomainReplay.cpp)
target_link_libraries(GmshParametricDomainReplay PRIVATE mesher)
add_executable(GmshBdsNodeSizeInitializationTest tests/GmshBdsNodeSizeInitializationTest.cpp)
target_link_libraries(GmshBdsNodeSizeInitializationTest PRIVATE mesher)
add_test(NAME GmshBdsNodeSizeInitializationTest COMMAND GmshBdsNodeSizeInitializationTest)
set_tests_properties(GmshBdsNodeSizeInitializationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshBdsInteriorPreparationTest tests/GmshBdsInteriorPreparationTest.cpp)
target_link_libraries(GmshBdsInteriorPreparationTest PRIVATE mesher)
add_test(NAME GmshBdsInteriorPreparationTest COMMAND GmshBdsInteriorPreparationTest)
set_tests_properties(GmshBdsInteriorPreparationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshInitialMeshLegalizationTest tests/GmshInitialMeshLegalizationTest.cpp)
target_link_libraries(GmshInitialMeshLegalizationTest PRIVATE mesher)
add_test(NAME GmshInitialMeshLegalizationTest COMMAND GmshInitialMeshLegalizationTest)
set_tests_properties(GmshInitialMeshLegalizationTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)
add_executable(GmshInitialMeshLegalizationReplay tests/GmshInitialMeshLegalizationReplay.cpp)
target_link_libraries(GmshInitialMeshLegalizationReplay PRIVATE mesher)
target_link_libraries(GmshTrueBoundaryTest PRIVATE mesher)
add_test(NAME GmshTrueBoundaryTest COMMAND GmshTrueBoundaryTest)
set_tests_properties(GmshTrueBoundaryTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshSurfaceMetricTest tests/GmshSurfaceMetricTest.cpp)
target_link_libraries(GmshSurfaceMetricTest PRIVATE mesher)
add_test(NAME GmshSurfaceMetricTest COMMAND GmshSurfaceMetricTest)
set_tests_properties(GmshSurfaceMetricTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshGeometryReplay tests/GmshGeometryReplay.cpp)
target_link_libraries(GmshGeometryReplay PRIVATE mesher)

add_executable(GmshGeometryPredicatesTest tests/GmshGeometryPredicatesTest.cpp)
target_link_libraries(GmshGeometryPredicatesTest PRIVATE mesher)
add_test(NAME GmshGeometryPredicatesTest COMMAND GmshGeometryPredicatesTest)
set_tests_properties(GmshGeometryPredicatesTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshLinearSystem3Test tests/GmshLinearSystem3Test.cpp)
target_link_libraries(GmshLinearSystem3Test PRIVATE mesher)
add_test(NAME GmshLinearSystem3Test COMMAND GmshLinearSystem3Test)
set_tests_properties(GmshLinearSystem3Test PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshSingularMatrixInverse3Test tests/GmshSingularMatrixInverse3Test.cpp)
target_link_libraries(GmshSingularMatrixInverse3Test PRIVATE mesher)
add_test(NAME GmshSingularMatrixInverse3Test COMMAND GmshSingularMatrixInverse3Test)
set_tests_properties(GmshSingularMatrixInverse3Test PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshSingularMatrixInverse3Replay tests/GmshSingularMatrixInverse3Replay.cpp)
target_link_libraries(GmshSingularMatrixInverse3Replay PRIVATE mesher)

add_executable(GmshSurfaceParameterInverseTest tests/GmshSurfaceParameterInverseTest.cpp)
target_link_libraries(GmshSurfaceParameterInverseTest PRIVATE mesher)
add_test(NAME GmshSurfaceParameterInverseTest COMMAND GmshSurfaceParameterInverseTest)
set_tests_properties(GmshSurfaceParameterInverseTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshSurfaceParameterInverseCadReplay tests/GmshSurfaceParameterInverseCadReplay.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccParameterInverse.cpp)
target_include_directories(GmshSurfaceParameterInverseCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshSurfaceParameterInverseCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshSurfaceParameterInverseCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(GmshCurveSurfaceIntersectionTest tests/GmshCurveSurfaceIntersectionTest.cpp)
target_link_libraries(GmshCurveSurfaceIntersectionTest PRIVATE mesher)
add_test(NAME GmshCurveSurfaceIntersectionTest COMMAND GmshCurveSurfaceIntersectionTest)
set_tests_properties(GmshCurveSurfaceIntersectionTest PROPERTIES LABELS "unit;gmsh_reproduction" TIMEOUT 60)

# 显式参考重放工具，输入来自仓库外冻结的同一 BREP，不自动跑未提供的案例。
add_executable(GmshCurveCadReplay tests/GmshCurveCadReplay.cpp)
target_include_directories(GmshCurveCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshCurveCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshCurveCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(GmshBoundaryCadReplay tests/GmshBoundaryCadReplay.cpp)
target_include_directories(GmshBoundaryCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshBoundaryCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshBoundaryCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(GmshTrueBoundaryCadReplay tests/GmshTrueBoundaryCadReplay.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshTrueBoundaryCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshTrueBoundaryCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshTrueBoundaryCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(GmshPeriodicBoundaryChartCadReplay tests/GmshPeriodicBoundaryChartCadReplay.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshPeriodicBoundaryChartCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshPeriodicBoundaryChartCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshPeriodicBoundaryChartCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(GmshOccPcurveTest tests/GmshOccPcurveTest.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshOccPcurveTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshOccPcurveTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshOccPcurveTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME GmshOccPcurveTest COMMAND GmshOccPcurveTest)
set_tests_properties(GmshOccPcurveTest PROPERTIES LABELS "integration;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshOccProjectionTest tests/GmshOccProjectionTest.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshOccProjectionTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshOccProjectionTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshOccProjectionTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME GmshOccProjectionTest COMMAND GmshOccProjectionTest)
set_tests_properties(GmshOccProjectionTest PROPERTIES LABELS "integration;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshOccProjectionCadReplay tests/GmshOccProjectionCadReplay.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshOccProjectionCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshOccProjectionCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshOccProjectionCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(GmshOccSurfaceDerivativeTest tests/GmshOccSurfaceDerivativeTest.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(GmshOccSurfaceDerivativeTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshOccSurfaceDerivativeTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshOccSurfaceDerivativeTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME GmshOccSurfaceDerivativeTest COMMAND GmshOccSurfaceDerivativeTest)
set_tests_properties(GmshOccSurfaceDerivativeTest PROPERTIES LABELS "integration;gmsh_reproduction" TIMEOUT 60)

add_executable(GmshPeriodicEmbeddedCadReplay tests/GmshPeriodicEmbeddedCadReplay.cpp src/cad/occ/OccCadGeometry.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(GmshPeriodicEmbeddedCadReplay PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(GmshPeriodicEmbeddedCadReplay PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(GmshPeriodicEmbeddedCadReplay PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")

# ---------- 2D Delaunay 原型自测（控制台，tests/Delaunay2DTest.cpp） ----------
add_executable(Delaunay2DTest tests/Delaunay2DTest.cpp)
target_link_libraries(Delaunay2DTest PRIVATE mesher)

add_executable(Constrained2DRobustTest tests/Constrained2DRobustTest.cpp)
target_link_libraries(Constrained2DRobustTest PRIVATE mesher)

# ---------- 3D Delaunay 四面体化自测（控制台，tests/Delaunay3DTest.cpp） ----------
add_executable(Delaunay3DTest tests/Delaunay3DTest.cpp)
target_link_libraries(Delaunay3DTest PRIVATE mesher)

# ---------- 3D 表面约束恢复自测（控制台，tests/Constrained3DTest.cpp） ----------
add_executable(Constrained3DTest tests/Constrained3DTest.cpp)
target_link_libraries(Constrained3DTest PRIVATE mesher)

add_executable(GeomPredicates3DTest tests/GeomPredicates3DTest.cpp)
target_link_libraries(GeomPredicates3DTest PRIVATE mesher)
add_executable(VolumeBoundaryValidationTest tests/VolumeBoundaryValidationTest.cpp)
target_link_libraries(VolumeBoundaryValidationTest PRIVATE mesher)
add_executable(VolumeDomain3DTest tests/VolumeDomain3DTest.cpp)
target_link_libraries(VolumeDomain3DTest PRIVATE mesher)

# ---------- 3D Delaunay 细化自测（控制台，tests/Ruppert3DTest.cpp） ----------
add_executable(Ruppert3DTest tests/Ruppert3DTest.cpp)
target_link_libraries(Ruppert3DTest PRIVATE mesher)

# ---------- 3D 翻转框架自测（控制台，tests/Flips3DTest.cpp） ----------
add_executable(Flips3DTest tests/Flips3DTest.cpp)
target_link_libraries(Flips3DTest PRIVATE mesher)

# ---------- 顶点平滑自测（控制台，tests/Smooth3DTest.cpp） ----------
add_executable(Smooth3DTest tests/Smooth3DTest.cpp)
target_link_libraries(Smooth3DTest PRIVATE mesher)

# ---------- 表面网格输入准备自测（控制台，tests/SurfaceMesh3DTest.cpp） ----------
add_executable(SurfaceMesh3DTest tests/SurfaceMesh3DTest.cpp)
target_link_libraries(SurfaceMesh3DTest PRIVATE mesher)

add_executable(CadSurfaceMesherTest tests/CadSurfaceMesherTest.cpp)
target_link_libraries(CadSurfaceMesherTest PRIVATE mesher)
add_executable(SurfaceFrontalLayoutTest tests/SurfaceFrontalLayoutTest.cpp)
target_link_libraries(SurfaceFrontalLayoutTest PRIVATE mesher)
add_test(NAME SurfaceFrontalLayoutTest COMMAND SurfaceFrontalLayoutTest)
set_tests_properties(SurfaceFrontalLayoutTest PROPERTIES LABELS unit TIMEOUT 60)
add_test(NAME SurfaceReferenceCadLayoutTest COMMAND SurfaceFrontalLayoutTest reference)
add_test(NAME SurfaceReferencePhysicalCadLayoutTest COMMAND SurfaceFrontalLayoutTest reference-physical)
set_tests_properties(SurfaceReferenceCadLayoutTest SurfaceReferencePhysicalCadLayoutTest PROPERTIES LABELS unit TIMEOUT 60)
add_executable(SurfaceFrontalGenerationTest tests/SurfaceFrontalGenerationTest.cpp)
target_link_libraries(SurfaceFrontalGenerationTest PRIVATE mesher)
add_test(NAME SurfaceFrontalGenerationTest COMMAND SurfaceFrontalGenerationTest)
set_tests_properties(SurfaceFrontalGenerationTest PROPERTIES LABELS unit TIMEOUT 60)
add_executable(SurfaceReferenceFrontTest tests/SurfaceReferenceFrontTest.cpp)
target_link_libraries(SurfaceReferenceFrontTest PRIVATE mesher)
add_test(NAME SurfaceReferenceFrontTest COMMAND SurfaceReferenceFrontTest)
set_tests_properties(SurfaceReferenceFrontTest PROPERTIES LABELS unit TIMEOUT 60)
add_executable(SurfaceReferenceBoundaryTest tests/SurfaceReferenceBoundaryTest.cpp)
target_link_libraries(SurfaceReferenceBoundaryTest PRIVATE mesher)
add_test(NAME SurfaceReferenceBoundaryTest COMMAND SurfaceReferenceBoundaryTest)
set_tests_properties(SurfaceReferenceBoundaryTest PROPERTIES LABELS unit TIMEOUT 60)
add_executable(SurfaceMeshingPartitionTest tests/SurfaceMeshingPartitionTest.cpp)
target_link_libraries(SurfaceMeshingPartitionTest PRIVATE mesher)
add_test(NAME SurfaceMeshingPartitionTest COMMAND SurfaceMeshingPartitionTest)
set_tests_properties(SurfaceMeshingPartitionTest PROPERTIES LABELS unit TIMEOUT 60)
add_test(NAME CadSurfaceDenseBoundaryTest COMMAND CadSurfaceMesherTest --dense-boundary)
add_test(NAME CadSurfaceShapeQualityTest COMMAND CadSurfaceMesherTest --shape-quality)
add_test(NAME CadSurfaceNarrowQualityTest COMMAND CadSurfaceMesherTest --narrow-quality)
add_test(NAME CadSurfaceNarrowFrameTest COMMAND CadSurfaceMesherTest --narrow-frame)
set_tests_properties(CadSurfaceNarrowFrameTest PROPERTIES LABELS unit TIMEOUT 60)
add_test(NAME CadSurfaceQualityReuseTest COMMAND CadSurfaceMesherTest --quality-reuse)
set_tests_properties(CadSurfaceQualityReuseTest PROPERTIES LABELS unit TIMEOUT 60)
set_tests_properties(CadSurfaceNarrowQualityTest PROPERTIES LABELS unit TIMEOUT 60)
set_tests_properties(CadSurfaceShapeQualityTest PROPERTIES LABELS unit TIMEOUT 60)
set_tests_properties(CadSurfaceDenseBoundaryTest PROPERTIES LABELS unit TIMEOUT 60)
add_test(NAME CadSurfaceSharedBoundaryTest COMMAND CadSurfaceMesherTest --shared-boundary)
set_tests_properties(CadSurfaceSharedBoundaryTest PROPERTIES LABELS unit TIMEOUT 60)

add_executable(CadSurfacePerformanceTest tests/CadSurfacePerformanceTest.cpp)
target_link_libraries(CadSurfacePerformanceTest PRIVATE mesher)

add_executable(CadSurfaceQualityReportTest tests/CadSurfaceQualityReportTest.cpp)
target_link_libraries(CadSurfaceQualityReportTest PRIVATE mesher)
foreach(surface_test SurfaceBoundarySizeFieldTest SurfaceSizingPolicyTest SurfaceQualityPolicyTest SurfaceRefinementCandidatesTest SurfaceLocalRemeshTest)
    add_executable(${surface_test} tests/${surface_test}.cpp)
    target_link_libraries(${surface_test} PRIVATE mesher)
    add_test(NAME ${surface_test} COMMAND ${surface_test})
    set_tests_properties(${surface_test} PROPERTIES LABELS unit TIMEOUT 30)
endforeach()

add_executable(CadBenchmarkOptionsTest tests/CadBenchmarkOptionsTest.cpp)
target_link_libraries(CadBenchmarkOptionsTest PRIVATE mesher)

add_executable(SurfaceEvaluationCacheTest tests/SurfaceEvaluationCacheTest.cpp)
target_link_libraries(SurfaceEvaluationCacheTest PRIVATE mesher)

add_executable(Mesh2DTransactionTest tests/Mesh2DTransactionTest.cpp)
target_link_libraries(Mesh2DTransactionTest PRIVATE mesher)

add_executable(Mesh2DEditEquivalenceTest tests/Mesh2DEditEquivalenceTest.cpp)
target_link_libraries(Mesh2DEditEquivalenceTest PRIVATE mesher)

add_executable(SurfaceDomainStateTest tests/SurfaceDomainStateTest.cpp)
target_link_libraries(SurfaceDomainStateTest PRIVATE mesher)

add_executable(SurfaceRefinementQueueTest tests/SurfaceRefinementQueueTest.cpp)
target_link_libraries(SurfaceRefinementQueueTest PRIVATE mesher)

add_executable(CadBoundaryQueueTest tests/CadBoundaryQueueTest.cpp)
target_link_libraries(CadBoundaryQueueTest PRIVATE mesher)

add_executable(OccCadSurfaceTest tests/OccCadSurfaceTest.cpp
    src/cad/import/ShapeLoader.cpp
    src/cad/occ/OccCadGeometry.cpp
    src/app/meshing/SurfaceMeshEngine.cpp src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccMeshVertexParameters.cpp)
target_include_directories(OccCadSurfaceTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(OccCadSurfaceTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>"
    "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(OccCadSurfaceTest PRIVATE Qt5::Core mesher ${VTK_LIBRARIES}
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>"
    "$<$<CONFIG:Release>:${OCC_LIBS}>")
vtk_module_autoinit(TARGETS OccCadSurfaceTest MODULES ${VTK_LIBRARIES})

add_executable(CadSurfaceProjectRoundTripTest tests/CadSurfaceProjectRoundTripTest.cpp
    src/cad/import/ShapeLoader.cpp src/cad/occ/OccCadGeometry.cpp src/app/meshing/SurfaceMeshEngine.cpp src/cad/occ/OccAlgorithm6Adapter.cpp src/cad/occ/OccMeshVertexParameters.cpp
    src/project/ProjectArchive.cpp src/project/ProjectPackage.cpp)
target_include_directories(CadSurfaceProjectRoundTripTest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR} ${QUAZIP_INC_DIR} ${ZLIB_INC_DIR})
target_compile_definitions(CadSurfaceProjectRoundTripTest PRIVATE QUAZIP_STATIC)
target_link_directories(CadSurfaceProjectRoundTripTest PRIVATE ${QUAZIP_LIB_DIR} ${ZLIB_LIB_DIR}
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(CadSurfaceProjectRoundTripTest PRIVATE Qt5::Core mesher ${VTK_LIBRARIES} quazip1-qt5.lib zs.lib
    "$<$<CONFIG:Debug>:${OCC_LIBS_D};TKFeatd>" "$<$<CONFIG:Release>:${OCC_LIBS};TKFeat>")
vtk_module_autoinit(TARGETS CadSurfaceProjectRoundTripTest MODULES ${VTK_LIBRARIES})
add_test(NAME CadSurfaceProjectRoundTripTest_split_plane COMMAND CadSurfaceProjectRoundTripTest split-plane)
add_test(NAME CadSurfaceProjectRoundTripTest_split_cylinder COMMAND CadSurfaceProjectRoundTripTest split-cylinder)
add_test(NAME CadSurfaceProjectRoundTripTest_algorithm6_split_cylinder COMMAND CadSurfaceProjectRoundTripTest split-cylinder --algorithm6)
set_tests_properties(CadSurfaceProjectRoundTripTest_algorithm6_split_cylinder PROPERTIES LABELS e2e TIMEOUT 120)
set_tests_properties(CadSurfaceProjectRoundTripTest_split_plane CadSurfaceProjectRoundTripTest_split_cylinder PROPERTIES LABELS e2e TIMEOUT 120)
add_test(NAME TetMeshEngineTest_virtual_patch_guard COMMAND TetMeshEngineTest --reject-virtual-patch)
set_tests_properties(TetMeshEngineTest_virtual_patch_guard PROPERTIES LABELS unit TIMEOUT 60)

add_executable(CadGeometryAuditTest tests/CadGeometryAuditTest.cpp
    src/cad/import/ShapeLoader.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(CadGeometryAuditTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(CadGeometryAuditTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>"
    "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(CadGeometryAuditTest PRIVATE Qt5::Core mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>"
    "$<$<CONFIG:Release>:${OCC_LIBS}>")

add_executable(OccCadGeometryQueryTest tests/OccCadGeometryQueryTest.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(OccCadGeometryQueryTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(OccCadGeometryQueryTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(OccCadGeometryQueryTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME OccCadGeometryQueryTest COMMAND OccCadGeometryQueryTest)
set_tests_properties(OccCadGeometryQueryTest PROPERTIES LABELS e2e TIMEOUT 60)

add_executable(OccSurfacePartitionTest tests/OccSurfacePartitionTest.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(OccSurfacePartitionTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(OccSurfacePartitionTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(OccSurfacePartitionTest PRIVATE mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D};TKFeatd>" "$<$<CONFIG:Release>:${OCC_LIBS};TKFeat>")
add_test(NAME OccSurfacePartitionTest COMMAND OccSurfacePartitionTest)
set_tests_properties(OccSurfacePartitionTest PROPERTIES LABELS e2e TIMEOUT 120)

# Explicit performance runs only; heavy real-model timing is not a default CTest.
add_executable(CadSurfaceBenchmark tests/CadSurfaceBenchmark.cpp
    src/cad/import/ShapeLoader.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(CadSurfaceBenchmark PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(CadSurfaceBenchmark PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(CadSurfaceBenchmark PRIVATE Qt5::Core mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
target_compile_definitions(CadSurfaceBenchmark PRIVATE
    CAD_BENCH_SOURCE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}"
    CAD_BENCH_CONFIG="$<CONFIG>"
    CAD_BENCH_COMPILER="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}")
if(WIN32)
    target_link_libraries(CadSurfaceBenchmark PRIVATE Psapi)
endif()

add_executable(CadSurfaceQualityAuditTest tests/CadSurfaceQualityAuditTest.cpp
    src/cad/import/ShapeLoader.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(CadSurfaceQualityAuditTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(CadSurfaceQualityAuditTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>"
    "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(CadSurfaceQualityAuditTest PRIVATE Qt5::Core mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>"
    "$<$<CONFIG:Release>:${OCC_LIBS}>")

# Manual original-model diagnosis, outside the default regression suite.
add_executable(CadModelDiagnosis tests/CadModelDiagnosis.cpp
    src/cad/import/ShapeLoader.cpp src/cad/occ/OccCadGeometry.cpp)
target_include_directories(CadModelDiagnosis PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR})
target_link_directories(CadModelDiagnosis PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(CadModelDiagnosis PRIVATE Qt5::Core mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_test(NAME OccCadSurfaceTest_sphere_quality COMMAND OccCadSurfaceTest sphere-quality)
set_tests_properties(OccCadSurfaceTest_sphere_quality PROPERTIES LABELS e2e TIMEOUT 60)
add_test(NAME OccCadSurfaceTest_assembly_contact COMMAND OccCadSurfaceTest assembly-contact)
add_test(NAME OccCadSurfaceTest_assembly_overlap COMMAND OccCadSurfaceTest assembly-overlap)
add_test(NAME OccCadSurfaceTest_assembly_independent COMMAND OccCadSurfaceTest assembly-independent)
set_tests_properties(OccCadSurfaceTest_assembly_contact OccCadSurfaceTest_assembly_overlap PROPERTIES LABELS e2e TIMEOUT 60)

add_executable(CadSurfaceAuditCoverageTest tests/CadSurfaceAuditCoverageTest.cpp)
target_include_directories(CadSurfaceAuditCoverageTest PRIVATE ${OCC_INCLUDE_DIR})
target_link_directories(CadSurfaceAuditCoverageTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(CadSurfaceAuditCoverageTest PRIVATE Qt5::Core
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>")
add_dependencies(CadSurfaceAuditCoverageTest CadSurfaceQualityAuditTest)

add_executable(MeshSettingsDialogTest tests/MeshSettingsDialogTest.cpp
    src/ui/dialogs/MeshSettingsDialog.cpp src/ui/dialogs/MeshSettingsDialog.h
    src/ui/forms/DialogFormFactory.cpp src/ui/forms/DialogFormFactory.h src/ui/platform/WindowChrome.cpp)
target_include_directories(MeshSettingsDialogTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(MeshSettingsDialogTest PRIVATE Qt5::Widgets mesher dwmapi)
add_executable(DialogFormFactoryTest tests/DialogFormFactoryTest.cpp
    src/ui/forms/DialogFormFactory.cpp src/ui/forms/DialogFormFactory.h
    src/ui/dialogs/ModelTransformDialog.cpp src/ui/dialogs/ModelTransformDialog.h src/ui/platform/WindowChrome.cpp)
target_include_directories(DialogFormFactoryTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(DialogFormFactoryTest PRIVATE Qt5::Widgets dwmapi)
add_test(NAME CadSurfaceAuditCoverageTest COMMAND CadSurfaceAuditCoverageTest)
set_tests_properties(CadSurfaceAuditCoverageTest PROPERTIES LABELS e2e TIMEOUT 90)

# ---------- 自研引擎端到端自测（控制台，需 Qt+VTK，tests/TetMeshEngineTest.cpp） ----------
add_executable(TetQualityAcceptanceTest tests/TetQualityAcceptanceTest.cpp src/app/meshing/TetMeshEngine.cpp)
add_executable(VolumePipelineTest tests/VolumePipelineTest.cpp src/app/meshing/TetMeshEngine.cpp)
target_include_directories(VolumePipelineTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(VolumePipelineTest PRIVATE mesher Qt5::Core ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS VolumePipelineTest MODULES ${VTK_LIBRARIES})
add_executable(GmshTetPipelineTest tests/GmshTetPipelineTest.cpp src/app/meshing/TetMeshEngine.cpp)
target_include_directories(GmshTetPipelineTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(GmshTetPipelineTest PRIVATE mesher Qt5::Core ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS GmshTetPipelineTest MODULES ${VTK_LIBRARIES})
target_include_directories(TetQualityAcceptanceTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(TetQualityAcceptanceTest PRIVATE mesher Qt5::Core ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TetQualityAcceptanceTest MODULES ${VTK_LIBRARIES})
add_executable(TetMeshEngineTest
    tests/TetMeshEngineTest.cpp
    src/app/meshing/TetMeshEngine.cpp
)
target_include_directories(TetMeshEngineTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(TetMeshEngineTest PRIVATE
    Qt5::Core
    ${VTK_LIBRARIES}
    mesher
)

# ---------- B-Rep 几何输入端到端自测（控制台，需 Qt+VTK+OCC） ----------
# 验证 STEP/IGES/BRep → BRepMesh 逐面离散 → 容差焊接 → 四面体引擎 这条链路。
# 与 TetMeshEngineTest 的区别：这里额外链接 OCC（离散化必须用 OCC）。
add_executable(BRepInputTest
    tests/BRepInputTest.cpp
    src/app/meshing/TetMeshEngine.cpp
    src/cad/import/ShapeLoader.cpp
    src/cad/import/ShapeToPolyData.cpp
    src/cad/import/ShapeTessellator.cpp
)
target_include_directories(BRepInputTest PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OCC_INCLUDE_DIR})
target_link_directories(BRepInputTest PRIVATE
    "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>"
    "$<$<CONFIG:Release>:${OCC_LIB_DIR}>")
target_link_libraries(BRepInputTest PRIVATE
    Qt5::Core
    ${VTK_LIBRARIES}
    mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>"
    "$<$<CONFIG:Release>:${OCC_LIBS}>"
)
vtk_module_autoinit(TARGETS BRepInputTest MODULES ${VTK_LIBRARIES})



# ============================================================
# CTest 注册（任务 #12）
# ------------------------------------------------------------
# 目的：把全部自测纳入 `ctest`，一条命令跑完整回归，避免手工逐个执行时
# 漏测（此前 TetMeshEngineTest 甚至未注册，只能靠人记得手动带参数跑）。
#
# 用法（在 build 目录）：
#   ctest -C Release --output-on-failure
# 只跑单元测试（跳过较慢的端到端）：
#   ctest -C Release -L unit --output-on-failure
#
# 标签：
#   unit   —— 纯算法单元测试（无 Qt/VTK 依赖，快）
#   e2e    —— 端到端链路（需 Qt+VTK，读 test_data/*.stl）
# ============================================================
# 实际 Qt/VTK 窗口流程，显式运行；依赖可用的桌面 OpenGL，不混入默认无窗口回归。
set(MESHSTUDIO_GUI_TEST_SOURCES ${MESHSTUDIO_SOURCES})
list(REMOVE_ITEM MESHSTUDIO_GUI_TEST_SOURCES src/app/main.cpp)
add_executable(CadSurfaceGuiTest tests/CadSurfaceGuiTest.cpp ${MESHSTUDIO_GUI_TEST_SOURCES} resources/resources.qrc)
target_include_directories(CadSurfaceGuiTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${OCC_INCLUDE_DIR} ${QUAZIP_INC_DIR} ${ZLIB_INC_DIR})
target_compile_definitions(CadSurfaceGuiTest PRIVATE QUAZIP_STATIC)
target_link_directories(CadSurfaceGuiTest PRIVATE "$<$<CONFIG:Debug>:${OCC_LIB_DIR_DEBUG}>" "$<$<CONFIG:Release>:${OCC_LIB_DIR}>" ${QUAZIP_LIB_DIR} ${ZLIB_LIB_DIR})
target_link_libraries(CadSurfaceGuiTest PRIVATE Qt5::Widgets Qt5::Svg Qt5::Concurrent ${VTK_LIBRARIES} mesher
    "$<$<CONFIG:Debug>:${OCC_LIBS_D}>" "$<$<CONFIG:Release>:${OCC_LIBS}>"
    "$<$<CONFIG:Debug>:quazip1-qt5d.lib>" "$<$<CONFIG:Release>:quazip1-qt5.lib>"
    "$<$<CONFIG:Debug>:zsd.lib>" "$<$<CONFIG:Release>:zs.lib>" dwmapi)
vtk_module_autoinit(TARGETS CadSurfaceGuiTest MODULES ${VTK_LIBRARIES})
foreach(shape sphere sphere-tight sphere-reference sphere-reference-physical cylinder annulus)
    add_test(NAME OccCadSurfaceTest_${shape} COMMAND OccCadSurfaceTest ${shape})
    set_tests_properties(OccCadSurfaceTest_${shape} PROPERTIES LABELS e2e TIMEOUT 120)
endforeach()

# ---- 单元测试 ----
set(MESHSTUDIO_UNIT_TESTS
    DialogFormFactoryTest
    MeshSettingsDialogTest
    Delaunay2DTest
    Constrained2DRobustTest
    Delaunay3DTest
    Constrained3DTest
    GeomPredicates3DTest
    VolumeBoundaryValidationTest
    VolumeDomain3DTest
    TetQualityAcceptanceTest
    VolumePipelineTest
    Ruppert3DTest
    Flips3DTest
    Smooth3DTest
    SurfaceMesh3DTest
    CadSurfaceMesherTest
    CadSurfacePerformanceTest
    CadSurfaceQualityReportTest
    CadBenchmarkOptionsTest
    SurfaceEvaluationCacheTest
    Mesh2DTransactionTest
    Mesh2DEditEquivalenceTest
    SurfaceDomainStateTest
    SurfaceRefinementQueueTest
    CadBoundaryQueueTest
    ProjectArchiveTest
)
foreach(t ${MESHSTUDIO_UNIT_TESTS})
    add_test(NAME ${t} COMMAND ${t})
    set_tests_properties(${t} PROPERTIES LABELS unit TIMEOUT 600)
endforeach()
set_tests_properties(MeshSettingsDialogTest DialogFormFactoryTest PROPERTIES TIMEOUT 30
    ENVIRONMENT "QT_QPA_PLATFORM_PLUGIN_PATH=$<TARGET_FILE_DIR:Qt5::QOffscreenIntegrationPlugin>")

# ---- 端到端（TetMeshEngineTest，需输入文件参数）----
# 两种尺度的立方体验证实体体积、外边界覆盖及真实质量，不锁定剖分数量。
set(MESHSTUDIO_TEST_DATA ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
# 固定的 Gmsh 4.13.1 T3 输入；此测试链不编译/调用 SurfaceMeshEngine。
# regenerate: tests/GmshTetFixtures.py；默认回归不要求 Python/Gmsh 安装。
foreach(gmshCase cube concave cavity disconnected sphere)
    foreach(level h1 h0.5)
        set(gmshFixture ${MESHSTUDIO_TEST_DATA}/gmsh-tet/${gmshCase}-${level}/surface.gmstet)
        if(gmshCase STREQUAL "sphere" AND level STREQUAL "h1")
            set(gmshTestName GmshTetBoundaryRejection_sphere_h1)
            set(gmshTestCase sphere-unsupported)
        else()
            set(gmshTestName GmshTetPipelineTest_${gmshCase}_${level})
            set(gmshTestCase ${gmshCase})
        endif()
        add_test(NAME ${gmshTestName} COMMAND GmshTetPipelineTest ${gmshFixture} 3 ${gmshTestCase}
            ${CMAKE_CURRENT_BINARY_DIR}/${gmshTestName}.json)
        set_tests_properties(${gmshTestName} PROPERTIES LABELS "tet;gmsh-fixture" TIMEOUT 120)
    endforeach()
endforeach()
if(EXISTS ${MESHSTUDIO_TEST_DATA}/cube.stl)
    add_test(NAME TetMeshEngineTest_cube
             COMMAND TetMeshEngineTest ${MESHSTUDIO_TEST_DATA}/cube.stl)
    set_tests_properties(TetMeshEngineTest_cube PROPERTIES LABELS e2e TIMEOUT 600)
endif()
if(EXISTS ${MESHSTUDIO_TEST_DATA}/cube10.stl)
    add_test(NAME TetMeshEngineTest_cube10
             COMMAND TetMeshEngineTest ${MESHSTUDIO_TEST_DATA}/cube10.stl)
    set_tests_properties(TetMeshEngineTest_cube10 PROPERTIES LABELS e2e TIMEOUT 600)
endif()

# ---- B-Rep 输入链路（STEP/IGES/BRep → 离散 → 焊接 → 四面体）----
# 这两个文件是 B-Rep 格式，不携带三角面片，正好覆盖新增的离散化路径。
#   * cube2.stp  —— 轻量，跑**全链路**（含引擎）作端到端基线；
#   * ball.brep  —— 曲面 + 负坐标，验证「离散 + 焊接」。
#     不带 --with-engine：球面点集凸且近似共球，Delaunay 高度退化、
#     7692 个约束面的 CDT 是分钟级，不适合放进默认回归。
if(EXISTS ${MESHSTUDIO_TEST_DATA}/cube2.stp)
    add_test(NAME OccCadSurfaceTest_step
             COMMAND OccCadSurfaceTest ${MESHSTUDIO_TEST_DATA}/cube2.stp)
    set_tests_properties(OccCadSurfaceTest_step PROPERTIES LABELS e2e TIMEOUT 120)
    add_test(NAME BRepInputTest_step
             COMMAND BRepInputTest --with-engine ${MESHSTUDIO_TEST_DATA}/cube2.stp)
    set_tests_properties(BRepInputTest_step PROPERTIES LABELS e2e TIMEOUT 600)
endif()
if(EXISTS ${MESHSTUDIO_TEST_DATA}/ball.brep)
    add_test(NAME CadSurfaceProjectRoundTripTest_ball
             COMMAND CadSurfaceProjectRoundTripTest ${MESHSTUDIO_TEST_DATA}/ball.brep)
    set_tests_properties(CadSurfaceProjectRoundTripTest_ball PROPERTIES LABELS e2e TIMEOUT 120)
    add_test(NAME CadSurfaceQualityAuditTest_ball
             COMMAND CadSurfaceQualityAuditTest ${MESHSTUDIO_TEST_DATA}/ball.brep)
    set_tests_properties(CadSurfaceQualityAuditTest_ball PROPERTIES LABELS e2e TIMEOUT 120)
    add_test(NAME OccCadSurfaceTest_ball
             COMMAND OccCadSurfaceTest ${MESHSTUDIO_TEST_DATA}/ball.brep 20 40000)
    set_tests_properties(OccCadSurfaceTest_ball PROPERTIES LABELS e2e TIMEOUT 120)
    add_test(NAME CadGeometryAuditTest_ball_face9
             COMMAND CadGeometryAuditTest ${MESHSTUDIO_TEST_DATA}/ball.brep 40 1000 verify 9)
    set_tests_properties(CadGeometryAuditTest_ball_face9 PROPERTIES LABELS e2e TIMEOUT 30)
    add_test(NAME CadGeometryAuditTest_ball_faces15_16
             COMMAND CadGeometryAuditTest ${MESHSTUDIO_TEST_DATA}/ball.brep 40 5000 verify 15,16)
    set_tests_properties(CadGeometryAuditTest_ball_faces15_16 PROPERTIES LABELS e2e TIMEOUT 30)
    add_test(NAME CadGeometryAuditTest_ball_face10
             COMMAND CadGeometryAuditTest ${MESHSTUDIO_TEST_DATA}/ball.brep 30 1000 verify 10)
    set_tests_properties(CadGeometryAuditTest_ball_face10 PROPERTIES LABELS e2e TIMEOUT 30)
    add_test(NAME CadGeometryAuditTest_ball_face11
             COMMAND CadGeometryAuditTest ${MESHSTUDIO_TEST_DATA}/ball.brep 31 5000 verify 11)
    set_tests_properties(CadGeometryAuditTest_ball_face11 PROPERTIES LABELS e2e TIMEOUT 30)
    add_test(NAME BRepInputTest_brep
             COMMAND BRepInputTest ${MESHSTUDIO_TEST_DATA}/ball.brep)
    set_tests_properties(BRepInputTest_brep PROPERTIES LABELS e2e TIMEOUT 600)
endif()
