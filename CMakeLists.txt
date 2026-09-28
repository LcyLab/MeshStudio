cmake_minimum_required(VERSION 3.16)

project(MeshStudio VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# 本机依赖路径（可用 -DCMAKE_PREFIX_PATH=... 覆盖）
list(APPEND CMAKE_PREFIX_PATH
    "F:/Qt/Qt5.14.2/5.14.2/msvc2017_64"
    "F:/ProgramData/opencascade-7.6.0/products/VTK-9.3.1/release"
)

if(MSVC)
    add_compile_options(/utf-8 /W3)
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
    src/main.cpp
    src/MainWindow.h
    src/MainWindow.cpp
    src/ViewportWidget.h
    src/ViewportWidget.cpp
    src/core/ShapeLoader.h
    src/core/ShapeLoader.cpp
    src/core/ShapeToPolyData.h
    src/core/ShapeToPolyData.cpp
    src/core/ShapeTessellator.h
    src/core/ShapeTessellator.cpp
    src/core/TetMeshParams.h
    src/core/TetMeshEngine.h
    src/core/TetMeshEngine.cpp
    src/core/MeshProject.h
    src/core/ProjectPackage.h
    src/core/ProjectPackage.cpp
    src/core/ProjectArchive.h
    src/core/ProjectArchive.cpp
    src/core/WindowChrome.h
    src/core/WindowChrome.cpp
    src/panels/ModelTreePanel.h
    src/panels/ModelTreePanel.cpp
    src/panels/MeshSettingsPanel.h
    src/panels/MeshSettingsPanel.cpp
    src/panels/OutputDock.h
    src/panels/OutputDock.cpp
    src/panels/PropertiesPanel.h
    src/panels/PropertiesPanel.cpp
    src/dialogs/ModelTransformDialog.h
    src/dialogs/ModelTransformDialog.cpp
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

# ---------- 工程存储层自测（控制台，tests/ProjectArchiveTest.cpp） ----------
add_executable(ProjectArchiveTest tests/ProjectArchiveTest.cpp
    src/core/MeshProject.h
    src/core/TetMeshParams.h
    src/core/TetMeshEngine.h
    src/core/ProjectPackage.h
    src/core/ProjectPackage.cpp
    src/core/ProjectArchive.h
    src/core/ProjectArchive.cpp)

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

# ---------- 自研网格算法库（纯 C++17，不依赖 Qt/VTK/Gmsh；3D 阶段沿用） ----------
add_library(mesher STATIC
    src/core/mesher/MesherTypes.h
    src/core/mesher/MesherTypes.cpp
    src/core/mesher/GeomPredicates.h
    src/core/mesher/GeomPredicates.cpp
    src/core/mesher/Mesh2D.h
    src/core/mesher/Mesh2D.cpp
    src/core/mesher/Delaunay2D.h
    src/core/mesher/Delaunay2D.cpp
    src/core/mesher/Constrained2D.h
    src/core/mesher/Constrained2D.cpp
    src/core/mesher/Ruppert2D.h
    src/core/mesher/Ruppert2D.cpp
    src/core/mesher/TetMesh3D.h
    src/core/mesher/TetMesh3D.cpp
    src/core/mesher/Delaunay3D.h
    src/core/mesher/Delaunay3D.cpp
    src/core/mesher/Constrained3D.h
    src/core/mesher/Constrained3D.cpp
    src/core/mesher/Ruppert3D.h
    src/core/mesher/Ruppert3D.cpp
    src/core/mesher/Flips3D.h
    src/core/mesher/Flips3D.cpp
    src/core/mesher/Quality3D.h
    src/core/mesher/Quality3D.cpp
    src/core/mesher/Smooth3D.h
    src/core/mesher/Smooth3D.cpp
    src/core/mesher/SurfaceMesh3D.h
    src/core/mesher/SurfaceMesh3D.cpp)
target_include_directories(mesher PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 主程序使用自研引擎（取代 Gmsh）
target_link_libraries(MeshStudio PRIVATE mesher)

# ---------- 2D Delaunay 原型自测（控制台，tests/Delaunay2DTest.cpp） ----------
add_executable(Delaunay2DTest tests/Delaunay2DTest.cpp)
target_link_libraries(Delaunay2DTest PRIVATE mesher)

# ---------- 3D Delaunay 四面体化自测（控制台，tests/Delaunay3DTest.cpp） ----------
add_executable(Delaunay3DTest tests/Delaunay3DTest.cpp)
target_link_libraries(Delaunay3DTest PRIVATE mesher)

# ---------- 3D 表面约束恢复自测（控制台，tests/Constrained3DTest.cpp） ----------
add_executable(Constrained3DTest tests/Constrained3DTest.cpp)
target_link_libraries(Constrained3DTest PRIVATE mesher)

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

# ---------- 自研引擎端到端自测（控制台，需 Qt+VTK，tests/TetMeshEngineTest.cpp） ----------
add_executable(TetMeshEngineTest
    tests/TetMeshEngineTest.cpp
    src/core/TetMeshEngine.cpp
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
    src/core/TetMeshEngine.cpp
    src/core/ShapeLoader.cpp
    src/core/ShapeToPolyData.cpp
    src/core/ShapeTessellator.cpp
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
enable_testing()

# ---- 单元测试（8 个）----
set(MESHSTUDIO_UNIT_TESTS
    Delaunay2DTest
    Delaunay3DTest
    Constrained3DTest
    Ruppert3DTest
    Flips3DTest
    Smooth3DTest
    SurfaceMesh3DTest
    ProjectArchiveTest
)
foreach(t ${MESHSTUDIO_UNIT_TESTS})
    add_test(NAME ${t} COMMAND ${t})
    set_tests_properties(${t} PROPERTIES LABELS unit TIMEOUT 600)
endforeach()

# ---- 端到端（TetMeshEngineTest，需输入文件参数）----
# cube.stl：最简单的回归基线（10 节点 / 11 体单元 / minSICN=0.577350）
# cube10.stl：带退化的压力用例（103 节点 / 405 体单元 / 含 sliver）
set(MESHSTUDIO_TEST_DATA ${CMAKE_CURRENT_SOURCE_DIR}/test_data)
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
    add_test(NAME BRepInputTest_step
             COMMAND BRepInputTest --with-engine ${MESHSTUDIO_TEST_DATA}/cube2.stp)
    set_tests_properties(BRepInputTest_step PROPERTIES LABELS e2e TIMEOUT 600)
endif()
if(EXISTS ${MESHSTUDIO_TEST_DATA}/ball.brep)
    add_test(NAME BRepInputTest_brep
             COMMAND BRepInputTest ${MESHSTUDIO_TEST_DATA}/ball.brep)
    set_tests_properties(BRepInputTest_brep PROPERTIES LABELS e2e TIMEOUT 600)
endif()
