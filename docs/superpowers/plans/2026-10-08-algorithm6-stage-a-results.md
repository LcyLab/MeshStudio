# Algorithm 6 阶段 A 结果（2026-10-08）

独立普通面入口已实现。rectangle、hole、narrow_band 在 h=0.3、边界传播0/1、平滑1轮的六组运行中，仅凭 BREP+配置生成最终网格；六组均通过本阶段形状门槛和独立拓扑/标签检查。周期、真实模型、GUI和正式三轮速度验收尚未开展。

## 实现与输入来源

| 模块 | 已实现职责 | 输入来源 |
| --- | --- | --- |
| OccAlgorithm6Adapter | buildModelInput/buildFaceInput：indexed实体、边/面关联、pcurve、参数和精确几何回调 | 原始BREP、OCC、已有OccMeshVertexParameters |
| BoundaryRegistry | 全模型唯一边离散、需要时细分未恢复段 | 自研尺寸求值、CAD曲线Value/D1/直接投影 |
| SurfaceGenerator | generateFaceMesh/generateSurfaceMesh：普通图表→初始/恢复/分类/合法化→BDS→前沿→转移→平滑 | 同一次运行连续传递的自研网格与CAD回调 |
| MeshAssembly | 统一节点身份、CAD标签和每角UV，检查基础拓扑、方向及重复/退化片 | 自研FaceMesh、共享边、CAD朝向 |
| 基准/独立审计器 | 阶段VTP、最终VTP、质量、裁剪CAD距离及计时 | 最终网格与原始CAD；不参与生成决策 |

纯编排/装配编入mesher静态库；OCC适配器位于core层。新流水线测试程序只链接自研mesher和OCC；导出工具另链接VTK。直接依赖检查未发现Gmsh。Gmsh只由独立Python参考进程调用固定4.15.2发行DLL。

自研进程接收的独立input目录只有model.brep，命令行只有尺寸、传播开关和输出目录；PATH仅提供OCC/VTK/Windows运行库，PYTHONPATH为空。它不读取参考网格、轨迹、拓扑枚举、参数、尺寸或节点顺序。

原始0D/1D/2D存储参数与前沿运行UV分别保存。OCC实例长于全部回调，单次调用串行。原始边界节点按部件及物理身份复用，面内新节点按面及局部身份区分，不按坐标焊接。BREP内的INTERNAL约束保留；BREP未保存的外部嵌入关系不猜测。

## 最终质量与外观

近等边定义为三个角均位于50°–70°。各区域使用同一独立C++三维测量；边界区域为接触任一CAD边界节点的三角形，其余为内部。窄带没有内部片，报告明确标为空；全域仍包含全部片。

| 案例 | 传播 | 自研/参考片数 | 自研/参考近等边比例 | 自研/参考最小角P5 |
| --- | --- | --- | --- | --- |
| rectangle | 0 | 330 / 328 | 75.76% / 70.73% | 46.514° / 46.250° |
| rectangle | 1 | 346 / 344 | 75.72% / 76.16% | 45.514° / 45.575° |
| hole | 0 | 319 / 319 | 55.49% / 55.49% | 45.094° / 45.094° |
| hole | 1 | 333 / 333 | 62.76% / 62.76% | 44.390° / 44.390° |
| narrow_band | 0 | 56 / 56 | 50.00% / 50.00% | 34.992° / 34.992° |
| narrow_band | 1 | 56 / 56 | 50.00% / 50.00% | 34.992° / 34.992° |

六组全域/非空边界/非空内部均满足：近等边比例不低于参考5个百分点，最小角P5不低于参考2°，小于25°比例不高于参考1个百分点。没有删除受边界约束的片，也没有放宽门槛。

整体与匹配区域放大图已实际查看：带孔和窄带两侧布局相同；矩形存在小范围不同连接/布点，每组多2片，最终指标通过。本阶段没有对矩形做首差逐值定位，不宣称独立入口逐浮点/逐连接完全等价。

两侧整体网格，传播1（本地证据，未入库：`/C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-stage-a-20261008-r3/full-comparison-extend1.png`）

两侧匹配区域放大，传播1（本地证据，未入库：`/C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-stage-a-20261008-r3/local-comparison-extend1.png`）

独立拓扑检查包括：有限/非零面积片、重复片、非流形和方向冲突、声明CAD边缺失/意外开边、裁剪域/孔内三角形重心、面片相交。另检查CAD边标签对应的端点、CAD面向、每角UV与XYZ/支撑面一致。六组两侧全部通过。

## 几何误差与速度边界

两侧采样合同相同：裁剪域81×81 UV格点、每CAD边use 301点；每片顶点/边中点/重心共7点。距离由独立三角最近距离及OCC裁剪面距离测量，不复用生成器判据。

- 矩形和窄带两侧双向采样距离约机器舍入量，最大不超过6.3e-16模型单位。
- 带孔两侧CAD→mesh最大采样距离约6.3e-16；mesh→裁剪CAD最大均0.020253513192751697模型单位。圆孔弦段近似向孔内延伸，必须保留此值。
- 未给定用户几何容限，不宣称精度正式达标；有限采样不证明全局距离界。

最终r3单次生成计时（读取、导出、审计分别记录；生成包含阶段快照开销）：

| 案例 | 传播 | 自研 / Gmsh生成秒数 |
| --- | --- | --- |
| rectangle | 0 | 0.016809 / 0.006370 |
| rectangle | 1 | 0.017002 / 0.007073 |
| hole | 0 | 0.016209 / 0.004683 |
| hole | 1 | 0.015492 / 0.004842 |
| narrow_band | 0 | 0.008329 / 0.002360 |
| narrow_band | 1 | 0.005383 / 0.002324 |

单次自研约参考2.3–3.5倍，尚未达到拟定2倍速度门槛。尚未进行关闭详细观察的三轮计时或热点优化，不能将此表作为阶段D速度验收。

## 验证与复现

- 流水线stub先运行RED，失败于CAD-only pipeline not implemented；接完整主链后GREEN。
- 实际测试启动取消、转移后中途取消、累计预算在chart阶段耗尽、节点预算、注入CAD查询失败、seam和真实圆柱周期CAD拒绝、重复片装配失败的原子输出及标签身份。
- 已重建48个复刻测试及相关CAD/事务目标；实际63项CTest全部通过，24项Python语义测试通过。补标签断言后再次运行流水线测试通过。
- 共享边恢复细分/重建和默认最多10次重试已编排；现有边界恢复/细分叶测试通过，但六组完整入口均0次重试，未证明真实CAD触发重试的端到端路径。
- 当前不是GUI验收。未改变正式程序/生产默认，未创建分支/worktree或提交。

证据根目录：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-stage-a-20261008-r3/`。每案例目录含独立BREP、两侧最终VTP、参考配置/日志、两侧同口径审计；own子目录另含boundary/initial/pre_smoothing/final阶段VTP、generation.json和timing.json。实验VTP用line cell的单分量CadEdgeId表示CAD边，三角cell保留CadFaceId/CadBodyId/六分量CadCornerUV；纯核心输出仍是现有SurfaceMeshResult，实验VTP尚未做产品工程往返验证。总清单comparison.json记录CAD、程序、脚本及网格SHA，validation.json记录源码/静态库/测试程序SHA。63项CTest日志位于r2/ctest-final.log；r3/pipeline-real-periodic-final.log为新增标签断言及真实圆柱拒绝后的最终复核。

独立生成命令（输出目录必须尚不存在）：

```powershell
$env:PATH = 'F:/ProgramData/opencascade-7.6.0/install/win64/vc14/bin;F:/ProgramData/opencascade-7.6.0/products/VTK-9.3.1/release/bin;' + $env:PATH
& 'C:/Users/30861/AppData/Local/MeshStudioValidation/gmsh-reproduction-20261007/Release/GmshSurfaceReproductionBenchmark.exe' generate 'C:/Users/30861/AppData/Local/MeshStudioValidation/evidence-20261007-reproduction/rectangle.brep' 0.3 1 'C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-stage-a-manual-rectangle'
```

下一步停在阶段A结果展示；周期临时对象清理/曲面分支、整模型共享装配验收、GUI和阶段D性能属于后续阶段。
