# Algorithm 6 球面独立修复

后续收尾已完成：[当前最终验收报告](2026-10-09-algorithm6-final-acceptance-results.md)。新程序34资格及31配置93轮性能全部通过，真实窗口/完整相交/自然边界重试已补齐。下文保留球面修复时的程序签名和当时证据边界，不再表示当前缺少全矩阵性能或窗口验收。

2026-10-09。用户授权继续修复球面对照差异，范围保持“达到固定Gmsh对照门槛”；不恢复双方共同缺陷优化、不改原CAD、生产默认或正式EXE，不创建分支/worktree，不提交。

## 修复与原因

官方Windows64 Gmsh 4.15.2的球面Value/D1使用x87成对sin/cos，本地OCC分别求值。约一个浮点舍入单位的差别在圆／曲面有限差分Newton中改变迭代分支，最终影响前沿布局。实际SDK二进制的调用及指令已保存；独立成对求值小程序对218个候选的5983次SDK球面Value查询全部逐值一致，证实数值来源。

`OccAlgorithm6Adapter`只为球面前沿Value/D1读取含Location的BREP支撑面位置和半径，调用纯mesher中的`NativeSinCos`；MSVC x64使用独立MASM，其他平台使用标准库，不声明与该Windows基线逐值相同。初始图表/度量、原平滑、通用OCC查询、Newton规则、尺寸、标签及独立审计不改。新生成入口不调用Gmsh，不接收参考网格或几何查询。

已按先失败后修复核验`OccAlgorithm6SphereEvaluationTest`：原版本明确报球面前沿求值舍入不符；修复后通过。回归还覆盖变换/反向球面、极点、失败候选的大UV、大参数约减、非有限输入和其他曲面的原查询路径。旧核心及生成程序已按原哈希归档。

## 原球面最终网格

同一BREP、h=.3、传播0/1、平滑1、一阶、串行、失败切换0；官方参考在独立进程重新生成。

| 配置 | 自研节点／片数 | 官方节点／片数 | 最大对应XYZ差 | 边界近等边（两侧） |
| --- | ---: | ---: | ---: | ---: |
| 传播0 | 179／354 | 179／354 | 3.6989e-15 | 36／49 |
| 传播1 | 200／396 | 200／396 | 4.6404e-15 | 37／50（74%） |

两种配置逐片核对全部有向连接、CAD面/部件标签及CAD边连接/标签一致；独立审计的开放边、重复片、非流形边、错误方向、域外片心、错误逐角UV、相交均为0。三种形状区域的原门槛全部通过，传播1原29／49（59.18%）的边界近等边差异已修复。实际查看了两种配置两侧整体和同区域局部图。

传播1网格→CAD最大距离自研0.024070385483025175、参考0.024070385483025172；CAD→网格自研0.024738301833925943、参考0.024738301833925946。单位沿用模型，无用户指定绝对几何容限，只报告实测及差值。1e-12只用于导出节点对应，不改变生成或审计容差。

## 相关验证

- 全Release构建成功，包含实验程序及工程往返目标；已有不相关弃用警告保留。
- 53项Algorithm6回归、2项CAD查询回归、4项工程往返通过。其中直接支撑面求值回归同时计入前53项；合计58项不同C++测试。追加大参数及非有限输入断言后的球面回归也单独重建并通过。
- 37项Python对照工具回归通过。
- 生成程序依赖检查没有Gmsh；自研进程PATH仅含OCC/VTK及系统目录，输入目录只有BREP，诊断DLL环境变量移除。
- 新核心SHA256 `efcbbd18ca1adae007f2076b34d8a21b123a899a541bce969ea33a3a1afd5d49`；新benchmark `60c7dbffa30d610b8c238ddf23a94fff339928ffc0ba2349039b8f80fe53a61c`。旧78f7／8ffe在仓库外`algorithm6-pre-sphere-sincos-20261009`保留。

新版本34配置已全部重新生成并完成独立复核，**34/34达原形状及无新增已报告拓扑的相对门槛**。26份最终自研VTP与修复前逐字节一致，包括手机常量/点尺寸；其余8份属于球面和含球面的凹球配置，全部达到对照门槛。手机常量66域外/首次相交、点尺寸38域外及凹球点传播1双方2域外仍保持绝对审计false，未被改成成功。

第一次矩阵控制器在最后手机点尺寸产物发布前结束，旧输出目录为空，未计算终态；33个已完成行逐文件哈希核验后，仅最后输入在新`own-resumed`目录续跑。最初续跑误假设空目录未创建而被断言拒绝，未启动生成；检查后保留空目录，续跑完成。最终报告独立保存为`summary-final.json`，旧中间报告保留，不把未结束记录当成功。

其他构建/生成/审计全部结束后，对新球面传播0/1各进行三轮串行性能测量，无阶段观察；6轮自研及参考VTP各自与新资格文件逐字节相同，质量全通过。生成耗时中位数比值（自研／Gmsh）分别1.256826、1.129675，均满足原≤2门槛。导入、生成、导出和审计分别记录，逐轮有效配置、原CAD/程序/输出哈希、质量及中位数已独立复算。旧版本90轮正式性能保持原版本绑定，不替代新版本全矩阵性能。

## 证据

- 独立修复逐片核对（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-sincos-repair-r1-20261009/independent-mesh-binding.json`）
- 5983次数值来源核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-sincos-query-proof-20261009.json`）
- 传播1自研最终VTP（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-sincos-repair-r1-20261009/sphere-extend1/own/own.vtp`）
- 传播1官方最终VTP（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-sincos-repair-r1-20261009/sphere-extend1/reference.vtp`）
- 传播1整体与局部图（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-sincos-repair-r1-20261009/sphere-extend1/whole-and-local-3d.png`）
- 两种传播质量报告（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-sincos-repair-r1-20261009/comparison.json`）
- 新34配置终态（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-repair-matrix-r1-20261009/summary-final.json`）
- 球面三轮性能（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-repair-performance-r1-20261009/performance.json`）
- 最终独立证据核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-sphere-repair-final-check-r1-20261009.json`）

本轮未重新进行人工GUI操作验收。源代码和仓库外实验程序已包含修复，生产默认仍为legacy。
