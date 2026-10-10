# Algorithm 6：统一节点后的整模型装配检查

2026-10-09。由手机原 BREP 的点尺寸 `.0003` 反例触发，补阶段 C 的装配职责。原模型、尺寸规则、生成算法、审计门槛及生产默认保持。

## 问题与改动

此前 `MeshAssembly` 在各面局部节点上检查关联次数，却按物理身份合并节点后直接返回完整结果。手机两侧实际最终网格各有 32 条四片关联边；局部计数不能发现跨面共用内部边，也不能发现同一面不同局部节点合并后的异常。

现在在 CAD 面向调整后，以最终节点索引累计整模型边的关联次数及方向：超过两片关联或两片同向均返回 `InvalidTopology`。错误保存当前一基 CAD 面、局部三角编号、首次关联面和三角编号及最终节点索引。成功输出仍由同一原子赋值发布；失败保留调用者原结果，`SurfaceGenerator` 不发布完整模型，并把具体失败面标为 assembly 失败。

改动范围：`MeshAssembly.cpp/.h`、新增 `GmshMeshAssemblyTest.cpp` 和该测试的局部 CMake 注册。生成、投影、前沿和平滑模块没有修改。

## 实际验证

- 两个小型回归在原实现上实际失败：两片共享边同向仍返回成功；分别合法的两个面合并后四片共边仍返回成功。保留两份 RED 日志。
- 修正后检查合法跨面连接、四片共边、同一面物理身份别名合并、不同部件隔离，以及取消／预算失败时的输出原子性。
- 受影响的四个目标构建通过；最终 52 项 `gmsh_reproduction` 回归通过。
- 同一完整手机 BREP、同一 `.0003` 参数复跑：返回状态 5（`InvalidTopology`），失败面 970，失败阶段 assembly；没有导出 `own.vtp`。实际错误为：

```text
assembly: face=970 triangle=0: invalid assembled edge nodes=1286586,1286587 incidence=3 direction_sum=-1 first_face=969 first_triangle=0
```

这与先前失败网格第一条四片共边的节点、面及部件来源一致。第三片关联即拒绝，实际失败面报告为 `current=false/status=5/failed_stage=assembly`。这项验证确认不再把该坏连接发布为完整成功，不代表几何布局已经修复。

## 版本与证据

旧程序及核心保存到仓库外 `algorithm6-pre-global-assembly-check-20261009`：benchmark `cc5e3391…`、core `e23b283a…`。旧 30 配置／90 轮性能保持旧版本归属。

当前 benchmark SHA：`8ffe060247407513b339f3a93ab075d4b211f1046874edc30dc77d5e02e8a497`；core SHA：`78f7c8ca91216d7ae99424b94d14cdb0cc7d955c02206cedcd99779649a5e952`。

- RED：同向边（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-global-assembly-red-test-20261009.log`）
- RED：四片共边（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-global-assembly-red2-test-20261009.log`）
- 最终52项回归（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-global-assembly-final-reproduction-tests-20261009.log`）
- 实际手机失败（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-global-assembly-validation-r1-20261009/phone-refined-rejected/own/failure.txt`）
- 当前验证控制器（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-global-assembly-validation-r1-20261009/summary.json`）

当前验证依次运行：手机完整反例、原点尺寸及常量参数结果不变性、原 32 配置两侧资格复验，以及其中合格配置的三轮串行性能。只有相关生成／审计子进程结束后才进入正式计时；脚本保存实际子进程身份和终态，检查无竞争构建／网格任务。此报告不提前宣称未结束项目通过。

后续实际终态：32配置两侧资格结束，31完整审计/30形状通过，64最终VTP不变；新terminal-check --qualification-only从实际CAD、配置、固定SDK、审计门槛及哈希复算，退出0，保存到 `algorithm6-global-assembly-qualification-check-r1-20261009.json`。手机两原基线仍未通过质量。

原控制器误将comparison.json文件传给要求目录的--comparison，启动前FileNotFoundError、未创建性能试验；失败日志及终态保存到 `initial-controller-terminal-failure.json`。仅性能续跑6633/PID55508传实际目录，使用新的r2性能目录，串行测量30合格配置三轮。不改核心或重复资格；正式终态后独立逐轮核验。

新增只读原 CAD 面969边界诊断显示，共边端点是左右两条直线 seam 的中点，四个三角形共用的是面内直径弦，不是原 CAD 边。不能凭结果直接修改圆弧最低分段数或判定 CAD 损坏。诊断位于 `algorithm6-global-assembly-circle-source-r1-20261009/cad-face969.json`。

手机原 `.003` 的34条真实圆弧越界及4条分类矛盾、凹球 pcurve 不一致和球面形状差异仍保留；本改动不构成 B/C/D 整体验收。
