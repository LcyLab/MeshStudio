# Algorithm 6 剩余验收补齐

2026-10-09。范围为固定Gmsh 4.15.2、串行一阶Algorithm 6、同一原CAD和配置的相对验收；保持原门槛，不处理双方共有缺陷、不修改生产默认、不提交。

**当前状态：本次授权范围内的剩余相对验收任务已全部补齐。34组相对质量通过；31个绝对审计合格配置的93轮正式性能全部质量/速度通过；全量相交、真实边界重试和当前版本真实窗口验证均完成，终态逐文件独立核验通过。** 三个共有绝对失败配置保留，不宣称任意CAD、无缺陷网格或未指定绝对几何容限已达标。

## 当前程序与资格

新增`Algorithm6AcceptanceProbe`及局部CMake目标仅供显式诊断，没有修改生成核心或原相交判据。新增目标导致MASM重新封装并重新链接，因此对当前程序重新运行全部34个原配置，不能用旧程序签名代替。

当前benchmark SHA256 `a9a323dcff11a64002648f3fe933e777182e42b71ac4f78b7eb4534a3cad28f2`，core `05a2df361592d0aafeba5feb339a19a673a33f4c3e438f0f71ab2d91ad60ba65`。34组形状及无新增已报告拓扑的相对门槛通过；31组绝对审计通过，手机两组和凹球点尺寸传播1共有绝对失败保持。资格完整终态见下方。

## 全部相交集合

诊断复用`validateSurfaceIntersections`：每次真实命中后记录并继续遍历，沿用原CAD部件接触政策。扫描完成标志与无相交标志分开，不把回调继续返回true误称网格无相交。两组分离的错误相交及无相交回归通过，原首失败检查仍拒绝错误网格。

| 手机配置 | 自研错误相交对 | 官方错误相交对 | 完整对照 |
| --- | ---: | ---: | --- |
| 常量 | 11 | 11 | 全部11对物理三角形及CAD面/部件标签匹配，没有额外错误对 |
| 点尺寸0.003 | 0 | 0 | 两份完整扫描均无错误对 |

合法不同部件接触另行记录。其数量可因局部剖分不同而改变，不把这些接触算成错误相交。旧审计的首相交及`complete=false`保留，不修改为成功。

## 自然边界细分重试

OCC新建并导出有效的偏心窄环BREP，再从该BREP独立建立输入；没有注入恢复失败或喂入参考中间状态。粗尺寸h=4下边界节点14→17，触发一次细分、重建该面并生成33片。全部33个物理三角形和CAD标签与独立官方结果对应。两侧均有1个裁剪域外片心，绝对审计失败如实保留。

同一真实重试案例在细分后的重访中取消：明确返回取消，重试统计为1，不发布任何完整网格。此项补齐完整入口路径，不能据此声称任意CAD重试均收敛。

## 当前版本真实窗口

球面和原手机常量两例均用Windows Qt/OpenGL真实窗口完成：显式选择Algorithm 6、取消、生成、显示、VTP导出、工程保存、新窗口重开。球面200点396片，手机29980点59612片；生成与重开截图已实际查看。球面GUI导出另经独立最终CAD审计通过。手机窗口成功不替代其独立绝对审计失败。

全Release构建成功；59项不同C++回归及37项Python工具回归通过，另有真实BREP重试/取消显式诊断通过。生产默认仍为legacy，正式程序未替换。

## 正式性能

全部构建、诊断、GUI及资格生成结束后，只对31个绝对审计合格配置无阶段观察串行三轮。导入、生成、导出、审计分开计时，保持原生成中位耗时≤Gmsh两倍的门槛。**31/31质量和速度均通过，合计93轮；最慢中位比为1.636623。**

终态独立核验从原文件重算每轮全域/边界/内部形状门槛和三轮中位耗时，并检查实际计时、有效配置、固定SDK版本/算法及关闭切换、CAD/程序/网格hash。93轮两侧VTP各自均与当前资格VTP逐字节一致；当前资格34份自研VTP又与球面修复后基线逐字节一致。当前程序依赖检查无Gmsh。三组共有绝对失败配置明确排除性能资格，没有借31组结果声称手机速度通过。

首次启动在计时前因三个MSBuild缓存节点被检查拒绝，空目录保留。两次CPU采样确认其为无增长的nodeReuse缓存，无活动编译/链接；正式r2保存两次进程清单并只允许这种空闲缓存。旧版本2.573倍首次失败仍保留；新计时不删除历史。

## 证据

- 当前34组完整资格（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-final-qualification-r1-20261009/summary.json`）
- 手机完整相交集合独立核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-all-intersections-r1-20261009/final-all-pair-binding.json`）
- 自然BREP重试及取消（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-natural-boundary-retry-r1-20261009/independent-retry-binding.json`）
- 当前真实窗口证据（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-final-gui-evidence-20261009.json`）
- 球面生成窗口（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/MeshStudio-gui-JZXOSI/generated-window.png`）
- 手机重开窗口（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/MeshStudio-gui-qZnFaj/restored-window.png`）
- 正式性能目录（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-final-performance-r2-20261009/summary.json`）
- 最终逐轮独立证据核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-final-acceptance-check-r1-20261009.json`）

无用户指定绝对CAD距离容限，继续报告双向实测距离，不宣称绝对精度达标。验收范围仅为上述模型/配置，不代表任意CAD或跨平台逐浮点数一致。
