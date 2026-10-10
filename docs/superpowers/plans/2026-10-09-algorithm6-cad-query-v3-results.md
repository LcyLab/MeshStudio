# Algorithm 6 独立 CAD 查询 v3 复验

2026-10-09。B/C/D仍在实施；本文记录当前查询修正及已终态证据，不把局部修复当作手机整体验收。

## 具体问题与改动范围

原手机点尺寸结果两侧网格→CAD最大距离均为0.026683711101516232，来自同一原CAD面472的同一内部边中点。自研原完整网格片28171、参考片28205（片号从0开始）的XYZ、每角UV和标签均从实际VTP提取。该点到原平面支撑面的残差为8.6736×10⁻¹⁹；原默认分类器在审计UV容差1e-10和原面容差1e-7下均判OUT，而包围盒分类与FClass2d均判IN。原ExtPF据此丢弃距离0的支撑投影，返回到上方CAD边的距离。

原面有12条直线和2条圆弧。独立构造14边凹口面复现默认OUT及距离误报，未读取原手机或参考数据。修正仅在测试审计层：原共享CAD顶点闭合且原三维/二维直线或圆参数、完整圆系数在舍入量级一致的平面，使用解析域分类和原曲线物理距离。其他几何保留OCC路径；不修改CAD、生成核心、采样合同、容差或验收门槛。

初版额外格点检查发现圆弧最低端点计数错误，v3-r1任务已明确中断并保留不完整证据。修复后，圆孔、反向圆弧、跨2π参数、平移旋转、弧端点和切线、亚容差真实外点距离以及不一致pcurve回退均纳入最终回归。

## 已终态验证

当前benchmark SHA为`cc5e33916b820a6568912cafa1c8549614d7e718316cb0e6552f25162f432ba5`；生成核心`mesher.lib`仍为`e23b283a11c6ba5abc3b3b3d8e84cd1c4dafc076372f8d9306d06f2dd04db1d9`。

- 三个受影响目标成功构建；最终51项`gmsh_reproduction`回归通过。
- 两份原手机问题片摘录逐项核对原完整VTP的XYZ、所有CAD标签及每角UV，原CAD、原完整VTP和摘录均未改；最终查询七个样本最大距离均为3.7746098409546433e-19。该结论只覆盖这两片。
- 32组独立最终生成复验已终态：31组拓扑通过，30组形状通过；64份两侧最终VTP与查询v2版本逐字节相同。
- 独立核验从原审计重算全域/边界/内部形状门槛，核对实际两侧配置、同一BREP、固定SDK4.15.2、串行、一阶、算法6、平滑1、失败切换0及程序/网格哈希。未通过的球传播1形状与凹球点尺寸传播1域外仍保留。
- 原32对整体／局部图和分布报告已与当前64份VTP和查询v3审计重新绑定，片数、P5和最大边核对一致。图只证明相同网格布局及形状，不替代新版距离和拓扑报告。

证据目录均在仓库外：

| 证据 | 路径 |
| --- | --- |
| 两份实际问题片及最终查询绑定 | analytic-final-evidence-summary.json（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-distance-peaks-r1-20261009/analytic-final-evidence-summary.json`） |
| 原CAD局部图 | original-cad-phone-distance-peak.png（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-distance-peaks-r1-20261009/original-cad-phone-distance-peak.png`） |
| 32组终态索引 | summary.json（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-cad-query-qualification-v3-r2-20261009/summary.json`） |
| 32组实际配置／审计／哈希核验 | matrix-check.json（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-query-v3-matrix-check-r1-20261009.json`） |
| 32对图与64侧分布绑定 | distribution-binding.json（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-query-v3-distribution-binding-r1-20261009.json`） |
| 最终回归日志 | reproduction-tests.log（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-analytic-query-v3-final-reproduction-tests-20261009.log`） |

## 未完成的验证与已保留问题

四份原手机网格的全模型查询v3审计已结束，原完整网格及形状统计不变；独立终态核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-query-v3-qualification-check-r1-20261009.json`）已逐份复查实际审计、哈希及计数。局部修复仍未让手机全域通过。

常量尺寸两侧均66个域外片、0错误角UV、首个相交1；双向CAD最大仍约0.001120827319。点尺寸两侧均38个域外片（v2为43）、0错误角UV、相交0；网格→CAD最大约0.003278297105（v2为0.026683711102），CAD→网格最大约0.000283088261。每份CAD→网格采样10692163点，比v2增加24个被修正分类收入的真实域内点。距离下降不能替代质量门槛，四份审计均保持失败。

球传播1新增平滑变化位置证据（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-sphere-smoothing-transitions-r1-20261009.json`）：用CAD边界坐标锚定邻接图，全部物理节点与三角身份对应验证后，自研1个边界片进入、3个退出近等边范围；退出片111/237/274都位于负Y、负Z缝邻区域。参考平滑0诊断到平滑1基准为3个进入、0个退出。两侧前沿网格不同，尚不能归咎于平滑；未换顺序、关闭平滑或追加优化。

最终版本三轮性能控制器在上述特定上游退出、全终态独立核验通过且无竞争任务后完成30配置90轮正式测量；独立逐轮复核通过，质量与速度30/30，最大生成中位比值1.799324653。见[当前v3性能报告](2026-10-09-algorithm6-query-v3-performance.md)。旧查询v2/a186的90轮已归档，不替代当前证据。

球面传播1形状未达标；凹球点尺寸传播1的原CAD三维边与pcurve不一致，需要用户选择是否在外部新CAD副本修复；手机常量尺寸相交和曲率尺寸退化仍保留独立证据。原CAD、生产默认legacy、正式EXE及用户工作区改动保持。矩形小差异按用户要求暂不追。

## 剩余距离的位置和解释边界

正式性能结束后，只读重放点尺寸两侧完整网格的446684/446530个样本，精确复现上述最大0.003278297105到原面610（零基609）、自研片33236／参考片33270。同一位置的默认2D/3D分类OUT，包围盒及FClass2d均IN；原平面支撑残差约5.28e-18。两侧原完整VTP和CAD哈希再次保持；证据distance-peaks-v3/summary.json（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-distance-peaks-v3-r1-20261009/summary.json`）。

原面610有33直线、3圆弧；101点曲线/pcurve配对最大差4.69e-17。但新只读端点诊断发现原CAD边1525的共享顶点到原曲线／pcurve端点距离为7.6553201278e-11，超出舍入一致性门控，解释该面没有走解析查询。没有放宽门控、修复CAD或把默认距离自动归零。这个最大值仍是当前回退查询报告值，分类路径存在争议，尚不能当作已确认的真实几何偏差。

原面611的次大值也存在同类分类争议；8条直线的曲线/pcurve配对最大差7.76e-18，但原边1544的共享顶点与端点相差7.6552701678e-11。两面均通过原BRepCheck且SameParameter/SameRange为true，完整圆系数检查通过；这些标志不能替代更严格的共享端点检查。剩余查询来源核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-distance-peaks-v3-r1-20261009/remaining-query-evidence-summary.json`）已绑定实际两侧最大位置、原CAD及网格哈希、旧查询诊断程序和新门控诊断程序，以及具体被拒绝的两条边。原面121的101点曲线/pcurve配对差达4.33e-10，另行保留，未套用解析修正。

新增原CAD边类型、顶点和完整圆系数诊断仅改`GmshSurfaceInitializationProfile`，目标构建通过；相同运行库环境的流水线1/1通过，benchmark/core哈希cc5e/e23不变。首次附加CTest因漏设运行库PATH以0xc0000135启动失败，补齐环境后重跑通过，原失败没有计入算法回归。原profile535e程序已归档，当前诊断程序a01890d9。

新版手机点尺寸整体／局部图（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-query-v3-final-plots-r1-20261009/point-whole-and-local.png`）已生成并实际查看，图内按两份当前审计标记位置，明确显示两侧审计失败。对应来源绑定（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-query-v3-final-plots-r1-20261009/evidence.json`）记录两侧VTP、审计及图片哈希；视图相似不能使数值门槛通过。
