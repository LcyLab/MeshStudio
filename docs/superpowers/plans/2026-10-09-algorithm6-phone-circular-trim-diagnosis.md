# 手机点尺寸：圆弧窄域越界定位

2026-10-09。沿阶段 C 的真实模型失败继续核查；小布局差异暂放下。没有修改生成算法、原 CAD、审计门槛或生产默认。

## 最新：加密后仍有真实圆弧残差

从保留的 `.0003` 完整试验网格逐条提取原面289/309，两侧均保留原节点/单元编号、CAD面/部件/边标签及逐角UV。源网格、原圆域证据和两侧32条非流形连接的终态哈希重新绑定。两侧局部均为面289共146片、其中4个真实域外片心；面309共82片、其中6个真实域外片心。原 `.003` 的20/14个减少为4/6个，但没有消除。

| 原面（一基） | `.003`每侧真实域外 | `.0003`每侧真实域外 | 新边界最大弦中点偏离 | 自研片心最大进入量 |
| --- | ---: | ---: | ---: | ---: |
| 289 | 20 | 4 | `1.9885849230e-6` | `8.8637805355e-7` |
| 309 | 14 | 6 | `2.0213838506e-6` | `9.2608378960e-7` |

实际新最大弦长约 `.000295166/.000298202`，已接近试验点尺寸。剩余越界位于两圆接近交点的狭窄区；新增局部图实际查看，红色原三角片仍进入蓝色应排除圆盘。此结果不支持继续仅凭统一点尺寸加密就宣称完整模型通过。两侧完整网格仍各有32条四片关联边；新装配入口现在明确拒绝该坏连接，不发布最终网格。

首轮提取工具假定CellData先于Polys，在参考VTP的合法另一种数组顺序上失败。原r1局部和失败脚本保留；小型顺序回归实际RED后修正解析顺序，GREEN确认两种合法排列的坐标、UV、标签及连接完全一致。r2两侧提取和独立局部核验均实际退出0；图中UV轴旋转到局部切向/法向、横纵比例不同，明确标注，不用视觉比例判断物理形状质量。

- 两侧原圆弧局部及全部残差（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-phone-refined-circular-patches-r2-20261009/summary.json`）
- 原单元身份、有效配置和图像独立核对（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-phone-refined-circular-patches-r2-20261009/local-evidence-check.json`）
- 实际局部图（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-phone-refined-circular-patches-r2-20261009/actual-circular-trim-local.png`）
- 解析顺序RED日志（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-circular-patch-parser-order-red-20261009.log`）
- 解析顺序GREEN日志（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-circular-patch-parser-order-green-20261009.log`）

当前所有相关进程已终态。下一项先在已有CAD点尺寸能力内核验局部边界配置，避免整模型盲目加密；额外质量优化、原CAD修复或算法切换仍不能自动加入。下述启动/等待状态仅为历史，B/C/D未完成。

## 本次核验结果

当前点尺寸 `.003`、最大尺寸 `10.65`、边界传播 1、平滑 1 的原网格，两侧完整审计各有 38 条域外片心记录。全部 76 条记录从原最终 VTP 按原三角编号、节点、CAD 标签及逐角 UV 提取，重新运行原 CAD 查询；不是重新生成的小面替代原模型。

| 原 CAD 面号（一基） | 每侧记录 | 原边界核查 | 当前判断 |
| --- | ---: | --- | --- |
| 289 | 20 | 五段原圆弧构成两个解析圆，片心进入应排除圆盘；进入量 `2.4713146254e-7` 至 `1.2520734790e-4` | 真实越界 |
| 309 | 14 | 三段原圆弧构成两个解析圆，片心进入应排除圆盘；进入量 `1.3579853819e-7` 至 `1.7085373306e-5` | 真实越界 |
| 121 | 1 | 默认分类 OUT、FClass2d IN；解析审计门控拒绝 | 分类矛盾保留 |
| 610 | 3 | 默认分类 OUT、FClass2d IN；解析审计门控拒绝 | 分类矛盾保留 |

原生诊断明确记录二维及三维边类型为圆。使用三个原 pcurve 求值点重建对应解析圆，并以每条边全部 101 个原求值点复核，最大径向残差小于 `1e-15`；没有把圆弧折线化后进行域判断。两个圆盘的保留侧由原裁剪弧确定，不按半径大小猜测孔洞。这 34 条越界量与原距离查询相符，且分别大于相关原 CAD 顶点／曲线表示不一致量的 200 倍。本次证据限定这两面和这 34 条记录，不把另外四条分类矛盾直接改判为通过。

自研这 34 个失败三角形的物理坐标和连接，在保存的 `initial`、`pre_smoothing`、`final` 阶段均逐值存在。两面分别为 44、41 片，三个阶段片数保持。因此这些失败最晚在初始网格时已出现，不能归因为后续前沿或一次平滑产生的新三角形；还没有证明初始化内的唯一错误决策。

进一步按原 VTP 的 CAD 边标签提取实际约束线段：面 289 排除圆边界有 26 段、最大弦长 `.0027024273204`；面 309 有 25 段、最大弦长 `.0012498973115`。实际弦中点进入排除圆的最大量分别为 `.0001692701761`、`.00003562087903`，均超过两圆窄域可达到的最大宽度 `.00007599044843`、`.00003143924810`。两侧数值一致；原角 UV 的同节点差及端点到原圆误差单独核对。此证据支持现有边界尺寸／弦误差原因，不需要先改平滑，也不能凭收紧点尺寸就保证完整模型通过。

## 图与来源

已生成并实际查看两侧整面及局部图。局部图可直接看到直线三角片进入蓝色原 CAD 圆弧排除区；局部横纵比例不同，图中已明确标注。

- 逐条原网格身份及分类查询（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-outside-classification-r1-20261009/summary.json`）
- 解析圆域及原阶段网格核验（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-outside-classification-r1-20261009/arc-domain-evidence.json`）
- 原 CAD 标签边界弦误差与窄域宽度（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-outside-classification-r1-20261009/boundary-chord-evidence.json`）
- 终态来源与另一组圆求值点的70位十进制复核（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-outside-classification-r1-20261009/terminal-evidence-check.json`）
- 整面图（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-outside-classification-r1-20261009/original-circular-trim-and-failed-triangles.png`）
- 局部图（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-outside-classification-r1-20261009/original-circular-trim-local.png`）

自研最终 VTP SHA 为 `9310f7765bfc071f861dc2a7206cc7abf986022d22e066bed8521facca59a8a2`，参考为 `0f4fa583e39a25b76ba2c80569ee32de7ef35f56c49edf65e797affbb883aae4`；原 BREP SHA 为 `f204e4f48c3c9da9e6a4ecab21a0e8e8b2768c0999c035be076a64ec62293a07`。来源摘要绑定原审计、程序、核心和各查询文件实际哈希。核验脚本终态退出 0；本轮没有 C++ 修改，不重复构建。

## 下一验证

启动同一完整 BREP 的单变量诊断：只将 CAD 点尺寸 `.003` 收紧为 `.0003`，最大尺寸、传播、平滑、曲率关闭、算法 6 及失败切换 0 保持。自研和固定 SDK4.15.2 各自独立读取 BREP，原失败配置仍保留，不以新参数替换原资格结果。这一试验检查现有尺寸控制对原圆弧弦误差的影响，不增加细化补丁或其他算法。

控制器：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-refinement-controller-r1-20261009/state.json`。结果目录：`algorithm6-bcd-phone-point-refinement-probe-r1-20261009`。启动和运行进程已实际核对；本段不提前宣称新网格通过。原球传播 1、手机整体质量及凹球原 CAD 不一致仍未全部验收，B/C/D 保持进行中。

后续实际检查：自研生成状态 0、完整，耗时 `183.4827337` 秒；新增 `1290624` 点、几何查询 `275637873`，没有边界恢复重试。最终 VTP 头明确为 `1355797` 点、`2711246` 三角形；此时进程仍在独立审计，固定参考尚未执行。点尺寸收紧显著增加密度及耗时，不能在质量与参考终态未取得时推荐为默认参数或声明性能通过。另一组原圆采样点及 70 位十进制复算已独立确认 68 个进入量（两侧共计），所有来源、当前程序、阶段及两幅图重新核验，终态退出 0。

### 该试验发现的新失败

后续自研审计实际终止并退出失败：`nonmanifold or inconsistent triangle orientation`；没有写出完整质量报告。读取其原最终 2711246 片重新按节点身份统计，确定为 **32 条非流形边，每条四个三角形关联，方向冲突二关联边为 0**。诊断保存所有关联片的 CAD 面、部件、节点、方向及 XYZ，而不是仅保留笼统错误字符串。实际非流形连接（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-refinement-probe-r1-20261009/phone-refined-extend1/own-topology-diagnostic.json`）。首条边在原面969/970、部件68（一基），两面各两片共用同一组装配节点；部分其他失败边的四片属于同一 CAD 面。

源码核对：`MeshAssembly.cpp` 的 incidence 使用每面局部节点身份，完成后直接设置 `result.ok=true`，没有按最终统一节点索引检查整模型边关联次数。这是可独立修复的装配检查缺口；不据此认定最早三角生成决策或原 CAD 损坏。下一步先补最小跨面／等价节点合并回归，再增加整模型关联及方向检查，保持失败原子性；几何／边界布局修复另需来源证据。

固定 SDK 同参数原模型已完成生成 `2715568` 片、原生成时间 `82.4238995` 秒，仍由PID50732做独立审计。此单变量试验不是正式三轮性能，新参数仍未获资格。旧自研审计PID50932已经退出，不重启；只观察现有控制器44637及实际参考审计进程。

### 最终终态覆盖上述等待状态

控制器44637及其所有生成／审计子进程均已退出，最终退出1。固定参考同样审计失败，并独立统计为32条四片关联非流形边、二关联方向冲突0。两侧这32条边的部件和关联CAD面逐一对应，物理端点坐标 **完全相同（最大差0）**。`terminal-evidence-binding.json` 重新核对实际配置、固定版本、原BREP/程序/VTP哈希、原失败日志和全部对应连接，实际退出0；不是仅凭相同笼统错误判断根因。新参数两侧终态绑定（本地证据，未入库：`C:/Users/30861/AppData/Local/MeshStudioValidation/algorithm6-bcd-phone-point-refinement-probe-r1-20261009/phone-refined-extend1/terminal-evidence-binding.json`）。

收紧点尺寸不能单独取得整模型资格；本轮也没有加入额外优化。当前下一项是已由真实失败触发的装配整模型连接检查及失败原子性回归，而不是继续盲目加密。原.003的34条真实圆弧越界和4条分类矛盾仍保留；此时无本轮诊断任务在运行，B/C/D未完成。
