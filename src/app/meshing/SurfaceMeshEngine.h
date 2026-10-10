#pragma once
// 文件职责：应用层表面生成入口：读取 CAD、选择生成路线，再将网格与 CAD 标签转换为 VTK 数据。

#include "mesh/surface/legacy/CadSurfaceMesher.h"
#include "mesh/surface/legacy/SurfaceMeshingPartition.h"
#include "mesh/surface/algorithm6/pipeline/SurfaceGenerator.h"

#include <QString>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

// detail 保留算法数据和诊断，surface 携带同一结果的 VTK 显示/保存数据。
// ok 只表示生成和转换成功；Algorithm 6 的质量与 CAD 距离另行审计。
struct SurfaceGenerateResult {
    bool ok = false;
    QString error;
    mesher::SurfaceMeshResult detail;
    vtkSmartPointer<vtkPolyData> surface;
    // 实验入口的全部CAD面状态；失败也保留，未尝试/失效面不算成功。
    std::vector<mesher::gmshReference::Algorithm6FaceSummary> algorithm6Faces;
};

namespace SurfaceMeshEngine {
/**
 * @brief 从 CAD 文件生成表面网格，并保留 CAD 面、边、部件和逐角 UV。
 * @param filePath STEP、IGES 或 BRep 文件；本函数自行读取并持有 CAD 对象。
 * @param criteria 生成参数；长度沿用导入后的模型坐标单位，角度字段采用度。
 * @param progress 运行回调，返回 false 请求取消；Algorithm 6 用作心跳，无已知总工作量。
 * @param partitionPolicy 默认路线的分区设置；Algorithm 6 要求使用原 CAD 面。
 * @return 失败时 ok=false 并给出 error；成功结果也不等于体边界或独立质量审计合格。
 * 一次调用内部串行执行，可由后台任务调用；本函数不操作窗口或渲染对象。
 * 独立部件之间的接触/干涉作为关系记录保留，不能据此认定为可直接体网格化的公共界面。
 */
SurfaceGenerateResult run(const QString& filePath,const mesher::SurfaceCriteria& criteria,
                          mesher::MeshingProgress progress={},
                          const mesher::SurfacePartitionPolicy& partitionPolicy={});
}
