#ifndef LOCAL_CUT_RESULT_H
#define LOCAL_CUT_RESULT_H

#include <map>
#include <vector>
#include <utility>
#include "cmesh.h"
#include "JasMeshLocalMarkAndCutSplitInternal.h"

namespace MeshCutByMark
{

// 拼接边上的单个切点。
// 迁移第 1 步先用 double 坐标收集，后续统一换成精确核坐标比较。
struct SeamCutPoint
{
    int localVertexIndex = -1;   // 局部顶点下标
    int globalVertexIndex = -1;  // 全局顶点下标（合并后有效）
    double t = 0.0;              // 沿拼接边 a->b 的投影参数，用于排序
    vcg::Point3d point;          // 切点坐标（double，用于调试/打印）
    jaslmc::ExactPoint exactPoint; // 切点精确坐标，用于缝合阶段去重（不得用 double 比较）
};

// 一条拼接边的切点集合，按沿边方向有序。
struct SeamCutLine
{
    int globalVertexA = -1;      // 拼接边端点（规范化，小下标在前）
    int globalVertexB = -1;
    // 拼接边对面的全部全局邻接面（非流形缝边可能有多个，缝合阶段都细分）。
    std::vector<int> externalFaceIndices;
    std::vector<SeamCutPoint> points;
};

// 单个局部单元（flood-fill 得到的 curFaces）的切割结果。
// 局部阶段直接以 ExactMesh 为载体，串行合并阶段写回全局并映射缝边。
struct LocalCutResult
{
    std::vector<int> faceGlobals;      // 该局部单元涉及的全局面下标
    int targetMark = 0;
    jaslmc::ExactCutResult exact;      // 切好且分区好的局部 ExactMesh 与映射
    std::vector<SeamCutLine> seams;    // 合并阶段映射后的全局拼接边切点表

    // 多边形切割路径（prepareLocalCutPolygon）标记：合并阶段改走
    // mergeLocalCutPolygon，不写回/不切分全局面，也不产生 seams。
    bool usePolygonPath = false;
    // 多边形路径合并阶段追加的切点孤立顶点（全局下标 + 精确坐标）。
    // 切点可能落在与邻域共享的 mark-diff 边上，但邻域不沿同一条线切割，
    // 其边界环不会细分该边；主流程据此在 Phase 3 做跨区域切点 splice，
    // 保证邻接区域输出环共享同一顶点细分序列（消除 T 形结）。
    std::vector<std::pair<int, jaslmc::ExactPoint>> orphanCutPoints;
    // 多边形路径合并阶段输出：newMark -> 边界环（外圈在前，其余为洞），
    // 环内为全局顶点下标（含追加的切点孤立顶点）。空表示该 newMark
    // 需回退 SubRegionBoundary 提取。
    std::map<int, std::vector<std::vector<int>>> polyLoops;
};

} // namespace MeshCutByMark

#endif // LOCAL_CUT_RESULT_H
