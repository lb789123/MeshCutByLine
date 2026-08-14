#ifndef LOCAL_CUT_RESULT_H
#define LOCAL_CUT_RESULT_H

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
};

} // namespace MeshCutByMark

#endif // LOCAL_CUT_RESULT_H
