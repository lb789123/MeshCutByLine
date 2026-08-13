#ifndef LOCAL_CUT_RESULT_H
#define LOCAL_CUT_RESULT_H

#include <vector>
#include <utility>
#include "cmesh.h"

namespace MeshCutByMark
{

// 拼接边上的单个切点。
// 迁移第 1 步先用 double 坐标收集，后续统一换成精确核坐标比较。
struct SeamCutPoint
{
    int localVertexIndex = -1;   // 局部顶点下标
    int globalVertexIndex = -1;  // 全局顶点下标（合并后有效）
    double t = 0.0;              // 沿拼接边 a->b 的投影参数，用于排序
    vcg::Point3d point;          // 切点坐标
};

// 一条拼接边的切点集合，按沿边方向有序。
struct SeamCutLine
{
    int globalVertexA = -1;      // 拼接边端点（规范化，小下标在前）
    int globalVertexB = -1;
    std::vector<SeamCutPoint> points;
};

// 单个局部单元（flood-fill 得到的 curFaces）的切割结果。
// 第 1 步仅收集数据，不改变现有切割行为；后续步骤再把它作为并行局部与
// 全局合并之间的唯一交换结构。
struct LocalCutResult
{
    std::vector<int> faceGlobals;      // 该局部单元涉及的全局面下标
    std::vector<SeamCutLine> seams;    // 拼接边切点表
};

} // namespace MeshCutByMark

#endif // LOCAL_CUT_RESULT_H
