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
    int externalFaceIndex = -1;  // 拼接边对面的全局邻接面（缝合阶段细分它）
    std::vector<SeamCutPoint> points;
};

// 局部阶段的拼接边切点（尚未映射到全局顶点下标，供并行收集）。
struct LocalSeam
{
    int localVertexA = -1;
    int localVertexB = -1;
    int externalFaceIndex = -1;  // 全局外部邻接面（extract 时已知）
    std::vector<SeamCutPoint> points;
};

// 单个局部单元（flood-fill 得到的 curFaces）的切割结果。
// 并行阶段只填局部数据；串行合并阶段再把局部缝边映射为全局缝边。
struct LocalCutResult
{
    std::vector<int> faceGlobals;      // 该局部单元涉及的全局面下标
    CMeshOD localMesh;                 // 切好的局部网格
    std::vector<int> localToGlobalVert; // 原始顶点 local -> global
    std::vector<int> localFaceToGlobal; // 原始面 local -> global
    int Nv0 = 0;                       // 原始顶点数
    int targetMark = 0;
    std::vector<LocalSeam> localSeams; // 局部拼接边切点表
    std::vector<SeamCutLine> seams;    // 合并阶段映射后的全局拼接边切点表
};

} // namespace MeshCutByMark

#endif // LOCAL_CUT_RESULT_H
