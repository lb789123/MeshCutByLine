// tool/polygon_mesh.h
// 多边形网格结构的便利头文件。
// 核心类型（PolygonVertex, PolygonFace, PolygonMesh）定义在
// external/cgalLocalMeshCut/JasMeshLocalMarkAndCutSplitInternal.h 中。
// 本文件仅提供从 ExactCutResult 构建 PolygonMesh 的辅助函数。
#ifndef POLYGON_MESH_H
#define POLYGON_MESH_H

#include "JasMeshLocalMarkAndCutSplitInternal.h"

namespace MeshCutByMark
{

// 从 CGAL ExactCutResult 的 polyResult 字段获取已构建的 PolygonMesh。
// CutMeshExact 内部已自动填充该字段。
inline const jaslmc::PolygonMesh& getPolygonResult(
    const jaslmc::ExactCutResult& exactResult)
{
    return exactResult.polyResult;
}

} // namespace MeshCutByMark

#endif // POLYGON_MESH_H