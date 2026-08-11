// tool/cut_plane.h
#ifndef CUT_PLANE_H
#define CUT_PLANE_H

#include <vcg/space/plane3.h>
#include <vcg/space/point3.h>
#include "polyline.h"

namespace MeshCutByMark
{

// Cut plane manager: constructs cutting planes from polyline endpoints
class CutPlaneManager
{
public:
    // 在折线端点构造切割平面
    vcg::Plane3d makeCutPlane(
        const Polyline& polyline,
        bool isStart,
        CMeshOD* mesh
    );

    // 判断边的两侧面 mark 是否不同
    bool isOnMarkDiffEdge(
        int faceIdx,
        int edgeIdx,
        CMeshOD* mesh
    );

private:
    // 计算点到平面的有符号距离
    double signedDistance(const vcg::Point3d& point, const vcg::Plane3d& plane);

    // 计算线段与平面的交点（线性插值）
    vcg::Point3d intersectSegmentPlane(
        const vcg::Point3d& point0,
        const vcg::Point3d& point1,
        double distance0,
        double distance1,
        const vcg::Plane3d& plane
    );
};

// --- Implementations ---

// 在折线端点构造切割平面
inline vcg::Plane3d CutPlaneManager::makeCutPlane(
    const Polyline& polyline,
    bool isStart,
    CMeshOD* mesh
)
{
    // Get endpoint vertex index and face index
    int vertexIdx = isStart ? polyline.vertexIndices[0] : polyline.vertexIndices.back();
    int faceIdx = isStart ? polyline.startFaceIdx : polyline.endFaceIdx;

    // Compute the polyline direction at the endpoint (outward)
    vcg::Point3d direction;
    if (isStart)
    {
        vcg::Point3d endpointPoint = mesh->vert[polyline.vertexIndices[0]].P();
        vcg::Point3d adjacentPoint = mesh->vert[polyline.vertexIndices[1]].P();
        direction = endpointPoint - adjacentPoint; // 从相邻点指向端点（折线起点向外）
    }
    else
    {
        int vertexCount = static_cast<int>(polyline.vertexIndices.size());
        vcg::Point3d adjacentPoint = mesh->vert[polyline.vertexIndices[vertexCount - 2]].P();
        vcg::Point3d endpointPoint = mesh->vert[polyline.vertexIndices[vertexCount - 1]].P();
        direction = endpointPoint - adjacentPoint; // 从相邻点指向端点（折线终点向外）
    }
    direction.Normalize();

    // Get the triangle normal
    vcg::Point3d faceNormal = mesh->face[faceIdx].N();

    // Cut plane normal = cross product of edge direction and face normal
    vcg::Point3d cutPlaneNormal = direction ^ faceNormal;
    cutPlaneNormal.Normalize();

    // Build the plane through the endpoint with the cut plane normal
    vcg::Plane3d plane;
    plane.Init(mesh->vert[vertexIdx].P(), cutPlaneNormal);

    return plane;
}

// 计算点到平面的有符号距离
inline double CutPlaneManager::signedDistance(
    const vcg::Point3d& point,
    const vcg::Plane3d& plane
)
{
    return vcg::SignedDistancePlanePoint(plane, point);
}

// 计算线段与平面的交点（线性插值）
inline vcg::Point3d CutPlaneManager::intersectSegmentPlane(
    const vcg::Point3d& point0,
    const vcg::Point3d& point1,
    double distance0,
    double distance1,
    const vcg::Plane3d& /*plane*/
)
{
    // Linear interpolation to find the intersection point
    double interpolationFactor = distance0 / (distance0 - distance1);
    return point0 + (point1 - point0) * interpolationFactor;
}

// 判断边的两侧面 mark 是否不同
inline bool CutPlaneManager::isOnMarkDiffEdge(
    int faceIdx,
    int edgeIdx,
    CMeshOD* mesh
)
{
    // Get the two vertices of the edge
    int vertex0 = mesh->face[faceIdx].V(edgeIdx)->Index();
    int vertex1 = mesh->face[faceIdx].V((edgeIdx + 1) % 3)->Index();

    // Check that FF adjacency is available
    if (!mesh->face.IsFFAdjacencyEnabled())
    {
        return false;
    }

    // Get the adjacent face across this edge
    CFaceOD* adjacentFace = mesh->face[faceIdx].FFp(edgeIdx);
    if (adjacentFace == nullptr)
    {
        return false; // boundary edge
    }

    int adjacentFaceIdx = static_cast<int>(adjacentFace - &mesh->face[0]);
    if (adjacentFaceIdx < 0 || adjacentFaceIdx == faceIdx)
    {
        return false; // boundary or self-referencing
    }

    // Check if marks differ between the two faces
    return mesh->face[faceIdx].IMark() != mesh->face[adjacentFaceIdx].IMark();
}

} // namespace MeshCutByMark

#endif // CUT_PLANE_H
