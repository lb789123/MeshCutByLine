// tool/cut_plane.h
#ifndef CUT_PLANE_H
#define CUT_PLANE_H

#include <vcg/space/plane3.h>
#include <vcg/space/point3.h>
#include "polyline.h"

namespace MeshCutByMark {

// Cut plane manager: constructs cutting planes from polyline endpoints
class CutPlaneManager {
public:
    // Construct a cutting plane at the start or end of a polyline
    vcg::Plane3d makeCutPlane(
        const Polyline& polyline,
        bool isStart,
        CMeshO* mesh
    );

    // Cut a triangle by a plane
    void cutTriangleByPlane(
        int faceIdx,
        const vcg::Plane3d& plane,
        CMeshO* mesh
    );

    // Check if an edge separates faces with different marks
    bool isOnMarkDiffEdge(
        int faceIdx,
        int edgeIdx,
        CMeshO* mesh
    );

private:
    // Compute signed distance from a point to a plane
    double signedDistance(const vcg::Point3d& point, const vcg::Plane3d& plane);

    // Compute intersection of a segment with a plane
    vcg::Point3d intersectSegmentPlane(
        const vcg::Point3d& p0,
        const vcg::Point3d& p1,
        double d0,
        double d1,
        const vcg::Plane3d& plane
    );
};

// --- Implementations ---

inline vcg::Plane3d CutPlaneManager::makeCutPlane(
    const Polyline& polyline,
    bool isStart,
    CMeshO* mesh
) {
    // Get endpoint vertex index and face index
    int vertexIdx = isStart ? polyline.vertexIndices[0] : polyline.vertexIndices.back();
    int faceIdx = isStart ? polyline.startFaceIdx : polyline.endFaceIdx;

    // Compute the polyline direction at the endpoint (outward)
    vcg::Point3d dir;
    if (isStart) {
        vcg::Point3d v0 = mesh->vert[polyline.vertexIndices[0]].P();
        vcg::Point3d v1 = mesh->vert[polyline.vertexIndices[1]].P();
        dir = v0 - v1;  // from v1 towards v0 (outward from polyline start)
    } else {
        int n = static_cast<int>(polyline.vertexIndices.size());
        vcg::Point3d v0 = mesh->vert[polyline.vertexIndices[n - 2]].P();
        vcg::Point3d v1 = mesh->vert[polyline.vertexIndices[n - 1]].P();
        dir = v1 - v0;  // from v0 towards v1 (outward from polyline end)
    }
    dir.Normalize();

    // Get the triangle normal
    vcg::Point3d N = mesh->face[faceIdx].N();

    // Cut plane normal = cross product of edge direction and face normal
    vcg::Point3d C = dir ^ N;
    C.Normalize();

    // Build the plane through the endpoint with normal C
    vcg::Plane3d plane;
    plane.Init(mesh->vert[vertexIdx].P(), C);

    return plane;
}

inline double CutPlaneManager::signedDistance(
    const vcg::Point3d& point,
    const vcg::Plane3d& plane
) {
    return vcg::SignedDistancePlanePoint(plane, point);
}

inline vcg::Point3d CutPlaneManager::intersectSegmentPlane(
    const vcg::Point3d& p0,
    const vcg::Point3d& p1,
    double d0,
    double d1,
    const vcg::Plane3d& /*plane*/
) {
    // Linear interpolation to find the intersection point
    double t = d0 / (d0 - d1);
    return p0 + (p1 - p0) * t;
}

inline bool CutPlaneManager::isOnMarkDiffEdge(
    int faceIdx,
    int edgeIdx,
    CMeshO* mesh
) {
    // Get the two vertices of the edge
    int v0 = mesh->face[faceIdx].V(edgeIdx)->Index();
    int v1 = mesh->face[faceIdx].V((edgeIdx + 1) % 3)->Index();

    // Check that FF adjacency is available
    if (!mesh->face.IsFFAdjacencyEnabled()) {
        return false;
    }

    // Get the adjacent face across this edge
    CFaceO* adjFace = mesh->face[faceIdx].FFp(edgeIdx);
    if (adjFace == nullptr) {
        return false;  // boundary edge
    }

    int adjFaceIdx = static_cast<int>(adjFace - &mesh->face[0]);
    if (adjFaceIdx < 0 || adjFaceIdx == faceIdx) {
        return false;  // boundary or self-referencing
    }

    // Check if marks differ between the two faces
    return mesh->face[faceIdx].IMark() != mesh->face[adjFaceIdx].IMark();
}

inline void CutPlaneManager::cutTriangleByPlane(
    int faceIdx,
    const vcg::Plane3d& plane,
    CMeshO* mesh
) {
    // Compute signed distances from each vertex to the cutting plane
    double d0 = signedDistance(mesh->face[faceIdx].V(0)->P(), plane);
    double d1 = signedDistance(mesh->face[faceIdx].V(1)->P(), plane);
    double d2 = signedDistance(mesh->face[faceIdx].V(2)->P(), plane);

    // TODO: implement full triangle splitting logic
    // Determine cutting pattern based on sign of (d0, d1, d2):
    //   - All same sign: no intersection, triangle stays on one side
    //   - One vertex on opposite side: split into 1 triangle + 1 quad (2 triangles)
    //   - Two vertices on opposite side: split into 1 triangle + 1 quad (2 triangles)
    //   - One vertex on plane: insert a new vertex on the opposite edge
    // This is a placeholder for future implementation.
    (void)d0;
    (void)d1;
    (void)d2;
}

} // namespace MeshCutByMark

#endif // CUT_PLANE_H
