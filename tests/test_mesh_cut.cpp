// tests/test_mesh_cut.cpp
#include <iostream>
#include <cassert>
#include <algorithm>
#include "tool/edge_info.h"
#include "tool/polyline.h"
#include "tool/cut_plane.h"
#include "tool/region_marker.h"
#include "JasMeshMarkAndSplit.h"

void testEdgeHash() {
    MeshCutByMark::EdgeHash hash;
    std::pair<int,int> e1 = {1, 2};
    std::pair<int,int> e2 = {2, 1};

    // 相同边应该有相同的哈希值
    assert(hash(e1) == hash(e2));
    std::cout << "testEdgeHash passed" << std::endl;
}

void testBuildEdgeInfo() {
    // Create a simple test mesh: two triangles sharing edge (v1, v2)
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;

    // Add 4 vertices (reserves space first to avoid reallocation)
    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    // Add 2 triangles using vertex pointers (safe after AddVertices batch)
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable mark component and set marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;

    // Build edge info
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);

    // Edge (v1, v2) = (1, 2) is the shared edge between two faces with different marks
    assert(edgeInfo.getEdgeType(1, 2) == MeshCutByMark::CUT_EDGE_MARK_DIFF);

    // Edge (v0, v1) = (0, 1) is only in face 0, so it's a boundary edge
    assert(edgeInfo.getEdgeType(0, 1) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (v0, v2) = (0, 2) is only in face 0, so it's a boundary edge
    assert(edgeInfo.getEdgeType(0, 2) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (v1, v3) = (1, 3) is only in face 1, so it's a boundary edge
    assert(edgeInfo.getEdgeType(1, 3) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (v2, v3) = (2, 3) is only in face 1, so it's a boundary edge
    assert(edgeInfo.getEdgeType(2, 3) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // isCutEdge should return true for all edges in this mesh
    assert(edgeInfo.isCutEdge(1, 2));
    assert(edgeInfo.isCutEdge(0, 1));

    // A non-existent edge should return CUT_EDGE_NONE
    assert(edgeInfo.getEdgeType(0, 3) == MeshCutByMark::CUT_EDGE_NONE);

    // getAdjacentFaces for shared edge should return 2 faces
    auto faces = edgeInfo.getAdjacentFaces(1, 2);
    assert(faces.size() == 2);

    std::cout << "testBuildEdgeInfo passed" << std::endl;
}

void testFindCutEdges() {
    // Create a test mesh with 5 vertices and 4 faces
    //
    //   v2 ---- v3
    //   /|\     |
    //  / | \    |
    // v0 +- v1  |
    //  \ | / \  |
    //   \|/   \ |
    //    v4 --- +
    //
    // Face 0: v0, v1, v2 (mark=1) -- shares edge (1,2) with face 1 (mark diff)
    // Face 1: v1, v3, v2 (mark=2) -- shares edge (1,2) with face 0 (mark diff)
    // Face 2: v0, v4, v2 (mark=1) -- shares edge (0,2) with face 0 (same mark)
    // Face 3: v4, v1, v2 (mark=1) -- shares edge (1,2) with faces 0 and 1 (non-manifold)

    CMeshO mesh;

    // Add 5 vertices
    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 5);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);
    mesh.vert[4].P() = Point3m(0, -1, 0);

    // Add 4 faces
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[4], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[4], &mesh.vert[1], &mesh.vert[2]);

    // Enable mark component and set marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;
    mesh.face[2].IMark() = 1;
    mesh.face[3].IMark() = 1;

    // Build edge info
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);

    // Verify edge types
    // Edge (1,2) is shared by faces 0,1,3 -> non-manifold
    assert(edgeInfo.getEdgeType(1, 2) == MeshCutByMark::CUT_EDGE_NON_MANIFOLD);

    // Edge (0,2) is shared by faces 0,2 (both mark=1) -> not a cut edge
    assert(edgeInfo.getEdgeType(0, 2) == MeshCutByMark::CUT_EDGE_NONE);

    // Edge (0,1) is only in face 0 -> boundary
    assert(edgeInfo.getEdgeType(0, 1) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (0,4) is only in face 2 -> boundary
    assert(edgeInfo.getEdgeType(0, 4) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (2,4) is shared by face 2 (mark=1) and face 3 (mark=1) -> not a cut edge
    assert(edgeInfo.getEdgeType(2, 4) == MeshCutByMark::CUT_EDGE_NONE);

    // Edge (4,1) is only in face 3 -> boundary
    assert(edgeInfo.getEdgeType(1, 4) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (1,3) is only in face 1 -> boundary
    assert(edgeInfo.getEdgeType(1, 3) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Edge (2,3) is only in face 1 -> boundary
    assert(edgeInfo.getEdgeType(2, 3) == MeshCutByMark::CUT_EDGE_BOUNDARY);

    // Now test findCutEdges
    JasMeshMarkAndSplit splitter;
    splitter.SetMainMesh(&mesh);
    splitter.BuildEdgeInfo();

    std::vector<int> curFaces = {0, 1, 2, 3};
    auto cutEdges = splitter.findCutEdges(curFaces);

    // Count edges by type
    int boundaryCount = 0;
    int nonManifoldCount = 0;
    int markDiffCount = 0;

    for (const auto& edge : cutEdges) {
        switch (edge.type) {
            case MeshCutByMark::CUT_EDGE_BOUNDARY:
                boundaryCount++;
                break;
            case MeshCutByMark::CUT_EDGE_NON_MANIFOLD:
                nonManifoldCount++;
                break;
            case MeshCutByMark::CUT_EDGE_MARK_DIFF:
                markDiffCount++;
                break;
            default:
                break;
        }
    }

    // Face 0: edges (0,1) boundary, (0,2) NONE, (1,2) non-manifold -> 2 cut edges
    // Face 1: edges (1,3) boundary, (2,3) boundary, (1,2) non-manifold -> 3 cut edges
    // Face 2: edges (0,4) boundary, (2,4) NONE, (0,2) NONE -> 1 cut edge
    // Face 3: edges (1,4) boundary, (1,2) non-manifold, (2,4) NONE -> 2 cut edges
    // Total: 8 cut edges
    assert(cutEdges.size() == 8);
    assert(boundaryCount == 5);       // 5 boundary edges
    assert(nonManifoldCount == 3);    // 3 non-manifold edges (edge (1,2) appears in 3 faces)
    assert(markDiffCount == 0);       // no mark-diff edges (edge (0,2) has same marks)

    std::cout << "testFindCutEdges passed" << std::endl;
}

void testConnectEdgesToPolylines() {
    // Create test cut edges: a chain 0-1-2-3
    std::vector<MeshCutByMark::CutEdge> cutEdges;

    // Edge 1: v0-v1
    cutEdges.push_back({0, 1, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});
    // Edge 2: v1-v2
    cutEdges.push_back({1, 2, 1, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});
    // Edge 3: v2-v3
    cutEdges.push_back({2, 3, 2, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});

    MeshCutByMark::PolylineManager polylineManager;
    CMeshO mesh; // empty mesh, only used for the API signature

    auto polylines = polylineManager.connectEdgesToPolylines(cutEdges, &mesh);

    // Should connect into one polyline
    assert(polylines.size() == 1);
    assert(polylines[0].vertexIndices.size() == 4);
    assert(polylines[0].vertexIndices[0] == 0);
    assert(polylines[0].vertexIndices[1] == 1);
    assert(polylines[0].vertexIndices[2] == 2);
    assert(polylines[0].vertexIndices[3] == 3);

    std::cout << "testConnectEdgesToPolylines passed" << std::endl;
}

void testConnectEdgesToPolylinesMultiple() {
    // Test with two separate chains: 0-1-2 and 4-5
    std::vector<MeshCutByMark::CutEdge> cutEdges;

    // Chain 1: 0-1, 1-2
    cutEdges.push_back({0, 1, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});
    cutEdges.push_back({1, 2, 1, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});

    // Chain 2: 4-5
    cutEdges.push_back({4, 5, 2, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});

    MeshCutByMark::PolylineManager polylineManager;
    CMeshO mesh;

    auto polylines = polylineManager.connectEdgesToPolylines(cutEdges, &mesh);

    // Should produce 2 polylines
    assert(polylines.size() == 2);

    // Find the longer polyline (3 vertices) and the shorter one (2 vertices)
    const MeshCutByMark::Polyline* longPoly = nullptr;
    const MeshCutByMark::Polyline* shortPoly = nullptr;
    for (const auto& p : polylines) {
        if (p.vertexIndices.size() == 3) longPoly = &p;
        if (p.vertexIndices.size() == 2) shortPoly = &p;
    }

    assert(longPoly != nullptr);
    assert(shortPoly != nullptr);

    // Long polyline: 0-1-2
    assert(longPoly->vertexIndices[0] == 0);
    assert(longPoly->vertexIndices[2] == 2);

    // Short polyline: 4-5
    assert(shortPoly->vertexIndices[0] == 4);
    assert(shortPoly->vertexIndices[1] == 5);

    std::cout << "testConnectEdgesToPolylinesMultiple passed" << std::endl;
}

void testConnectEdgesToPolylinesEmpty() {
    // Empty input should return empty output
    std::vector<MeshCutByMark::CutEdge> cutEdges;
    MeshCutByMark::PolylineManager polylineManager;
    CMeshO mesh;

    auto polylines = polylineManager.connectEdgesToPolylines(cutEdges, &mesh);
    assert(polylines.empty());

    std::cout << "testConnectEdgesToPolylinesEmpty passed" << std::endl;
}

void testMakeCutPlane() {
    // Create a test mesh with two triangles sharing edge (v1, v2)
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;
    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Compute face normals
    vcg::tri::UpdateNormal<CMeshO>::PerFace(mesh);

    // Create a polyline along the shared edge
    MeshCutByMark::Polyline polyline;
    polyline.vertexIndices = {1, 2};  // edge (v1, v2)
    polyline.startFaceIdx = 0;
    polyline.startEdgeIdx = 2;  // edge V(2)->V(0) i.e. (v2, v0) in face 0? No, edge 0 = V(0)->V(1), edge 1 = V(1)->V(2), edge 2 = V(2)->V(0)
    polyline.endFaceIdx = 1;
    polyline.endEdgeIdx = 0;

    MeshCutByMark::CutPlaneManager cutPlaneManager;

    // Test cutting plane at the start of the polyline
    auto planeStart = cutPlaneManager.makeCutPlane(polyline, true, &mesh);
    assert(planeStart.Direction().Norm() > 0.99);

    // The start vertex is v1 at (1,0,0)
    // The plane should pass through this point
    double dist = vcg::SignedDistancePlanePoint(planeStart, mesh.vert[1].P());
    assert(std::abs(dist) < 1e-10);

    // Test cutting plane at the end of the polyline
    auto planeEnd = cutPlaneManager.makeCutPlane(polyline, false, &mesh);
    assert(planeEnd.Direction().Norm() > 0.99);

    // The end vertex is v2 at (0,1,0)
    dist = vcg::SignedDistancePlanePoint(planeEnd, mesh.vert[2].P());
    assert(std::abs(dist) < 1e-10);

    std::cout << "testMakeCutPlane passed" << std::endl;
}

void testMakeCutPlaneLongPolyline() {
    // Create a mesh with a polyline of 3 vertices
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;
    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    vcg::tri::UpdateNormal<CMeshO>::PerFace(mesh);

    // Polyline: 0 -> 1 -> 3
    MeshCutByMark::Polyline polyline;
    polyline.vertexIndices = {0, 1, 3};
    polyline.startFaceIdx = 0;
    polyline.startEdgeIdx = 0;
    polyline.endFaceIdx = 1;
    polyline.endEdgeIdx = 0;

    MeshCutByMark::CutPlaneManager cutPlaneManager;

    // Start plane should pass through v0 (0,0,0)
    auto planeStart = cutPlaneManager.makeCutPlane(polyline, true, &mesh);
    assert(planeStart.Direction().Norm() > 0.99);
    double dist = vcg::SignedDistancePlanePoint(planeStart, mesh.vert[0].P());
    assert(std::abs(dist) < 1e-10);

    // End plane should pass through v3 (1,1,0)
    auto planeEnd = cutPlaneManager.makeCutPlane(polyline, false, &mesh);
    assert(planeEnd.Direction().Norm() > 0.99);
    dist = vcg::SignedDistancePlanePoint(planeEnd, mesh.vert[3].P());
    assert(std::abs(dist) < 1e-10);

    // The two planes should have different directions
    // (they are at different endpoints with different edge directions)
    assert((planeStart.Direction() - planeEnd.Direction()).Norm() > 0.01);

    std::cout << "testMakeCutPlaneLongPolyline passed" << std::endl;
}

void testIsOnMarkDiffEdge() {
    // Create two triangles sharing edge (v1, v2) with different marks
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;
    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);

    // Enable marks and set different marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;

    MeshCutByMark::CutPlaneManager cutPlaneManager;

    // In face 0: edge 1 = V(1)->V(2) is the shared edge
    // This should be a mark-diff edge
    assert(cutPlaneManager.isOnMarkDiffEdge(0, 1, &mesh) == true);

    // In face 0: edge 0 = V(0)->V(1) is a boundary edge (no adjacent face)
    // Should return false
    assert(cutPlaneManager.isOnMarkDiffEdge(0, 0, &mesh) == false);

    // Now set same marks
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;

    // With same marks, the shared edge should not be a mark-diff edge
    assert(cutPlaneManager.isOnMarkDiffEdge(0, 1, &mesh) == false);

    std::cout << "testIsOnMarkDiffEdge passed" << std::endl;
}

void testSignedDistanceAndIntersection() {
    // Create a simple XY-plane at z=0
    vcg::Plane3d plane;
    plane.Init(vcg::Point3d(0, 0, 0), vcg::Point3d(0, 0, 1));

    MeshCutByMark::CutPlaneManager cutPlaneManager;

    // Use the private methods indirectly through the public interface
    // We test via makeCutPlane which internally uses signedDistance

    // Create a minimal mesh and polyline for the test
    CMeshO mesh;
    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::UpdateNormal<CMeshO>::PerFace(mesh);

    MeshCutByMark::Polyline polyline;
    polyline.vertexIndices = {0, 1};
    polyline.startFaceIdx = 0;
    polyline.startEdgeIdx = 0;
    polyline.endFaceIdx = 0;
    polyline.endEdgeIdx = 0;

    auto planeResult = cutPlaneManager.makeCutPlane(polyline, true, &mesh);

    // The plane should be well-formed
    assert(planeResult.Direction().Norm() > 0.99);

    // The start vertex (v0 at origin) should be on the plane
    double dist = vcg::SignedDistancePlanePoint(planeResult, mesh.vert[0].P());
    assert(std::abs(dist) < 1e-10);

    std::cout << "testSignedDistanceAndIntersection passed" << std::endl;
}

void testFloodFill() {
    // Create 2 triangles sharing edge (v1, v2), both with mark=1
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);

    // Enable marks and set same marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;

    // Build edge info
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);

    // Test flood-fill from face 0
    MeshCutByMark::RegionMarker regionMarker;
    auto result = regionMarker.floodFill(0, 1, &mesh, edgeInfo);

    // Should find both triangles (they share an edge and have same mark)
    assert(result.size() == 2);

    // Verify both faces are in the result
    bool hasFace0 = std::find(result.begin(), result.end(), 0) != result.end();
    bool hasFace1 = std::find(result.begin(), result.end(), 1) != result.end();
    assert(hasFace0);
    assert(hasFace1);

    std::cout << "testFloodFill passed" << std::endl;
}

void testFloodFillMarkDiff() {
    // Create 2 triangles sharing edge (v1, v2) with different marks
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);

    // Enable marks and set different marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;

    // Build edge info
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);

    // Test flood-fill from face 0 with targetMark=1
    MeshCutByMark::RegionMarker regionMarker;
    auto result = regionMarker.floodFill(0, 1, &mesh, edgeInfo);

    // Should only find face 0 (face 1 has different mark)
    assert(result.size() == 1);
    assert(result[0] == 0);

    // Test flood-fill from face 1 with targetMark=2
    auto result2 = regionMarker.floodFill(1, 2, &mesh, edgeInfo);
    assert(result2.size() == 1);
    assert(result2[0] == 1);

    std::cout << "testFloodFillMarkDiff passed" << std::endl;
}

void testFloodFillBoundary() {
    // Create a single isolated triangle
    //
    //   v2
    //   / \
    //  /   \
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);

    // Enable marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;

    // Build edge info
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);

    // Test flood-fill from the only face
    MeshCutByMark::RegionMarker regionMarker;
    auto result = regionMarker.floodFill(0, 1, &mesh, edgeInfo);

    // Should find exactly 1 face (all edges are boundaries)
    assert(result.size() == 1);
    assert(result[0] == 0);

    std::cout << "testFloodFillBoundary passed" << std::endl;
}

void testExtractSubRegions() {
    // Create 4 triangles forming a quad, with a "cut" between faces 0-1 and 2-3
    //
    //   v4 ---- v3
    //   /|      /|
    //  / |     / |
    // v0 +--- v1 |
    //  \ |     \ |
    //   \|      \|
    //    v5 ---- v2
    //
    // Actually, simpler: 4 triangles sharing a central vertex
    // Face 0: v0, v1, v4 (center)
    // Face 1: v1, v2, v4
    // Face 2: v2, v3, v4
    // Face 3: v3, v0, v4
    //
    // All have the same mark. If we "cut" between face 0 and face 3
    // (by treating edge v0-v4 as a cut edge), we get two sub-regions:
    // {0, 1, 2} and {3}

    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 5);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(1, 1, 0);
    mesh.vert[3].P() = Point3m(0, 1, 0);
    mesh.vert[4].P() = Point3m(0.5, 0.5, 0);  // center

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[2], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[2], &mesh.vert[3], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[3], &mesh.vert[0], &mesh.vert[4]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);

    // Enable marks (all same mark)
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;
    mesh.face[2].IMark() = 1;
    mesh.face[3].IMark() = 1;

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);

    // All 4 faces are in curFaces
    std::vector<int> curFaces = {0, 1, 2, 3};

    // Without any cuts, all faces should be in one region
    auto subRegions = regionMarker.extractSubRegions(curFaces, &mesh);
    assert(subRegions.size() == 1);
    assert(subRegions[0].size() == 4);

    std::cout << "testExtractSubRegions passed" << std::endl;
}

void testMarkSubRegions() {
    // Test marking sub-regions with incrementing counters
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);

    // Create two sub-regions: {face 0} and empty (to test counter increment)
    std::vector<std::vector<int>> subRegions = {{0}, {}};
    int newMarkCounter = 5;

    regionMarker.markSubRegions(subRegions, &mesh, newMarkCounter);

    // Face 0 should have newMark = 5 (first region)
    assert(regionMarker.getNewMark(0) == 5);

    // Counter should be incremented by 2 (one per region)
    assert(newMarkCounter == 7);

    std::cout << "testMarkSubRegions passed" << std::endl;
}

void testInitNewMark() {
    // Test that initNewMark zeros all face marks
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);

    // All newMarks should be 0
    assert(regionMarker.getNewMark(0) == 0);

    // Set a value and re-init
    regionMarker.setNewMark(0, 42);
    assert(regionMarker.getNewMark(0) == 42);

    regionMarker.initNewMark(&mesh);
    assert(regionMarker.getNewMark(0) == 0);

    std::cout << "testInitNewMark passed" << std::endl;
}

void testExtractBoundaryEdges() {
    // Create a single triangle
    //
    //   v2
    //   / \
    //  /   \
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

    JasMeshMarkAndSplit splitter;
    splitter.SetMainMesh(&mesh);

    std::vector<int> regionFaces = {0};
    auto boundaries = splitter.extractBoundaryEdges(regionFaces);

    // A single triangle has 3 boundary edges forming 1 closed loop
    assert(boundaries.size() == 1);
    assert(boundaries[0].size() == 3);

    // Verify the boundary contains all 3 vertex indices
    bool hasV0 = std::find(boundaries[0].begin(), boundaries[0].end(), 0) != boundaries[0].end();
    bool hasV1 = std::find(boundaries[0].begin(), boundaries[0].end(), 1) != boundaries[0].end();
    bool hasV2 = std::find(boundaries[0].begin(), boundaries[0].end(), 2) != boundaries[0].end();
    assert(hasV0);
    assert(hasV1);
    assert(hasV2);

    std::cout << "testExtractBoundaryEdges passed" << std::endl;
}

void testExtractBoundaryEdgesTwoTriangles() {
    // Create two triangles sharing edge (v1, v2)
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    JasMeshMarkAndSplit splitter;
    splitter.SetMainMesh(&mesh);

    // Both faces form one region
    std::vector<int> regionFaces = {0, 1};
    auto boundaries = splitter.extractBoundaryEdges(regionFaces);

    // Two adjacent triangles form a quad with 4 boundary edges
    assert(boundaries.size() == 1);
    assert(boundaries[0].size() == 4);

    // Verify the boundary contains all 4 vertex indices
    bool hasV0 = std::find(boundaries[0].begin(), boundaries[0].end(), 0) != boundaries[0].end();
    bool hasV1 = std::find(boundaries[0].begin(), boundaries[0].end(), 1) != boundaries[0].end();
    bool hasV2 = std::find(boundaries[0].begin(), boundaries[0].end(), 2) != boundaries[0].end();
    bool hasV3 = std::find(boundaries[0].begin(), boundaries[0].end(), 3) != boundaries[0].end();
    assert(hasV0);
    assert(hasV1);
    assert(hasV2);
    assert(hasV3);

    std::cout << "testExtractBoundaryEdgesTwoTriangles passed" << std::endl;
}

void testSplitMeshByMarkAndEdge() {
    // Create two triangles sharing edge (v1, v2) with different marks
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Set different marks on the two faces
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;

    // Run the main algorithm
    JasMeshMarkAndSplit splitter;
    splitter.SetMainMesh(&mesh);

    std::vector<JasMeshMarkAndSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    // Should produce 2 regions (one per mark)
    assert(regions.size() == 2);

    // Verify each region has correct properties
    for (const auto& reg : regions) {
        // Each region should have at least 1 triangle
        assert(reg.inTris.size() >= 1);

        // Normal should be well-formed
        assert(reg.normal.Norm() > 0.99);

        // newMark should be positive
        assert(reg.newMark > 0);

        // boundary should have at least 3 vertices (a triangle)
        assert(reg.boundlines.size() >= 3);
    }

    // Verify the two regions have different marks
    assert(regions[0].mark != regions[1].mark);

    // Verify the two regions have different newMarks
    assert(regions[0].newMark != regions[1].newMark);

    std::cout << "testSplitMeshByMarkAndEdge passed" << std::endl;
}

void testSplitMeshByMarkAndEdgeSameMark() {
    // Create two triangles sharing edge (v1, v2) with SAME mark
    //
    //   v2 ---- v3
    //   / \     |
    //  /   \    |
    // v0 --- v1
    //
    CMeshO mesh;

    vcg::tri::Allocator<CMeshO>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshO>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Same marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;

    JasMeshMarkAndSplit splitter;
    splitter.SetMainMesh(&mesh);

    std::vector<JasMeshMarkAndSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    // Should produce 1 region (both faces have same mark, connected)
    assert(regions.size() == 1);
    assert(regions[0].inTris.size() == 2);
    assert(regions[0].mark == 1);
    assert(regions[0].newMark == 1);

    std::cout << "testSplitMeshByMarkAndEdgeSameMark passed" << std::endl;
}

int main() {
    testEdgeHash();
    testBuildEdgeInfo();
    testFindCutEdges();
    testConnectEdgesToPolylines();
    testConnectEdgesToPolylinesMultiple();
    testConnectEdgesToPolylinesEmpty();
    testMakeCutPlane();
    testMakeCutPlaneLongPolyline();
    testIsOnMarkDiffEdge();
    testSignedDistanceAndIntersection();
    testFloodFill();
    testFloodFillMarkDiff();
    testFloodFillBoundary();
    testExtractSubRegions();
    testMarkSubRegions();
    testInitNewMark();
    testExtractBoundaryEdges();
    testExtractBoundaryEdgesTwoTriangles();
    testSplitMeshByMarkAndEdge();
    testSplitMeshByMarkAndEdgeSameMark();
    return 0;
}
