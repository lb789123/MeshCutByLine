// tests/test_mesh_cut.cpp
#include <iostream>
#include <cassert>
#include <algorithm>
#include <set>
#include "tool/edge_info.h"
#include "tool/polyline.h"
#include "tool/cut_plane.h"
#include "tool/region_marker.h"
#include "tool/local_mesh_cut.h"
#include "JasMeshMarkAndCutSplit.h"

// 真实 cutter 由外部库 cgalLocalMeshCut（external/cgalLocalMeshCut submodule）
// 提供：JasMeshAddCutLines::AddCutLines 在链接 cglmcut 时解析。

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
    CMeshOD mesh;

    // Add 4 vertices (reserves space first to avoid reallocation)
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    // Add 2 triangles using vertex pointers (safe after AddVertices batch)
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

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

    CMeshOD mesh;

    // Add 5 vertices
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);
    mesh.vert[4].P() = Point3m(0, -1, 0);

    // Add 4 faces
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[4], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[1], &mesh.vert[2]);

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
    JasMeshMarkAndCutSplit splitter;
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
    CMeshOD mesh; // empty mesh, only used for the API signature

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
    CMeshOD mesh;

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

void testConnectEdgesToPolylinesDeduplicate()
{
    // 同一几何边被相邻面各记录一次（同向或反向），折线中不应出现回头重叠线段
    MeshCutByMark::PolylineManager polylineManager;
    CMeshOD mesh;

    // Case 1: 同向重复记录只保留一条
    std::vector<MeshCutByMark::CutEdge> duplicateEdges =
    {
        {0, 1, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF},
        {0, 1, 1, 1, MeshCutByMark::CUT_EDGE_MARK_DIFF}
    };
    auto duplicatePolylines = polylineManager.connectEdgesToPolylines(duplicateEdges, &mesh);
    assert(duplicatePolylines.size() == 1);
    assert(duplicatePolylines[0].vertexIndices.size() == 2);

    // Case 2: 反向记录同样只保留一条
    std::vector<MeshCutByMark::CutEdge> oppositeEdges =
    {
        {0, 1, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF},
        {1, 0, 1, 1, MeshCutByMark::CUT_EDGE_MARK_DIFF}
    };
    auto oppositePolylines = polylineManager.connectEdgesToPolylines(oppositeEdges, &mesh);
    assert(oppositePolylines.size() == 1);
    assert(oppositePolylines[0].vertexIndices.size() == 2);

    // Case 3: 链条中间边重复时，保持 0-1-2，不出现 0-1-2-1 回头线
    std::vector<MeshCutByMark::CutEdge> chainEdges =
    {
        {0, 1, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF},
        {1, 2, 1, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF},
        {2, 1, 2, 1, MeshCutByMark::CUT_EDGE_MARK_DIFF}
    };
    auto chainPolylines = polylineManager.connectEdgesToPolylines(chainEdges, &mesh);
    assert(chainPolylines.size() == 1);
    assert(chainPolylines[0].vertexIndices.size() == 3);
    assert(chainPolylines[0].vertexIndices[0] == 0);
    assert(chainPolylines[0].vertexIndices[1] == 1);
    assert(chainPolylines[0].vertexIndices[2] == 2);

    // Case 4: 反向重复记录混在链条中（2-7、2-3、3-2），不应出现 2-3-2-7 回头线
    std::vector<MeshCutByMark::CutEdge> reverseChainEdges =
    {
        {2, 7, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF},
        {2, 3, 1, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF},
        {3, 2, 2, 1, MeshCutByMark::CUT_EDGE_MARK_DIFF}
    };
    auto reverseChainPolylines = polylineManager.connectEdgesToPolylines(reverseChainEdges, &mesh);
    assert(reverseChainPolylines.size() == 1);
    assert(reverseChainPolylines[0].vertexIndices.size() == 3);
    // 不允许出现相邻顶点重复（即回头折返）
    for (int vertexIndex = 0; vertexIndex + 1 < (int)reverseChainPolylines[0].vertexIndices.size(); vertexIndex++)
    {
        assert(reverseChainPolylines[0].vertexIndices[vertexIndex] !=
               reverseChainPolylines[0].vertexIndices[vertexIndex + 1]);
    }
    // 三个顶点必须是 2、3、7
    std::vector<int> sortedVertices = reverseChainPolylines[0].vertexIndices;
    std::sort(sortedVertices.begin(), sortedVertices.end());
    assert(sortedVertices[0] == 2 && sortedVertices[1] == 3 && sortedVertices[2] == 7);

    std::cout << "testConnectEdgesToPolylinesDeduplicate passed" << std::endl;
}

void testConnectEdgesToPolylinesEmpty() {
    // Empty input should return empty output
    std::vector<MeshCutByMark::CutEdge> cutEdges;
    MeshCutByMark::PolylineManager polylineManager;
    CMeshOD mesh;

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
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Compute face normals
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);

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
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);

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
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

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
    // signedDistance/intersectSegmentPlane are private; we exercise the public makeCutPlane here

    // Create a minimal mesh and polyline for the test
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);

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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

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

void testBuildCutInput() {
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

    MeshCutByMark::LocalMeshCutManager mgr;
    std::vector<int> curFaces = {0};
    MeshCutByMark::LocalMeshCutManager::LocalMesh lm;
    mgr.extractLocalMesh(&mesh, curFaces, lm);

    // 折线 0->1，端点 v0 悬空
    MeshCutByMark::Polyline pl;
    pl.vertexIndices = {0, 1};
    pl.startFaceIdx = 0; pl.startEdgeIdx = 0;
    pl.endFaceIdx = 0; pl.endEdgeIdx = 0;

    // 仅首端延长：切割输入 = 首端延长段 + 折线本体（2 个顶点）-> 共 3 点
    auto ci = mgr.buildCutInput(pl, true, false, lm, &mesh);
    // 切割输入 = 延长段 + 折线本体（2 个顶点）-> 共 3 点
    assert(ci.line.size() == 3);
    // 延长段起点 = v0 + 方向*L，方向 v0-v1 归一化
    vcg::Point3d D = (mesh.vert[0].P() - mesh.vert[1].P()); D.Normalize();
    vcg::Point3d seg = ci.line[0] - ci.line[1]; seg.Normalize();
    assert((seg - D).Norm() < 1e-9);
    // 折线本体：v0 -> v1
    assert((ci.line[1] - mesh.vert[0].P()).Norm() < 1e-9);
    assert((ci.line[2] - mesh.vert[1].P()).Norm() < 1e-9);
    // normal = 面法向 (0,0,1)
    assert(std::abs(ci.normal.Z() - 1.0) < 1e-9);

    // 首尾两端都延长：切割输入 = 首端延长段 + 折线本体 + 尾端延长段 -> 共 4 点
    auto ciBoth = mgr.buildCutInput(pl, true, true, lm, &mesh);
    assert(ciBoth.line.size() == 4);
    assert((ciBoth.line[1] - mesh.vert[0].P()).Norm() < 1e-9);
    assert((ciBoth.line[2] - mesh.vert[1].P()).Norm() < 1e-9);
    // 尾端延长方向 = v1 - v0 归一化
    vcg::Point3d endDirection = mesh.vert[1].P() - mesh.vert[0].P();
    endDirection.Normalize();
    vcg::Point3d endSeg = ciBoth.line[3] - ciBoth.line[2];
    endSeg.Normalize();
    assert((endSeg - endDirection).Norm() < 1e-9);
    std::cout << "testBuildCutInput passed" << std::endl;
}

void testMergeBack() {
    // 主网格：1 个三角形 (v0,v1,v2)
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    // OCF enables: extractLocalMesh 内会跑 FFp seam 循环，且下面要写 IMark()；
    // 未 enable 会触发 UB（空 OCF 向量）。与 testExtractLocalMesh 对齐。
    // 必须在 IMark()=5 之前 enable，否则写入未分配存储。
    mesh.face.EnableFFAdjacency();
    mesh.face.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    mesh.face[0].IMark() = 5;

    MeshCutByMark::LocalMeshCutManager mgr;
    MeshCutByMark::LocalMeshCutManager::LocalMesh lm;
    mgr.extractLocalMesh(&mesh, {0}, lm);
    // 模拟 cutter：把 local 面0 分裂——在边 (v0,v1) 中点加新顶点 nv，
    // 用面 (v0,nv,v2) 替换 face0，加面 (nv,v1,v2)。无需写来源属性。
    vcg::tri::Allocator<CMeshOD>::AddVertices(lm.mesh, 1);
    int nv = (int)lm.mesh.vert.size() - 1;  // 新顶点 local 下标 (>= Nv0)
    lm.mesh.vert[nv].P() = vcg::Point3d(0.5, 0, 0);
    lm.mesh.face[0].V(1) = &lm.mesh.vert[nv];          // face0 改成 (v0,nv,v2)
    vcg::tri::Allocator<CMeshOD>::AddFace(lm.mesh,
        &lm.mesh.vert[nv], &lm.mesh.vert[1], &lm.mesh.vert[2]);  // 新面 (nv,v1,v2)

    auto res = mgr.mergeBack(&mesh, lm, /*targetMark*/ 5);

    // 主网格：face0 槽位被原位改写（不 SetD，拓扑连续）；额外分片 append 1 个。
    assert(!mesh.face[0].IsD());
    assert(mesh.face[0].IMark() == 5);
    assert(mesh.face[0].V(1)->Index() == 3);  // 改写为 (v0, nv, v2)，nv=全局下标 3
    // 只有 1 个 append 的额外分片（(nv,v1,v2)）继承 mark=5
    int aliveNew = 0;
    for (int i = 1; i < (int)mesh.face.size(); i++) {
        if (!mesh.face[i].IsD() && mesh.face[i].IMark() == 5) aliveNew++;
    }
    assert(aliveNew == 1);
    // 顶点 append 了一个新顶点
    assert((int)mesh.vert.size() == 4);
    std::cout << "testMergeBack passed" << std::endl;
}

void testMergeBackSharedEdge() {
    // face0=(v0,v1,v2), face1=(v1,v3,v2) 共享边 (v1,v2)。都 mark=5。
    //   v2 ---- v3
    //    \     |
    //     \    |
    //   v0 --- v1
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    mesh.vert[3].P() = Point3m(1,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]); // face0
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]); // face1
    mesh.face.EnableFFAdjacency();
    mesh.face.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    mesh.face[0].IMark() = 5;
    mesh.face[1].IMark() = 5;

    MeshCutByMark::LocalMeshCutManager mgr;
    MeshCutByMark::LocalMeshCutManager::LocalMesh lm;
    mgr.extractLocalMesh(&mesh, {0, 1}, lm);
    // 模拟 cutter 只切 face0：在 face0 的边 (v0,v1) 中点加 nv，
    // face0 改成 (v0,nv,v2)，加 (nv,v1,v2)。注意 (nv,v1,v2) 的完整边是 (v1,v2)=共享边。
    vcg::tri::Allocator<CMeshOD>::AddVertices(lm.mesh, 1);
    int nv = (int)lm.mesh.vert.size() - 1;
    lm.mesh.vert[nv].P() = vcg::Point3d(0.5, 0, 0);
    lm.mesh.face[0].V(1) = &lm.mesh.vert[nv];            // face0 -> (v0,nv,v2)
    vcg::tri::Allocator<CMeshOD>::AddFace(lm.mesh,
        &lm.mesh.vert[nv], &lm.mesh.vert[1], &lm.mesh.vert[2]);  // 新面 (nv,v1,v2)

    auto res = mgr.mergeBack(&mesh, lm, /*targetMark*/ 5);

    // 关键断言：被切的是 face0 -> 槽位原位改写（不 SetD）；邻居 face1 没被切 -> 保持
    assert(!mesh.face[0].IsD());
    assert(!mesh.face[1].IsD());
    std::cout << "testMergeBackSharedEdge passed" << std::endl;
}

void testExtractLocalMesh() {
    // 两个三角形共享边 (v1,v2)，都 mark=1
    //   v2 ---- v3
    //    \     |
    //     \    |
    //   v0 --- v1
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    mesh.vert[3].P() = Point3m(1,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

    std::vector<int> curFaces = {0, 1};
    MeshCutByMark::LocalMeshCutManager mgr;
    MeshCutByMark::LocalMeshCutManager::LocalMesh lm;
    mgr.extractLocalMesh(&mesh, curFaces, lm);

    assert(lm.mesh.vert.size() == 4);      // 4 unique verts
    assert(lm.mesh.face.size() == 2);      // 2 faces
    assert(lm.Nv0 == 4);                    // all original
    assert(lm.localToGlobalVert.size() == 4);
    assert(lm.localFaceToGlobal.size() == 2);
    assert(lm.localFaceToGlobal[0] == 0);
    assert(lm.localFaceToGlobal[1] == 1);
    // 顶点坐标一致
    assert((lm.mesh.vert[0].P() - mesh.vert[0].P()).Norm() < 1e-9);
    std::cout << "testExtractLocalMesh passed" << std::endl;
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

    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(1, 1, 0);
    mesh.vert[3].P() = Point3m(0, 1, 0);
    mesh.vert[4].P() = Point3m(0.5, 0.5, 0);  // center

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[2], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[3], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[3], &mesh.vert[0], &mesh.vert[4]);

    // Enable FF adjacency and compute topology
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);

    JasMeshMarkAndCutSplit splitter;
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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    JasMeshMarkAndCutSplit splitter;
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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Set different marks on the two faces
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;

    // Run the main algorithm
    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);

    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
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
    CMeshOD mesh;

    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);

    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);

    // Same marks
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;

    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);

    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    // Should produce 1 region (both faces have same mark, connected)
    assert(regions.size() == 1);
    assert(regions[0].inTris.size() == 2);
    assert(regions[0].mark == 1);
    assert(regions[0].newMark == 1);

    std::cout << "testSplitMeshByMarkAndEdgeSameMark passed" << std::endl;
}

void testIntegration() {
    // Create a more complex test mesh: 6 vertices, 4 triangles, 2 mark regions
    //
    //   v2 ---- v3 ---- v5
    //   / \     | \     |
    //  /   \    |  \    |
    // v0 --- v1   v4 --+
    //
    // Face 0: v0, v1, v2  (mark=1)
    // Face 1: v1, v3, v2  (mark=1)
    // Face 2: v1, v4, v3  (mark=2)
    // Face 3: v4, v5, v3  (mark=2)
    //
    // Faces 0-1 share edge (1,2), same mark => not a cut edge
    // Faces 1-2 share edge (1,3), different mark => cut edge
    // Faces 2-3 share edge (3,4), same mark => not a cut edge
    CMeshOD mesh;

    // Add 6 vertices
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 6);
    mesh.vert[0].P() = Point3m(0, 0, 0);
    mesh.vert[1].P() = Point3m(1, 0, 0);
    mesh.vert[2].P() = Point3m(0, 1, 0);
    mesh.vert[3].P() = Point3m(1, 1, 0);
    mesh.vert[4].P() = Point3m(2, 0, 0);
    mesh.vert[5].P() = Point3m(2, 1, 0);

    // Add 4 triangles
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[4], &mesh.vert[3]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[5], &mesh.vert[3]);

    // Set marks: faces 0-1 => mark 1, faces 2-3 => mark 2
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;
    mesh.face[2].IMark() = 2;
    mesh.face[3].IMark() = 2;

    // Run the main algorithm
    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);

    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    // Verify: should produce 2 regions (one per mark value)
    assert(regions.size() == 2);

    // Verify: all 4 triangles are assigned across regions
    int totalFaces = 0;
    for (const auto& reg : regions) {
        totalFaces += reg.inTris.size();
    }
    assert(totalFaces == 4);

    // Verify: each region has a valid boundary (at least 3 vertices forming a closed loop)
    for (const auto& reg : regions) {
        assert(reg.boundlines.size() >= 3);
    }

    // Verify: the two regions have different original marks
    assert(regions[0].mark != regions[1].mark);

    // Verify: each region has a well-formed normal
    for (const auto& reg : regions) {
        assert(reg.normal.Norm() > 0.99);
    }

    // Verify: each region has a positive newMark
    for (const auto& reg : regions) {
        assert(reg.newMark > 0);
    }

    std::cout << "testIntegration passed" << std::endl;
}

void testMarkCutEdges() {
    // 两个三角形拼成四边形，中间一条边要被标成分割边
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    mesh.vert[3].P() = Point3m(1,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[3]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[3], &mesh.vert[2]);
    mesh.face.EnableFFAdjacency();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

    // cutLine: 假装顶点 0 和 3 之间的边是切割边（global 顶点 0,3）
    std::vector<std::vector<int>> cutLines = { {0, 3} };
    MeshCutByMark::LocalMeshCutManager mgr;
    mgr.markCutEdges(&mesh, cutLines);

    // 边 (0,3) 两侧都应 FFp 自指
    auto checkBoundary = [&](int f, int e){
        return mesh.face[f].FFp(e) == &mesh.face[f];
    };
    // 找到以 (0,3) 为边的面边，确认两侧自指
    bool found = false;
    for (int f = 0; f < 2; f++) {
        for (int e = 0; e < 3; e++) {
            int a = mesh.face[f].V(e)->Index();
            int b = mesh.face[f].V((e+1)%3)->Index();
            auto k = std::minmax(a,b);
            if (k == std::minmax(0,3)) {
                assert(checkBoundary(f, e));
                found = true;
            }
        }
    }
    assert(found);
    std::cout << "testMarkCutEdges passed" << std::endl;
}

void testPropagateExternal() {
    // curFaces=face0，外部邻接面=face1 共享边 (v0,v2)；新顶点落该边
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    mesh.vert[3].P() = Point3m(-1,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]); // face0 区域内
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[2], &mesh.vert[3]); // face1 外部，共享 (v0,v2)
    mesh.face.EnableFFAdjacency();
    // OCF: splitExternalFace 会读写 IMark()（新面继承外部面 mark）；
    // 未 EnableMark 会读未分配存储 -> UB。与 testExtractLocalMesh/testMergeBack 对齐。
    mesh.face.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

    MeshCutByMark::LocalMeshCutManager mgr;
    MeshCutByMark::LocalMeshCutManager::LocalMesh lm;
    mgr.extractLocalMesh(&mesh, {0}, lm);
    // 在 local 边 (v0_local, v2_local) 中点加新顶点
    vcg::tri::Allocator<CMeshOD>::AddVertices(lm.mesh, 1);
    int nv = (int)lm.mesh.vert.size() - 1;
    lm.mesh.vert[nv].P() = vcg::Point3d(0, 0.5, 0);  // (v0,v2) 中点
    // local v0=0, v2=2；缝边 key = minmax(0,2)
    assert(lm.seamExternal.count({0,2}) == 1);  // 抓到了外部面 face1

    // 手工 merge：local 原顶点 0,1,2 -> global 0,1,2；新顶点(local 3) -> global 4
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 1);
    mesh.vert[4].P() = vcg::Point3d(0, 0.5, 0);
    MeshCutByMark::LocalMeshCutManager::MergeResult mr;
    mr.vertLocalToGlobal = {0, 1, 2, 4};

    mgr.propagateExternal(&mesh, lm, mr);

    // 外部面 face1 被 SetD，且新增了把 face1 分成两块的面
    assert(mesh.face[1].IsD());
    int newExt = 0;
    for (int i = 2; i < (int)mesh.face.size(); i++)
        if (!mesh.face[i].IsD()) newExt++;
    assert(newExt == 2);  // 原外部面一分为二

    // 回归：新面必须保持原外部面 (0,2,3) 的绕序（法向 +Z）
    for (int faceIndex = 2; faceIndex < (int)mesh.face.size(); faceIndex++)
    {
        if (mesh.face[faceIndex].IsD())
        {
            continue;
        }
        vcg::Point3d newFaceNormal =
            (mesh.face[faceIndex].V(1)->P() - mesh.face[faceIndex].V(0)->P()) ^
            (mesh.face[faceIndex].V(2)->P() - mesh.face[faceIndex].V(0)->P());
        assert(newFaceNormal.Z() > 0);
    }

    // 反绕序回归：face1 存为 (0,3,2)（原法向 -Z），两个新面仍应与原面同向
    {
        CMeshOD reversedMesh;
        vcg::tri::Allocator<CMeshOD>::AddVertices(reversedMesh, 4);
        reversedMesh.vert[0].P() = Point3m(0,0,0);
        reversedMesh.vert[1].P() = Point3m(1,0,0);
        reversedMesh.vert[2].P() = Point3m(0,1,0);
        reversedMesh.vert[3].P() = Point3m(-1,1,0);
        vcg::tri::Allocator<CMeshOD>::AddFace(reversedMesh, &reversedMesh.vert[0], &reversedMesh.vert[1], &reversedMesh.vert[2]);
        vcg::tri::Allocator<CMeshOD>::AddFace(reversedMesh, &reversedMesh.vert[0], &reversedMesh.vert[3], &reversedMesh.vert[2]);
        reversedMesh.face.EnableFFAdjacency();
        reversedMesh.face.EnableMark();
        vcg::tri::UpdateTopology<CMeshOD>::FaceFace(reversedMesh);

        MeshCutByMark::LocalMeshCutManager::LocalMesh reversedLocalMesh;
        mgr.extractLocalMesh(&reversedMesh, {0}, reversedLocalMesh);
        vcg::tri::Allocator<CMeshOD>::AddVertices(reversedLocalMesh.mesh, 1);
        int reversedNewVertex = (int)reversedLocalMesh.mesh.vert.size() - 1;
        reversedLocalMesh.mesh.vert[reversedNewVertex].P() = vcg::Point3d(0, 0.5, 0);
        assert(reversedLocalMesh.seamExternal.count({0,2}) == 1);

        vcg::tri::Allocator<CMeshOD>::AddVertices(reversedMesh, 1);
        reversedMesh.vert[4].P() = vcg::Point3d(0, 0.5, 0);
        MeshCutByMark::LocalMeshCutManager::MergeResult reversedMergeResult;
        reversedMergeResult.vertLocalToGlobal = {0, 1, 2, 4};
        mgr.propagateExternal(&reversedMesh, reversedLocalMesh, reversedMergeResult);

        for (int faceIndex = 2; faceIndex < (int)reversedMesh.face.size(); faceIndex++)
        {
            if (reversedMesh.face[faceIndex].IsD())
            {
                continue;
            }
            vcg::Point3d newFaceNormal =
                (reversedMesh.face[faceIndex].V(1)->P() - reversedMesh.face[faceIndex].V(0)->P()) ^
                (reversedMesh.face[faceIndex].V(2)->P() - reversedMesh.face[faceIndex].V(0)->P());
            assert(newFaceNormal.Z() < 0);
        }
    }
    std::cout << "testPropagateExternal passed" << std::endl;
}

void testGrowNewMark() {
    MeshCutByMark::RegionMarker rm;
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    rm.initNewMark(&mesh);          // m_newMark = {0}
    rm.setNewMark(0, 7);
    rm.growNewMark(3);              // 扩到 3
    assert(rm.getNewMark(0) == 7);  // 已有不重置
    assert(rm.getNewMark(1) == 0);  // 新增为 0
    assert(rm.getNewMark(2) == 0);
    std::cout << "testGrowNewMark passed" << std::endl;
}

void testRebuildCurFaces() {
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]); // face1 新
    mesh.face[0].SetD();
    std::vector<int> curFaces = {0};
    MeshCutByMark::LocalMeshCutManager::MergeResult mr;
    mr.newFaceGlobals = {1};
    MeshCutByMark::LocalMeshCutManager::rebuildCurFaces(curFaces, &mesh, mr);
    assert(curFaces.size() == 1 && curFaces[0] == 1);
    std::cout << "testRebuildCurFaces passed" << std::endl;
}

void testCutRegionPlumbing() {
	// 区域 2 个三角形 mark=1（单位正方形），构造一条 NON_MANIFOLD 折线端点悬空。
	// 真实 cutter（cgalLocalMeshCut）下：端点 v0 沿折线方向（对角线）切穿区域，
	// 两个原面都被切开并 SetD，新面 append，curFaces 重建。
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
	mesh.vert[0].P() = Point3m(0,0,0);
	mesh.vert[1].P() = Point3m(1,0,0);
	mesh.vert[2].P() = Point3m(0,1,0);
	mesh.vert[3].P() = Point3m(1,1,0);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
	mesh.face.EnableFFAdjacency(); mesh.face.EnableMark();
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
	vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
	mesh.face[0].IMark() = 1; mesh.face[1].IMark() = 1;

	MeshCutByMark::RegionMarker rm; rm.initNewMark(&mesh);
	std::vector<int> curFaces = {0, 1};

	MeshCutByMark::Polyline pl;
	pl.type = MeshCutByMark::CUT_EDGE_NON_MANIFOLD;
	// 方向取 v0 -> v3（对角线），保证切割线从端点切入区域内部而非沿边界边
	pl.vertexIndices = {0, 3};
	pl.startFaceIdx = 0; pl.startEdgeIdx = 0;
	pl.endFaceIdx = 0;   pl.endEdgeIdx = 0;  // 端点不在 mark-diff 边 -> 触发切割

	MeshCutByMark::LocalMeshCutManager mgr;
	int newMarkCounter = 1;
	mgr.cutRegion(&mesh, curFaces, {pl}, /*targetMark*/1, rm, newMarkCounter);

	// 真实 cutter 切割后拓扑保持连续：原始面槽位被替换/保留，不会 IsD。
	// plumbing 只验证管线跑通与 curFaces 有效（切不动时保守 no-op 也合法）。
	assert(curFaces.size() >= 2);
	assert(!mesh.face[0].IsD());
	assert(!mesh.face[1].IsD());

	// AddCutLines 在 local mesh 上按“切割边不可跨越”完成区域拆分并重新标记，
	// cutRegion 应把 local 区域标记同步为全局 new-mark：对角线切割后，
	// curFaces 中每个面都被标记，且恰好分成两个不同区域。
	std::set<int> regionMarks;
	for (int faceIndex : curFaces)
	{
		assert(rm.getNewMark(faceIndex) > 0);
		regionMarks.insert(rm.getNewMark(faceIndex));
	}
	assert(regionMarks.size() == 2);
	std::cout << "testCutRegionPlumbing passed" << std::endl;
}

int main() {
    testEdgeHash();
    testBuildEdgeInfo();
    testFindCutEdges();
    testConnectEdgesToPolylines();
    testConnectEdgesToPolylinesMultiple();
    testConnectEdgesToPolylinesDeduplicate();
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
    testIntegration();
    testExtractLocalMesh();
    testBuildCutInput();
    testMergeBack();
    testMergeBackSharedEdge();
    testMarkCutEdges();
    testPropagateExternal();
    testGrowNewMark();
    testRebuildCurFaces();
    testCutRegionPlumbing();
    return 0;
}
