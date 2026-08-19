// tests/test_mesh_cut.cpp
// Release 配置下 CMake 默认带 NDEBUG，会把裸 assert 编译成空操作，测试形同虚设
// （2026-08 多边形切割路径回归正是因此漏网）。在包含 <cassert> 前取消 NDEBUG，
// 保证断言在 Debug/Release 两种配置下都生效。
#undef NDEBUG
#include <iostream>
#include <cassert>
#include <algorithm>
#include <array>
#include <set>
#include <tuple>
#include "tool/edge_info.h"
#include "tool/polyline.h"
#include "tool/region_marker.h"
#include "tool/local_mesh_cut.h"
#include "JasMeshMarkAndCutSplit.h"

// 真实 cutter 由外部库 cgalLocalMeshCut（external/cgalLocalMeshCut submodule）
// 提供：JasMeshAddCutLines::AddCutLines 在链接 cglmcut 时解析。

// Release 下裸 assert 被 NDEBUG 关闭，改用 REQUIRE 保证关键断言在两种配置下都拦截。
#define REQUIRE(cond) \
    do { if (!(cond)) { \
        std::cerr << "REQUIRE failed: " << #cond << " @ " << __FILE__ \
                  << ":" << __LINE__ << std::endl; \
        std::abort(); \
    } } while (0)

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

// 回归：同一条区域内多条互不相同的复杂折线连续切割时，
// AddCutLines 通过 f:source 确定分片父面，不得产生重复/退化三角形，面积必须守恒。
static CMeshOD BuildTempGridMesh(int cellCount, double waveHeight)
{
	CMeshOD mesh;
	int vertexPerSide = cellCount + 1;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, vertexPerSide * vertexPerSide);
	for (int j = 0; j <= cellCount; j++)
	{
		for (int i = 0; i <= cellCount; i++)
		{
			double lift = waveHeight * std::sin(i * 0.7) * std::cos(j * 0.6);
			mesh.vert[j * vertexPerSide + i].P() = vcg::Point3d(i, j, lift);
		}
	}
	for (int j = 0; j < cellCount; j++)
	{
		for (int i = 0; i < cellCount; i++)
		{
			int bl = j * vertexPerSide + i;
			int br = j * vertexPerSide + i + 1;
			int tl = (j + 1) * vertexPerSide + i;
			int tr = (j + 1) * vertexPerSide + i + 1;
			vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[tl], &mesh.vert[bl], &mesh.vert[br]);
			vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[tl], &mesh.vert[br], &mesh.vert[tr]);
		}
	}
	mesh.face.EnableFFAdjacency();
	mesh.face.EnableMark();
	mesh.vert.EnableMark();
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
	vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
	for (auto& face : mesh.face)
	{
		face.IMark() = 1;
	}
	return mesh;
}

static void CollectTempTriples(const CMeshOD& mesh, std::map<std::tuple<int, int, int>, int>& triples)
{
	triples.clear();
	for (const auto& face : mesh.face)
	{
		if (face.IsD())
		{
			continue;
		}
		std::array<int, 3> vertices = { face.V(0)->Index(), face.V(1)->Index(), face.V(2)->Index() };
		std::sort(vertices.begin(), vertices.end());
		triples[std::make_tuple(vertices[0], vertices[1], vertices[2])]++;
	}
}

static double CollectTempArea(const CMeshOD& mesh)
{
	double totalArea = 0.0;
	for (const auto& face : mesh.face)
	{
		if (face.IsD())
		{
			continue;
		}
		totalArea += ((face.P(1) - face.P(0)) ^ (face.P(2) - face.P(0))).Norm() * 0.5;
	}
	return totalArea;
}

struct TempManifoldStats
{
	int nonManifoldEdgeCount = 0;
	int nonManifoldVertexCount = 0;
	int coincidentVertexPairCount = 0;
};

// 流形检查：非流形边（>2 面共享）、非流形点（顶点邻面按共享边连通分量 >1）、重合顶点对
static TempManifoldStats CollectTempManifoldStats(const CMeshOD& mesh)
{
	TempManifoldStats stats;
	std::map<std::pair<int, int>, std::vector<int>> edgeFaces;
	std::map<int, std::vector<int>> vertexFaces;
	for (int faceIndex = 0; faceIndex < (int)mesh.face.size(); faceIndex++)
	{
		const auto& face = mesh.face[faceIndex];
		if (face.IsD())
		{
			continue;
		}
		for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
		{
			int vertexA = face.V(edgeIndex)->Index();
			int vertexB = face.V((edgeIndex + 1) % 3)->Index();
			auto edgeKey = std::minmax(vertexA, vertexB);
			edgeFaces[edgeKey].push_back(faceIndex);
		}
	}
	for (const auto& entry : edgeFaces)
	{
		if (entry.second.size() > 2)
		{
			stats.nonManifoldEdgeCount++;
		}
		for (int faceIndex : entry.second)
		{
			vertexFaces[entry.first.first].push_back(faceIndex);
			vertexFaces[entry.first.second].push_back(faceIndex);
		}
	}
	for (const auto& entry : vertexFaces)
	{
		std::set<int> faceSet(entry.second.begin(), entry.second.end());
		std::set<int> visited;
		int componentCount = 0;
		for (int startFace : faceSet)
		{
			if (visited.count(startFace))
			{
				continue;
			}
			componentCount++;
			std::vector<int> stack = { startFace };
			visited.insert(startFace);
			while (!stack.empty())
			{
				int currentFace = stack.back();
				stack.pop_back();
				for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
				{
					int vertexA = mesh.face[currentFace].V(edgeIndex)->Index();
					int vertexB = mesh.face[currentFace].V((edgeIndex + 1) % 3)->Index();
					if (vertexA != entry.first && vertexB != entry.first)
					{
						continue;
					}
					auto edgeKey = std::minmax(vertexA, vertexB);
					for (int neighborFace : edgeFaces[edgeKey])
					{
						if (neighborFace != currentFace && faceSet.count(neighborFace) &&
							!visited.count(neighborFace))
						{
							visited.insert(neighborFace);
							stack.push_back(neighborFace);
						}
					}
				}
			}
		}
		if (componentCount > 1)
		{
			stats.nonManifoldVertexCount++;
		}
	}
	for (int vertexIndexA = 0; vertexIndexA < (int)mesh.vert.size(); vertexIndexA++)
	{
		if (mesh.vert[vertexIndexA].IsD())
		{
			continue;
		}
		for (int vertexIndexB = vertexIndexA + 1; vertexIndexB < (int)mesh.vert.size(); vertexIndexB++)
		{
			if (mesh.vert[vertexIndexB].IsD())
			{
				continue;
			}
			if ((mesh.vert[vertexIndexA].P() - mesh.vert[vertexIndexB].P()).Norm() < 1e-9)
			{
				stats.coincidentVertexPairCount++;
			}
		}
	}
	return stats;
}

// 回归：真实非流形边（3 面共享一条边）下 f:source 路径不崩溃、不丢已存在面
void testCutRegionNonManifoldEdgeStability()
{
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
	mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
	mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
	mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
	mesh.vert[3].P() = vcg::Point3d(0.5, -0.5, 0.5);
	mesh.vert[4].P() = vcg::Point3d(0.5, 0.5, -0.5);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[3]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[4]);
	mesh.face.EnableFFAdjacency();
	mesh.face.EnableMark();
	mesh.vert.EnableMark();
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
	vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
	for (auto& face : mesh.face)
	{
		face.IMark() = 1;
	}

	JasMeshAddCutLines cutter;
	vcg::Point3d normal(0, 0, 1);
	std::vector<vcg::Point3d> line = {
		vcg::Point3d(0, 0, 0), vcg::Point3d(1, 0, 0) };
	std::vector<int> cutLine;
	cutter.AddCutLines(&mesh, normal, line, cutLine);

	int liveFaceCount = 0;
	int duplicateCount = 0;
	std::map<std::tuple<int, int, int>, int> triples;
	for (const auto& face : mesh.face)
	{
		if (face.IsD())
		{
			continue;
		}
		liveFaceCount++;
		std::array<int, 3> vertices = { face.V(0)->Index(), face.V(1)->Index(), face.V(2)->Index() };
		std::sort(vertices.begin(), vertices.end());
		triples[std::make_tuple(vertices[0], vertices[1], vertices[2])]++;
	}
	for (const auto& entry : triples)
	{
		if (entry.second > 1)
		{
			duplicateCount++;
		}
	}
	std::cout << "nonManifoldEdgeCut: liveFaces=" << liveFaceCount
		<< " duplicateTriples=" << duplicateCount << std::endl;
	assert(duplicateCount == 0);
	assert(liveFaceCount >= 3);
	std::cout << "testCutRegionNonManifoldEdgeStability passed" << std::endl;
}

// 回归：propagateExternal 在缝边上加点时，新顶点与缝边端点重合必须被跳过

// 回归：缝边上的切割新顶点必须在外部邻接面上被加点细分（无裂缝）；
// 同缝边多顶点一次分割，同一外部面多条缝边分割后更新邻居映射。
static int CountTempBoundaryEdges(const CMeshOD& mesh)
{
	std::map<std::pair<int, int>, int> edgeCount;
	for (const auto& face : mesh.face)
	{
		if (face.IsD())
		{
			continue;
		}
		for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
		{
			int vertexA = face.V(edgeIndex)->Index();
			int vertexB = face.V((edgeIndex + 1) % 3)->Index();
			edgeCount[std::minmax(vertexA, vertexB)]++;
		}
	}
	int boundaryCount = 0;
	for (const auto& entry : edgeCount)
	{
		if (entry.second == 1)
		{
			boundaryCount++;
		}
	}
	return boundaryCount;
}

static CMeshOD BuildTempSeamMesh()
{
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
	mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
	mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
	mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
	mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
	// 面 0 = 区域（mark 1），面 1 = 外部邻接（mark 2），共享缝边 (1,2)
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
	mesh.face.EnableFFAdjacency();
	mesh.face.EnableMark();
	mesh.vert.EnableMark();
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
	vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
	mesh.face[0].IMark() = 1;
	mesh.face[1].IMark() = 2;
	return mesh;
}

// 回归：stitchAllSeams 合并两侧缝边切点，消除双侧裂缝
void testStitchAllSeams()
{
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 6);
	mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
	mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
	mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
	mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
	mesh.vert[4].P() = vcg::Point3d(1, 0.5, 0);
	mesh.vert[5].P() = vcg::Point3d(1, 0.5, 0);
	// A 侧 face0 已被切割成 (1,4,0),(4,2,0)
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[4], &mesh.vert[0]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[2], &mesh.vert[0]);
	// B 侧 face1 已被切割成 (2,5,3),(5,1,3)
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[5], &mesh.vert[3]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[5], &mesh.vert[1], &mesh.vert[3]);
	mesh.face.EnableMark();
	for (int faceIndex = 0; faceIndex < 2; faceIndex++)
	{
		mesh.face[faceIndex].IMark() = 1;
	}
	for (int faceIndex = 2; faceIndex < 4; faceIndex++)
	{
		mesh.face[faceIndex].IMark() = 2;
	}

	MeshCutByMark::LocalCutResult resultA;
	MeshCutByMark::SeamCutLine seamA;
	seamA.globalVertexA = 1;
	seamA.globalVertexB = 2;
	seamA.externalFaceIndices.push_back(3);
	MeshCutByMark::SeamCutPoint pointA;
	pointA.globalVertexIndex = 4;
	pointA.point = vcg::Point3d(1, 0.5, 0);
	pointA.exactPoint = jaslmc::ExactPoint(1, 0.5, 0);
	seamA.points.push_back(pointA);
	resultA.seams.push_back(seamA);

	MeshCutByMark::LocalCutResult resultB;
	MeshCutByMark::SeamCutLine seamB;
	seamB.globalVertexA = 1;
	seamB.globalVertexB = 2;
	seamB.externalFaceIndices.push_back(0);
	MeshCutByMark::SeamCutPoint pointB;
	pointB.globalVertexIndex = 5;
	pointB.point = vcg::Point3d(1, 0.5, 0);
	pointB.exactPoint = jaslmc::ExactPoint(1, 0.5, 0);
	seamB.points.push_back(pointB);
	resultB.seams.push_back(seamB);

	MeshCutByMark::RegionMarker regionMarker;
	regionMarker.initNewMark(&mesh);
	MeshCutByMark::LocalMeshCutManager::stitchAllSeams(
		&mesh, { resultA, resultB }, regionMarker);

	std::map<std::pair<int, int>, int> edgeCount;
	for (const auto& face : mesh.face)
	{
		if (face.IsD())
		{
			continue;
		}
		for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
		{
			int vertexA = face.V(edgeIndex)->Index();
			int vertexB = face.V((edgeIndex + 1) % 3)->Index();
			edgeCount[std::minmax(vertexA, vertexB)]++;
		}
	}
	int boundaryEdges = 0;
	for (const auto& entry : edgeCount)
	{
		if (entry.second == 1)
		{
			boundaryEdges++;
		}
	}
	std::cout << "stitchAllSeams: boundaryEdges=" << boundaryEdges << std::endl;
	std::cout << "testStitchAllSeams passed" << std::endl;
}

// 回归：prepareLocalCut 后局部网格按切割边分区标记
void testPrepareLocalCutMarks()
{
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
	mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
	mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
	mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
	mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
	mesh.face.EnableFFAdjacency();
	mesh.face.EnableMark();
	mesh.vert.EnableMark();
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
	vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
	mesh.face[0].IMark() = 1;
	mesh.face[1].IMark() = 1;

	MeshCutByMark::Polyline polyline;
	polyline.type = MeshCutByMark::CUT_EDGE_NON_MANIFOLD;
	polyline.vertexIndices = { 0, 3 };
	MeshCutByMark::LocalMeshCutManager manager;
	MeshCutByMark::LocalCutResult result;
	manager.prepareLocalCut(&mesh, { 0, 1 }, { polyline }, 1, result);

	std::set<int> marks;
	for (auto faceIndex : result.exact.mesh.faces())
	{
		marks.insert(result.exact.face_mark_map[faceIndex]);
	}
    std::cout << "prepareLocalCutMarks: distinctMarks=" << marks.size()
        << " liveFaces=" << result.exact.mesh.number_of_faces() << std::endl;
    REQUIRE(marks.size() == 2);
    std::cout << "testPrepareLocalCutMarks passed" << std::endl;
}

// 回归：CutFacesExact 直接从全局面集构建 ExactMesh、切割、分区并收集缝边
void testCutFacesExact()
{
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
	mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
	mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
	mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
	mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
	mesh.face.EnableFFAdjacency();
	mesh.face.EnableMark();
	mesh.vert.EnableMark();
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
	mesh.face[0].IMark() = 1;
	mesh.face[1].IMark() = 1;

	std::vector<jaslmc::ExactPoint> normals = { jaslmc::ExactPoint(0, 0, 1) };
	std::vector<std::vector<jaslmc::ExactPoint>> lines = {
		{ jaslmc::ExactPoint(-2, -1, 0), jaslmc::ExactPoint(0, 0, 0),
		  jaslmc::ExactPoint(2, 1, 0), jaslmc::ExactPoint(4, 2, 0) } };
	jaslmc::ExactCutResult result;
	bool ok = jaslmc::CutFacesExact(mesh, { 0, 1 }, normals, lines, result);

	std::set<int> marks;
	for (auto face_index : result.mesh.faces())
	{
		marks.insert(result.face_mark_map[face_index]);
	}
    std::cout << "cutFacesExact: ok=" << ok
        << " faces=" << result.mesh.number_of_faces()
        << " distinctMarks=" << marks.size()
        << " seams=" << result.seams.size()
        << " dropped=" << result.dropped_input_face_count << std::endl;
    REQUIRE(marks.size() == 2);
    std::cout << "testCutFacesExact passed" << std::endl;
}

// 回归：SplitMeshByMarkAndEdge 输出多边形法向与内部三角形一致
void testSplitMeshNormals()
{
	std::cout << "splitMeshNormals: start" << std::endl;
	CMeshOD mesh;
	vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
	mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
	mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
	mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
	mesh.vert[3].P() = vcg::Point3d(2, 0, 0);
	mesh.vert[4].P() = vcg::Point3d(0, 2, 0);
	// mark1 区域：F1、F2；mark2 区域：F3；边 (1,2) 被 3 面共享（非流形）
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
	vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[4], &mesh.vert[2]);
	mesh.face.EnableMark();
	mesh.vert.EnableMark();
	mesh.face[0].IMark() = 1;
	mesh.face[1].IMark() = 1;
	mesh.face[2].IMark() = 2;

	JasMeshMarkAndCutSplit splitter;
	splitter.SetMainMesh(&mesh);
	std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
	try
	{
		splitter.SplitMeshByMarkAndEdge(regions);
	}
	catch (const std::exception& exception)
	{
		std::cout << "splitMeshNormals: exception=" << exception.what() << std::endl;
		return;
	}

	std::cout << "splitMeshNormals: regions=" << regions.size() << std::endl;
	int reversedNormalCount = 0;
	for (const auto& region : regions)
	{
		std::cout << "  region newMark=" << region.newMark
			<< " normal=(" << region.normal.X() << "," << region.normal.Y()
			<< "," << region.normal.Z() << ") tris=" << region.inTris.size()
			<< std::endl;
		if (region.normal.Z() < 0)
		{
			reversedNormalCount++;
		}
	}
	std::cout << "splitMeshNormals: reversedNormalCount="
		<< reversedNormalCount << std::endl;
	std::cout << "testSplitMeshNormals passed" << std::endl;
}

// 回归（步骤1a）：非流形边应在进入 corefine 前被劈开，CutFacesExact 不再
// 因 CGAL add_face 拒收而丢面；步骤 1b 的 dropped_faces 护栏仅在意外失败时兜底。
void testCutFacesExactDropRecording()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(0, -1, 0);
    mesh.vert[4].P() = vcg::Point3d(0, 0, 1);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[0], &mesh.vert[3]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[4]);
    mesh.face.EnableMark();
    mesh.face.EnableFFAdjacency();
    mesh.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    for (auto& face : mesh.face)
    {
        face.IMark() = 1;
    }

    // 只验证「构建局部 ExactMesh 时非流形边已劈开」，不进行实际切割。
    std::vector<jaslmc::ExactPoint> normals;
    std::vector<std::vector<jaslmc::ExactPoint>> lines;
    jaslmc::ExactCutResult result;
    jaslmc::CutFacesExact(mesh, { 0, 1, 2 }, normals, lines, result);

    std::cout << "cutFacesExactDropRecording: dropped="
        << result.dropped_input_face_count
        << " recorded=" << result.dropped_faces.size()
        << " exactFaces=" << result.mesh.number_of_faces() << std::endl;
    REQUIRE(result.dropped_input_face_count == 0);
    REQUIRE(result.dropped_faces.empty());
    REQUIRE(result.mesh.number_of_faces() == 3);
    std::cout << "testCutFacesExactDropRecording passed" << std::endl;
}

// 回归（步骤1b+2）：含非流形边的网格整体分割后，无 newMark=0 垃圾区、面数守恒、
// 每个区域的 mark 与输入 mark 一致。
void testNonManifoldEdgeRegion()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[4].P() = vcg::Point3d(0, 2, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[4], &mesh.vert[2]);
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;
    mesh.face[2].IMark() = 2;

    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    int liveFaces = 0;
    for (const auto& face : mesh.face)
    {
        if (!face.IsD())
        {
            liveFaces++;
        }
    }
    std::set<int> coveredFaces;
    for (const auto& region : regions)
    {
        REQUIRE(region.newMark >= 1);
        REQUIRE(region.mark == 1 || region.mark == 2);
        for (int faceIndex : region.inTris)
        {
            coveredFaces.insert(faceIndex);
        }
    }
    REQUIRE((int)coveredFaces.size() == liveFaces);
    std::cout << "testNonManifoldEdgeRegion passed: regions=" << regions.size()
        << " faces=" << liveFaces << std::endl;
}

// 回归（步骤2）：splitReg.mark 必须等于该区域面的原始 IMark，不被局部重标污染。
void testSplitRegMarkPreserved()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(1, 1, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 2;

    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    for (const auto& region : regions)
    {
        REQUIRE(!region.inTris.empty());
        const int expectedMark = mesh.face[region.inTris[0]].IMark();
        REQUIRE(region.mark == expectedMark);
    }
    std::cout << "testSplitRegMarkPreserved passed: regions=" << regions.size()
        << std::endl;
}

// 回归（步骤4a/7c）：star vertex 区域应保守跳过，并且贯穿完整
// SplitMeshByMarkAndEdge 时面数守恒、不崩溃。
void testStarVertexSkip()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(-1, 0, 0);
    mesh.vert[4].P() = vcg::Point3d(0, -1, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[3], &mesh.vert[4]);
    mesh.face.EnableMark();
    mesh.face.EnableFFAdjacency();
    mesh.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;

    std::vector<jaslmc::ExactPoint> normals = { jaslmc::ExactPoint(0, 0, 1) };
    std::vector<std::vector<jaslmc::ExactPoint>> lines = {
        { jaslmc::ExactPoint(-2, 0, 0), jaslmc::ExactPoint(2, 0, 0) } };
    jaslmc::ExactCutResult result;
    jaslmc::CutFacesExact(mesh, { 0, 1 }, normals, lines, result);
    REQUIRE(result.skipped);

    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    int liveFaces = 0;
    for (const auto& face : mesh.face)
    {
        if (!face.IsD())
        {
            liveFaces++;
        }
    }
    REQUIRE(liveFaces == 2);
    REQUIRE(regions.size() == 2);
    int coveredFaces = 0;
    for (const auto& region : regions)
    {
        REQUIRE(region.mark == 1);
        coveredFaces += (int)region.inTris.size();
    }
    REQUIRE(coveredFaces == 2);
    std::cout << "testStarVertexSkip passed" << std::endl;
}

// 回归（步骤3）：精确坐标缝合后，不得残留坐标精确相等但索引不同的顶点对。
void testSeamExactDedup()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 6);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
    mesh.vert[4].P() = vcg::Point3d(1, 0.5, 0);
    mesh.vert[5].P() = vcg::Point3d(1, 0.5, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[4], &mesh.vert[0]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[2], &mesh.vert[0]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[5], &mesh.vert[3]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[5], &mesh.vert[1], &mesh.vert[3]);
    mesh.face.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;
    mesh.face[2].IMark() = 2;
    mesh.face[3].IMark() = 2;

    MeshCutByMark::LocalCutResult resultA;
    MeshCutByMark::SeamCutLine seamA;
    seamA.globalVertexA = 1;
    seamA.globalVertexB = 2;
    seamA.externalFaceIndices.push_back(3);
    MeshCutByMark::SeamCutPoint pointA;
    pointA.globalVertexIndex = 4;
    pointA.point = vcg::Point3d(1, 0.5, 0);
    pointA.exactPoint = jaslmc::ExactPoint(1, 0.5, 0);
    seamA.points.push_back(pointA);
    resultA.seams.push_back(seamA);

    MeshCutByMark::LocalCutResult resultB;
    MeshCutByMark::SeamCutLine seamB;
    seamB.globalVertexA = 1;
    seamB.globalVertexB = 2;
    seamB.externalFaceIndices.push_back(0);
    MeshCutByMark::SeamCutPoint pointB;
    pointB.globalVertexIndex = 5;
    pointB.point = vcg::Point3d(1, 0.5, 0);
    pointB.exactPoint = jaslmc::ExactPoint(1, 0.5, 0);
    seamB.points.push_back(pointB);
    resultB.seams.push_back(seamB);

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);
    MeshCutByMark::LocalMeshCutManager::stitchAllSeams(
        &mesh, { resultA, resultB }, regionMarker);

    TempManifoldStats stats = CollectTempManifoldStats(mesh);
    std::cout << "seamExactDedup: boundaryEdges=" << CountTempBoundaryEdges(mesh)
        << " coincidentVertexPairs=" << stats.coincidentVertexPairCount << std::endl;
    REQUIRE(stats.coincidentVertexPairCount == 0);
    REQUIRE(CountTempBoundaryEdges(mesh) == 4);
    std::cout << "testSeamExactDedup passed" << std::endl;
}

// 回归（步骤4b）：非流形缝边有两个外部邻接面时，两个外部面都要被纯分割。
void testSeamMultiExternal()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(0, -1, 0);
    mesh.vert[4].P() = vcg::Point3d(1, 0, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[0], &mesh.vert[3]);
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;

    MeshCutByMark::LocalCutResult result;
    MeshCutByMark::SeamCutLine seam;
    seam.globalVertexA = 0;
    seam.globalVertexB = 1;
    seam.externalFaceIndices = { 0, 1 };
    MeshCutByMark::SeamCutPoint point;
    point.globalVertexIndex = 4;
    point.point = vcg::Point3d(1, 0, 0);
    point.exactPoint = jaslmc::ExactPoint(1, 0, 0);
    seam.points.push_back(point);
    result.seams.push_back(seam);

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);
    MeshCutByMark::LocalMeshCutManager::stitchAllSeams(
        &mesh, { result }, regionMarker);

    int liveFaces = 0;
    for (const auto& face : mesh.face)
    {
        if (!face.IsD())
        {
            liveFaces++;
        }
    }
    std::cout << "seamMultiExternal: liveFaces=" << liveFaces
        << " boundaryEdges=" << CountTempBoundaryEdges(mesh) << std::endl;
    REQUIRE(mesh.face[0].IsD());
    REQUIRE(mesh.face[1].IsD());
    REQUIRE(liveFaces == 4);
    REQUIRE(CountTempBoundaryEdges(mesh) == 4);
    std::cout << "testSeamMultiExternal passed" << std::endl;
}

// 回归（步骤4c）：切割产生的精确点若已存在于全局网格，mergeLocalCut 必须复用。
void testExistingVertexReuse()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 4);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 2, 0);
    mesh.vert[3].P() = vcg::Point3d(0.5, 0.5, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    mesh.face.EnableMark();
    mesh.face.EnableFFAdjacency();
    mesh.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    mesh.face[0].IMark() = 1;

    jaslmc::ExactCutResult exact;
    exact.vertex_global_map =
        exact.mesh.add_property_map<jaslmc::ExactMesh::Vertex_index, int>(
            "v:g", -1).first;
    exact.face_global_map =
        exact.mesh.add_property_map<jaslmc::ExactMesh::Face_index, int>(
            "f:global", -1).first;
    exact.face_mark_map =
        exact.mesh.add_property_map<jaslmc::ExactMesh::Face_index, int>(
            "f:mark", -1).first;
    jaslmc::ExactMesh::Vertex_index vertexA =
        exact.mesh.add_vertex(jaslmc::ExactPoint(0, 0, 0));
    jaslmc::ExactMesh::Vertex_index vertexB =
        exact.mesh.add_vertex(jaslmc::ExactPoint(2, 0, 0));
    jaslmc::ExactMesh::Vertex_index vertexC =
        exact.mesh.add_vertex(jaslmc::ExactPoint(0.5, 0.5, 0));
    exact.vertex_global_map[vertexA] = 0;
    exact.vertex_global_map[vertexB] = 1;
    exact.vertex_global_map[vertexC] = -1;
    jaslmc::ExactMesh::Face_index exactFace =
        exact.mesh.add_face(vertexA, vertexB, vertexC);
    REQUIRE(exactFace != jaslmc::ExactMesh::null_face());
    exact.face_global_map[exactFace] = 0;
    exact.face_mark_map[exactFace] = 1;

    std::map<jaslmc::ExactPoint, int> existingPointToVertex;
    for (int vertexIndex = 0; vertexIndex < (int)mesh.vert.size(); vertexIndex++)
    {
        if (mesh.vert[vertexIndex].IsD())
        {
            continue;
        }
        const vcg::Point3d& point = mesh.vert[vertexIndex].P();
        existingPointToVertex[jaslmc::ExactPoint(point.X(), point.Y(), point.Z())] =
            vertexIndex;
    }

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);
    MeshCutByMark::LocalCutResult localResult;
    localResult.exact = std::move(exact);
    localResult.faceGlobals = { 0 };
    localResult.targetMark = 1;
    MeshCutByMark::LocalMeshCutManager manager;
    int newMarkCounter = 1;
    const int vertexCountBefore = mesh.vn;
    manager.mergeLocalCut(&mesh, localResult, regionMarker, newMarkCounter,
        existingPointToVertex);

    TempManifoldStats stats = CollectTempManifoldStats(mesh);
    std::cout << "existingVertexReuse: vertices=" << mesh.vn
        << " before=" << vertexCountBefore
        << " coincidentVertexPairs=" << stats.coincidentVertexPairCount << std::endl;
    REQUIRE(mesh.vn == vertexCountBefore);
    REQUIRE(stats.coincidentVertexPairCount == 0);
    REQUIRE(mesh.face[0].V(2)->Index() == 3);
    std::cout << "testExistingVertexReuse passed" << std::endl;
}

// 回归：同一个外部面被两条缝边引用时，第一条缝边拆分后必须把第二条缝边的
// 外部面引用重定向到包含该边的子面，否则第二条缝边会被漏掉并留下裂缝。
void testSeamRedirectAfterSplit()
{
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 2, 0);
    mesh.vert[3].P() = vcg::Point3d(1, 0, 0);
    mesh.vert[4].P() = vcg::Point3d(1, 1, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    mesh.face[0].IMark() = 2;

    MeshCutByMark::LocalCutResult resultA;
    MeshCutByMark::SeamCutLine seamA;
    seamA.globalVertexA = 0;
    seamA.globalVertexB = 1;
    seamA.externalFaceIndices.push_back(0);
    MeshCutByMark::SeamCutPoint pointA;
    pointA.globalVertexIndex = 3;
    pointA.point = vcg::Point3d(1, 0, 0);
    pointA.exactPoint = jaslmc::ExactPoint(1, 0, 0);
    seamA.points.push_back(pointA);
    resultA.seams.push_back(seamA);

    MeshCutByMark::LocalCutResult resultB;
    MeshCutByMark::SeamCutLine seamB;
    seamB.globalVertexA = 1;
    seamB.globalVertexB = 2;
    seamB.externalFaceIndices.push_back(0);
    MeshCutByMark::SeamCutPoint pointB;
    pointB.globalVertexIndex = 4;
    pointB.point = vcg::Point3d(1, 1, 0);
    pointB.exactPoint = jaslmc::ExactPoint(1, 1, 0);
    seamB.points.push_back(pointB);
    resultB.seams.push_back(seamB);

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);
    MeshCutByMark::LocalMeshCutManager::stitchAllSeams(
        &mesh, { resultA, resultB }, regionMarker);

    int liveFaces = 0;
    for (const auto& face : mesh.face)
    {
        if (!face.IsD())
        {
            liveFaces++;
        }
    }
    std::cout << "seamRedirectAfterSplit: liveFaces=" << liveFaces
        << " boundaryEdges=" << CountTempBoundaryEdges(mesh) << std::endl;
    REQUIRE(mesh.face[0].IsD());
    REQUIRE(liveFaces == 3);
    std::cout << "testSeamRedirectAfterSplit passed" << std::endl;
}

void testComplexCutStress()
{
    const int cellCount = 4;
    CMeshOD mesh = BuildTempGridMesh(cellCount, 0.15);
    const int vertexPerSide = cellCount + 1;
    mesh.face.EnableFFAdjacency();
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
    for (auto& face : mesh.face)
    {
        face.IMark() = 1;
    }

    std::vector<int> curFaces;
    for (int faceIndex = 0; faceIndex < (int)mesh.face.size(); faceIndex++)
    {
        curFaces.push_back(faceIndex);
    }

    std::vector<std::vector<int>> polylineVertices = {
        { 0, vertexPerSide + 1, 2 * (vertexPerSide + 1), 3 * (vertexPerSide + 1), 4 * (vertexPerSide + 1) },
        { vertexPerSide - 1, vertexPerSide + 1, 2 * vertexPerSide + 1, 3 * vertexPerSide + 1, 4 * vertexPerSide },
        { 2, 2 * vertexPerSide + 2, 4 * vertexPerSide + 2 },
        { vertexPerSide, vertexPerSide + 2, 2 * vertexPerSide + 2, 3 * vertexPerSide + 2, 4 * vertexPerSide + 2 },
        { 1, vertexPerSide + 1, 2 * vertexPerSide, 3 * vertexPerSide + 2, 4 * vertexPerSide + 1 },
        { 0, vertexPerSide, 2 * vertexPerSide + 1, 3 * vertexPerSide + 1, 4 * vertexPerSide + 2 },
        { 3, 2 * vertexPerSide + 1, 3 * vertexPerSide + 3, 4 * vertexPerSide + 3 },
        { 4, 2 * vertexPerSide + 2, 3 * vertexPerSide + 1, 4 * vertexPerSide },
    };
    std::vector<MeshCutByMark::Polyline> polylines;
    for (const auto& vertices : polylineVertices)
    {
        MeshCutByMark::Polyline polyline;
        polyline.type = MeshCutByMark::CUT_EDGE_NON_MANIFOLD;
        polyline.vertexIndices = vertices;
        polylines.push_back(polyline);
    }

    MeshCutByMark::LocalMeshCutManager manager;
    MeshCutByMark::LocalCutResult result;
    manager.prepareLocalCut(&mesh, curFaces, polylines, 1, result);
    std::cout << "complexCutStress: faces=" << result.exact.mesh.number_of_faces()
        << " seams=" << result.exact.seams.size()
        << " skipped=" << result.exact.skipped << std::endl;
    std::cout << "testComplexCutStress passed" << std::endl;
}

// 回归（多边形切割路径）：prepareLocalCutPolygon + mergeLocalCutPolygon 直连。
// 内部顶点风扇网格 + 悬空切割线（无边界折线 → 双向延长）穿出区域边界内部，
// 产生真实切点。验证独立合并的契约：不切分/不复制任何面、无孤儿 newMark、
// 切点以孤立顶点追加、边界环全部引用有效全局下标且包含切点。
void testPolygonPathMergeDirect()
{
    // 四边形区域 0(0,0) 1(2,0) 3(2,1) 2(0,1)，内部顶点 4(1,0.4) 风扇三角化。
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
    mesh.vert[4].P() = vcg::Point3d(1, 0.4, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[3], &mesh.vert[2], &mesh.vert[4]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[0], &mesh.vert[4]);
    mesh.face.EnableFFAdjacency();
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
    for (auto& face : mesh.face)
    {
        face.IMark() = 1;
    }

    MeshCutByMark::Polyline polyline;
    polyline.type = MeshCutByMark::CUT_EDGE_NON_MANIFOLD;
    polyline.vertexIndices = { 0, 4 };
    MeshCutByMark::LocalMeshCutManager manager;
    MeshCutByMark::LocalCutResult result;
    manager.prepareLocalCutPolygon(&mesh, { 0, 1, 2, 3 }, { polyline }, 1, result);
    REQUIRE(!result.exact.skipped);

    MeshCutByMark::RegionMarker regionMarker;
    regionMarker.initNewMark(&mesh);
    std::map<jaslmc::ExactPoint, int> existingPointToVertex;
    for (int vi = 0; vi < (int)mesh.vert.size(); vi++)
    {
        const auto& p = mesh.vert[vi].P();
        existingPointToVertex[jaslmc::ExactPoint(p.X(), p.Y(), p.Z())] = vi;
    }
    int newMarkCounter = 1;
    manager.mergeLocalCutPolygon(&mesh, result, regionMarker, newMarkCounter,
        existingPointToVertex);

    // 面不被切分、不被复制；两个子区域 newMark 互异且 > 0（无孤儿）。
    REQUIRE((int)mesh.face.size() == 4);
    REQUIRE(newMarkCounter == 3);
    REQUIRE(regionMarker.getNewMark(0) != 0);
    REQUIRE(regionMarker.getNewMark(1) != 0);
    REQUIRE(regionMarker.getNewMark(2) != 0);
    REQUIRE(regionMarker.getNewMark(3) != 0);
    REQUIRE(regionMarker.getNewMark(0) != regionMarker.getNewMark(3));
    // 切线从右边 (2,0)-(2,1) 内部穿出 → 恰好追加 1 个切点孤立顶点。
    REQUIRE((int)mesh.vert.size() == 6);
    // 两个 newMark 各有边界环；环内下标全部有效，且包含新增切点顶点。
    REQUIRE(result.polyLoops.size() == 2);
    std::set<int> loopVerts;
    for (const auto& entry : result.polyLoops)
    {
        REQUIRE(!entry.second.empty());
        for (const auto& loop : entry.second)
        {
            REQUIRE(loop.size() >= 3);
            for (int vi : loop)
            {
                REQUIRE(vi >= 0);
                REQUIRE(vi < (int)mesh.vert.size());
                loopVerts.insert(vi);
            }
        }
    }
    bool hasCutVertex = false;
    for (int vi : loopVerts)
    {
        if (vi >= 5)
        {
            hasCutVertex = true;
        }
    }
    REQUIRE(hasCutVertex);
    std::cout << "polygonPathMergeDirect: faces=" << mesh.face.size()
        << " verts=" << mesh.vert.size()
        << " loops=" << result.polyLoops.size() << std::endl;
    std::cout << "testPolygonPathMergeDirect passed" << std::endl;
}

// 回归（多边形切割路径全管线）：SplitMeshByMarkAndEdge 在多边形路径下
// 不再产生孤儿面风暴 —— 面数不变、区域数等于任务数、inTris 恰好划分
// 全部存活面、mark 保留、边界环非空。
void testPolygonPathGridCut()
{
    // 复用 testNonManifoldEdgeRegion 拓扑：边 (1,2) 三面共边（非流形），
    // 三个 flood 任务互不连通，各产出 1 个区域，共 3 个。
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 5);
    mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
    mesh.vert[1].P() = vcg::Point3d(1, 0, 0);
    mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
    mesh.vert[3].P() = vcg::Point3d(2, 0, 0);
    mesh.vert[4].P() = vcg::Point3d(0, 2, 0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[2]);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[4], &mesh.vert[2]);
    mesh.face.EnableMark();
    mesh.vert.EnableMark();
    mesh.face[0].IMark() = 1;
    mesh.face[1].IMark() = 1;
    mesh.face[2].IMark() = 2;

    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);

    // 多边形路径不修改任何面：无重复面、无删除。
    REQUIRE((int)mesh.face.size() == 3);
    int liveFaces = 0;
    for (const auto& face : mesh.face)
    {
        if (!face.IsD())
        {
            liveFaces++;
        }
    }
    REQUIRE(liveFaces == 3);
    REQUIRE(regions.size() == 3);

    std::set<int> coveredFaces;
    std::set<int> newMarks;
    for (const auto& region : regions)
    {
        REQUIRE(region.newMark > 0);
        newMarks.insert(region.newMark);
        REQUIRE(region.mark == 1 || region.mark == 2);
        REQUIRE(!region.inTris.empty());
        REQUIRE(region.boundaries.size() >= 1);
        REQUIRE(region.boundaries[0].size() >= 3);
        REQUIRE(region.boundlines.size() >= 3);
        for (int faceIndex : region.inTris)
        {
            coveredFaces.insert(faceIndex);
        }
    }
    REQUIRE((int)newMarks.size() == 3);
    REQUIRE((int)coveredFaces.size() == liveFaces);
    std::cout << "polygonPathGridCut: regions=" << regions.size()
        << " faces=" << liveFaces << std::endl;
    std::cout << "testPolygonPathGridCut passed" << std::endl;
}

void testCutPathModeSelect()
{
    // 同一拓扑分别用两种切割路径跑全管线：外部参数 SetCutPathMode 决定
    // 走 prepareLocalCutPolygon（多边形）还是 prepareLocalCut（三角形）。
    // 两条路径的输出契约相同（retRegs / splitReg），差异在网格改写：
    // 三角形路径真实切割写回（面被切分、面数增长），多边形路径不切分
    // 任何面（切点追加为孤立顶点）。
    // 拓扑：四边形风扇 0-1-3-2（内部顶点 4，面 0..3 mark=1）+ 翻折面
    // (0,4,5) mark=2，使边 (0,4) 三面共边（非流形）。区域 A（4 面）的
    // 切割折线 {0,4} 端点 4 不在 A 的边界顶点集内 → 延长穿出面 1 内部，
    // 从右边 (2,0)-(2,1) 中点附近穿出 → 真实切割（非退化）。
    auto makeMesh = [](CMeshOD& mesh)
    {
        vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 6);
        mesh.vert[0].P() = vcg::Point3d(0, 0, 0);
        mesh.vert[1].P() = vcg::Point3d(2, 0, 0);
        mesh.vert[2].P() = vcg::Point3d(0, 1, 0);
        mesh.vert[3].P() = vcg::Point3d(2, 1, 0);
        mesh.vert[4].P() = vcg::Point3d(1, 0.4, 0);
        mesh.vert[5].P() = vcg::Point3d(0.5, -0.3, 0.5); // 翻折出平面，避免共面重叠
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[4]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[4]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[3], &mesh.vert[2], &mesh.vert[4]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[0], &mesh.vert[4]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[4], &mesh.vert[5]);
        mesh.face.EnableFFAdjacency();
        mesh.face.EnableMark();
        mesh.vert.EnableMark();
        vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
        vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
        mesh.face[0].IMark() = 1;
        mesh.face[1].IMark() = 1;
        mesh.face[2].IMark() = 1;
        mesh.face[3].IMark() = 1;
        mesh.face[4].IMark() = 2;
    };
    auto countLiveFaces = [](CMeshOD& mesh)
    {
        int liveFaces = 0;
        for (const auto& face : mesh.face)
        {
            if (!face.IsD())
            {
                liveFaces++;
            }
        }
        return liveFaces;
    };
    auto checkRegionsValid =
        [](const std::vector<JasMeshMarkAndCutSplit::splitReg>& regions, int liveFaces)
    {
        REQUIRE(regions.size() >= 3);
        int markOneRegions = 0;
        std::set<int> coveredFaces;
        std::set<int> newMarks;
        for (const auto& region : regions)
        {
            REQUIRE(region.newMark > 0);
            newMarks.insert(region.newMark);
            REQUIRE(region.mark == 1 || region.mark == 2);
            if (region.mark == 1)
            {
                markOneRegions++;
            }
            REQUIRE(!region.inTris.empty());
            REQUIRE(region.boundaries.size() >= 1);
            REQUIRE(region.boundaries[0].size() >= 3);
            for (int faceIndex : region.inTris)
            {
                coveredFaces.insert(faceIndex);
            }
        }
        // 切割线把区域 A 分成两半 + 翻折面单独一区，两条路径一致。
        REQUIRE(markOneRegions == 2);
        REQUIRE((int)newMarks.size() == (int)regions.size());
        REQUIRE((int)coveredFaces.size() == liveFaces);
    };

    // 三角形路径：真实切割写回，面 1 被切分，面数增长。
    {
        CMeshOD mesh;
        makeMesh(mesh);
        JasMeshMarkAndCutSplit splitter;
        splitter.SetMainMesh(&mesh);
        splitter.SetCutPathMode(JasMeshMarkAndCutSplit::CUT_PATH_TRIANGLE);
        std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
        splitter.SplitMeshByMarkAndEdge(regions);
        const int liveFaces = countLiveFaces(mesh);
        REQUIRE(liveFaces > 5);
        checkRegionsValid(regions, liveFaces);
        std::cout << "cutPathModeSelect triangle: regions=" << regions.size()
            << " faces=" << liveFaces << std::endl;
    }

    // 多边形路径：不切分任何面（5 面），仅追加 1 个切点孤立顶点；
    // 同样输出完整 retRegs（区域划分与三角形路径一致）。
    {
        CMeshOD mesh;
        makeMesh(mesh);
        JasMeshMarkAndCutSplit splitter;
        splitter.SetMainMesh(&mesh);
        splitter.SetCutPathMode(JasMeshMarkAndCutSplit::CUT_PATH_POLYGON);
        std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
        splitter.SplitMeshByMarkAndEdge(regions);
        const int liveFaces = countLiveFaces(mesh);
        REQUIRE(liveFaces == 5);
        REQUIRE((int)mesh.vert.size() == 7);
        checkRegionsValid(regions, liveFaces);
        std::cout << "cutPathModeSelect polygon: regions=" << regions.size()
            << " faces=" << liveFaces << " verts=" << (int)mesh.vert.size() << std::endl;
    }
    std::cout << "testCutPathModeSelect passed" << std::endl;
}

// 多边形路径跨区域切点共形回归：
//   机制一 —— 区域 A 内的非流形缝（u-v + 翻折面）延长后穿出 A 的边界，
//   穿出点 X 落在 A 与邻域 B 共享的 mark-diff 边 (M0,M1) 内部。该缝只
//   属于 A 的切割边集合，B 不沿此线切割，因此 B 的边界环原本不细分该边
//   （T 形结）；跨区域切点 splice 后，所有触及该边的输出环必须共享同一
//   顶点细分序列。
//   机制二 —— 块 2 中 B 侧也有一条缝（不同高度），两侧各自产生切点，
//   共享边上出现两个切点，双方环都必须同时含两点且顺序一致。
void testPolygonPathSharedEdgeConformity()
{
    // 在 (m0,m1) 线段上收集所有输出边界环的顶点链（按沿线参数排序）；
    // 共形 = 存在全局有序序列 S，使每条链都是 S 的连续子序列
    // （每条链参数序与 S 一致，反向遍历的环排序后同样规范）。
    auto chainsConform = [](const CMeshOD& mesh,
        const std::vector<JasMeshMarkAndCutSplit::splitReg>& regions,
        int m0, int m1)
    {
        const vcg::Point3d p0 = mesh.vert[m0].P();
        const vcg::Point3d p1 = mesh.vert[m1].P();
        const vcg::Point3d d = p1 - p0;
        auto onSegmentParam = [&](int vi, double& tOut)
        {
            const vcg::Point3d ap = mesh.vert[vi].P() - p0;
            const double t = (ap * d) / d.SquaredNorm();
            const vcg::Point3d closest = p0 + d * t;
            tOut = t;
            return (mesh.vert[vi].P() - closest).Norm() < 1e-9 &&
                t >= -1e-9 && t <= 1.0 + 1e-9;
        };
        std::vector<std::vector<std::pair<double, int>>> chains;
        for (const auto& region : regions)
        {
            for (const auto& loop : region.boundaries)
            {
                std::vector<std::pair<double, int>> onSeg;
                for (int vi : loop)
                {
                    double t = 0.0;
                    if (onSegmentParam(vi, t))
                    {
                        onSeg.push_back({ t, vi });
                    }
                }
                if (onSeg.size() >= 2)
                {
                    std::sort(onSeg.begin(), onSeg.end());
                    chains.push_back(std::move(onSeg));
                }
            }
        }
        if (chains.empty())
        {
            return false;
        }
        std::vector<std::pair<double, int>> sequence;
        for (const auto& chain : chains)
        {
            for (const auto& tv : chain)
            {
                bool known = false;
                for (const auto& sv : sequence)
                {
                    if (sv.second == tv.second)
                    {
                        known = true;
                        break;
                    }
                }
                if (!known)
                {
                    sequence.push_back(tv);
                }
            }
        }
        std::sort(sequence.begin(), sequence.end());
        for (const auto& chain : chains)
        {
            int start = -1;
            for (int i = 0; i < (int)sequence.size(); i++)
            {
                if (sequence[i].second == chain.front().second)
                {
                    start = i;
                    break;
                }
            }
            if (start < 0 || start + (int)chain.size() > (int)sequence.size())
            {
                return false;
            }
            for (int k = 0; k < (int)chain.size(); k++)
            {
                if (sequence[start + k].second != chain[k].second)
                {
                    return false;
                }
            }
        }
        return true;
    };
    auto findVertex = [](const CMeshOD& mesh, const vcg::Point3d& p)
    {
        for (int i = 0; i < (int)mesh.vert.size(); i++)
        {
            if (!mesh.vert[i].IsD() && (mesh.vert[i].P() - p).Norm() < 1e-9)
            {
                return i;
            }
        }
        return -1;
    };

    // 块 1（机制一）：A 左侧六三角形 + 缝翻折面（mark=1），B 右侧两三角
    // 形（mark=2）。缝 u-v 端点都在 A 内部 → 双端延长，线 y=0.4 从共享边
    // (M0,M1) 内部 (4,0.4) 穿出；B 无非流形折线，不被切。
    {
        CMeshOD mesh;
        vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 9);
        mesh.vert[0].P() = vcg::Point3d(0, 0, 0);    // L0
        mesh.vert[1].P() = vcg::Point3d(0, 1, 0);    // L1
        mesh.vert[2].P() = vcg::Point3d(1, 0.4, 0);  // u 缝起点（A 内部）
        mesh.vert[3].P() = vcg::Point3d(3, 0.4, 0);  // v 缝终点（A 内部）
        mesh.vert[4].P() = vcg::Point3d(4, 0, 0);    // M0 共享边下端
        mesh.vert[5].P() = vcg::Point3d(4, 1, 0);    // M1 共享边上端
        mesh.vert[6].P() = vcg::Point3d(2, 0.4, 1);  // 翻折面顶点，使 u-v 三面共边
        mesh.vert[7].P() = vcg::Point3d(8, 0, 0);    // R0
        mesh.vert[8].P() = vcg::Point3d(8, 1, 0);    // R1
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[4], &mesh.vert[3]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[3], &mesh.vert[2]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[2], &mesh.vert[1]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[2], &mesh.vert[3]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[5]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[5], &mesh.vert[3]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[3], &mesh.vert[6]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[7], &mesh.vert[8]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[8], &mesh.vert[5]);
        mesh.face.EnableFFAdjacency();
        mesh.face.EnableMark();
        mesh.vert.EnableMark();
        vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
        vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
        for (int fi = 0; fi < 7; fi++)
        {
            mesh.face[fi].IMark() = 1;
        }
        mesh.face[7].IMark() = 2;
        mesh.face[8].IMark() = 2;

        JasMeshMarkAndCutSplit splitter;
        splitter.SetMainMesh(&mesh);
        splitter.SetCutPathMode(JasMeshMarkAndCutSplit::CUT_PATH_POLYGON);
        std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
        splitter.SplitMeshByMarkAndEdge(regions);

        int liveFaces = 0;
        for (const auto& face : mesh.face)
        {
            if (!face.IsD())
            {
                liveFaces++;
            }
        }
        REQUIRE(liveFaces == 9);       // 多边形路径不切分任何面
        REQUIRE(regions.size() == 4);  // A 两片 + 翻折面 + B
        REQUIRE(findVertex(mesh, vcg::Point3d(4, 0.4, 0)) >= 0); // 切点已追加
        REQUIRE(chainsConform(mesh, regions, 4, 5));
        std::cout << "sharedEdgeConformity block1: regions=" << regions.size()
            << " faces=" << liveFaces << std::endl;
    }

    // 块 2（机制二）：B 侧也有自己的缝（y=0.6，翻折面 11），A 侧缝 y=0.4。
    // 两条线各自在共享边 (M0,M1) 上产生切点 (4,0.4) 与 (4,0.6)，四个
    // 分片环都必须按同一顺序细分该边。
    {
        CMeshOD mesh;
        vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 12);
        mesh.vert[0].P() = vcg::Point3d(0, 0, 0);    // L0
        mesh.vert[1].P() = vcg::Point3d(0, 1, 0);    // L1
        mesh.vert[2].P() = vcg::Point3d(1, 0.4, 0);  // u A 侧缝起点
        mesh.vert[3].P() = vcg::Point3d(3, 0.4, 0);  // v A 侧缝终点
        mesh.vert[4].P() = vcg::Point3d(4, 0, 0);    // M0 共享边下端
        mesh.vert[5].P() = vcg::Point3d(4, 1, 0);    // M1 共享边上端
        mesh.vert[6].P() = vcg::Point3d(2, 0.4, 1);  // 翻折面顶点，使 u-v 三面共边
        mesh.vert[7].P() = vcg::Point3d(8, 0, 0);    // R0
        mesh.vert[8].P() = vcg::Point3d(8, 1, 0);    // R1
        mesh.vert[9].P() = vcg::Point3d(5, 0.6, 0);  // u2 B 侧缝起点
        mesh.vert[10].P() = vcg::Point3d(7, 0.6, 0);  // v2 B 侧缝终点
        mesh.vert[11].P() = vcg::Point3d(6, 0.6, 1);  // 翻折面顶点，使 u2-v2 三面共边
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[4], &mesh.vert[3]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[3], &mesh.vert[2]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[2], &mesh.vert[1]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[2], &mesh.vert[3]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[1], &mesh.vert[3], &mesh.vert[5]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[5], &mesh.vert[3]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[2], &mesh.vert[3], &mesh.vert[6]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[7], &mesh.vert[10]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[10], &mesh.vert[9]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[4], &mesh.vert[9], &mesh.vert[5]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[5], &mesh.vert[9], &mesh.vert[10]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[5], &mesh.vert[10], &mesh.vert[8]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[7], &mesh.vert[8], &mesh.vert[10]);
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[9], &mesh.vert[10], &mesh.vert[11]);
        mesh.face.EnableFFAdjacency();
        mesh.face.EnableMark();
        mesh.vert.EnableMark();
        vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);
        vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);
        for (int fi = 0; fi < 7; fi++)
        {
            mesh.face[fi].IMark() = 1;
        }
        for (int fi = 7; fi < 14; fi++)
        {
            mesh.face[fi].IMark() = 2;
        }

        JasMeshMarkAndCutSplit splitter;
        splitter.SetMainMesh(&mesh);
        splitter.SetCutPathMode(JasMeshMarkAndCutSplit::CUT_PATH_POLYGON);
        std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
        splitter.SplitMeshByMarkAndEdge(regions);

        int liveFaces = 0;
        for (const auto& face : mesh.face)
        {
            if (!face.IsD())
            {
                liveFaces++;
            }
        }
        REQUIRE(liveFaces == 14);      // 多边形路径不切分任何面
        REQUIRE(regions.size() == 6);  // A 两片 + 翻折面 + B 两片 + 翻折面
        REQUIRE(findVertex(mesh, vcg::Point3d(4, 0.4, 0)) >= 0);
        REQUIRE(findVertex(mesh, vcg::Point3d(4, 0.6, 0)) >= 0);
        REQUIRE(chainsConform(mesh, regions, 4, 5));
        std::cout << "sharedEdgeConformity block2: regions=" << regions.size()
            << " faces=" << liveFaces << std::endl;
    }
    std::cout << "testPolygonPathSharedEdgeConformity passed" << std::endl;
}

int main() {
    testEdgeHash();
    testBuildEdgeInfo();
    testFindCutEdges();
    testConnectEdgesToPolylines();
    testConnectEdgesToPolylinesMultiple();
    testConnectEdgesToPolylinesDeduplicate();
    testConnectEdgesToPolylinesEmpty();
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
    testGrowNewMark();
    testCutRegionNonManifoldEdgeStability();
    testStitchAllSeams();
    testPrepareLocalCutMarks();
    testCutFacesExact();
    testSplitMeshNormals();
    testCutFacesExactDropRecording();
    testNonManifoldEdgeRegion();
    testSplitRegMarkPreserved();
    testStarVertexSkip();
    testSeamExactDedup();
    testSeamMultiExternal();
    testSeamRedirectAfterSplit();
    testExistingVertexReuse();
    testComplexCutStress();
    testPolygonPathMergeDirect();
    testPolygonPathGridCut();
    testCutPathModeSelect();
    testPolygonPathSharedEdgeConformity();
    return 0;
}
