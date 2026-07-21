// tests/test_mesh_cut.cpp
#include <iostream>
#include <cassert>
#include "tool/edge_info.h"
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

int main() {
    testEdgeHash();
    testBuildEdgeInfo();
    testFindCutEdges();
    return 0;
}
