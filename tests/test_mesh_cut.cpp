// tests/test_mesh_cut.cpp
#include <iostream>
#include <cassert>
#include "tool/edge_info.h"

void testEdgeHash() {
    MeshCutByMark::EdgeHash hash;
    std::pair<int,int> e1 = {1, 2};
    std::pair<int,int> e2 = {2, 1};

    // 相同边应该有相同的哈希值
    assert(hash(e1) == hash(e2));
    std::cout << "testEdgeHash passed" << std::endl;
}

int main() {
    testEdgeHash();
    return 0;
}
