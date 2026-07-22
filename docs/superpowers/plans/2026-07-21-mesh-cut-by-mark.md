# MeshCutByMark 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 MeshCutByMark 功能，将网格按 mark 分割成简单多边形区域

**Architecture:** 使用延长线切割算法，将切割边连接成折线，从端点延长切割，最终通过新标记机制得到简单多边形

**Tech Stack:** C++, VCGlib (MeshLab 核心库)

---

## 文件结构

在开始实现之前，需要创建或修改以下文件：

**新建文件:**
- `tool/edge_info.h` - 边信息数据结构和工具函数
- `tool/polyline.h` - 折线数据结构和连接算法
- `tool/cut_plane.h` - 切割平面构造和三角形切割
- `tool/region_marker.h` - 区域标记和新标记机制
- `tests/test_mesh_cut.cpp` - 单元测试

**修改文件:**
- `JasMeshMarkAndCutSplit.h` - 添加新数据结构和方法声明
- `JasMeshMarkAndCutSplit.cpp` - 实现核心算法

---

## Task 1: 创建边信息数据结构

**Files:**
- Create: `tool/edge_info.h`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 创建边信息头文件**

```cpp
// tool/edge_info.h
#ifndef EDGE_INFO_H
#define EDGE_INFO_H

#include <vector>
#include <unordered_map>
#include <utility>

namespace MeshCutByMark {

// 切割边类型
enum CutEdgeType {
    CUT_EDGE_NONE,           // 普通边（非切割边）
    CUT_EDGE_MARK_DIFF,      // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD,   // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY        // 只被 1 个三角形使用（孔洞边缘）
};

// 切割边信息
struct CutEdge {
    int v0, v1;              // 端点顶点索引 (v0 < v1)
    int faceIdx;             // 所属面索引
    int edgeIdx;             // 边在面中的索引 (0, 1, 2)
    CutEdgeType type;        // 切割边类型
};

// 边的哈希函数
struct EdgeHash {
    size_t operator()(const std::pair<int,int>& e) const {
        return std::hash<int>()(e.first) ^ (std::hash<int>()(e.second) << 1);
    }
};

// 边信息管理器
class EdgeInfoManager {
public:
    // 构建边信息
    void buildEdgeInfo(CMeshOD* mesh);
    
    // 获取边的类型
    CutEdgeType getEdgeType(int v0, int v1) const;
    
    // 获取边的所有邻接面
    std::vector<int> getAdjacentFaces(int v0, int v1) const;
    
    // 检查边是否是切割边
    bool isCutEdge(int v0, int v1) const;
    
private:
    CMeshOD* m_mesh;
    std::unordered_map<std::pair<int,int>, std::vector<int>, EdgeHash> m_edgeToFaces;
    std::unordered_map<std::pair<int,int>, CutEdgeType, EdgeHash> m_edgeTypes;
};

} // namespace MeshCutByMark

#endif // EDGE_INFO_H
```

- [ ] **Step 2: 创建测试文件**

```cpp
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
    std::cout << "✓ testEdgeHash passed" << std::endl;
}

int main() {
    testEdgeHash();
    return 0;
}
```

- [ ] **Step 3: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. tests/test_mesh_cut.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testEdgeHash passed`

- [ ] **Step 4: 提交代码**

```bash
git add tool/edge_info.h tests/test_mesh_cut.cpp
git commit -m "feat: add edge info data structure"
```

---

## Task 2: 实现边信息构建

**Files:**
- Modify: `tool/edge_info.h`
- Modify: `JasMeshMarkAndCutSplit.h`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 实现 EdgeInfoManager::buildEdgeInfo**

```cpp
// 在 tool/edge_info.h 中添加实现
void EdgeInfoManager::buildEdgeInfo(CMeshOD* mesh) {
    m_mesh = mesh;
    m_edgeToFaces.clear();
    m_edgeTypes.clear();
    
    // 遍历所有三角形，建立边→面的映射
    for (int i = 0; i < mesh->face.size(); i++) {
        if (mesh->face[i].IsD()) continue; // 跳过已删除的面
        
        for (int j = 0; j < 3; j++) {
            int v0 = mesh->face[i].V(j)->Index();
            int v1 = mesh->face[i].V((j+1)%3)->Index();
            
            // 确保 v0 < v1
            if (v0 > v1) std::swap(v0, v1);
            
            m_edgeToFaces[{v0, v1}].push_back(i);
        }
    }
    
    // 分类每条边
    for (const auto& [edge, faces] : m_edgeToFaces) {
        CutEdgeType type = CUT_EDGE_NONE;
        
        if (faces.size() == 1) {
            type = CUT_EDGE_BOUNDARY;
        } else if (faces.size() >= 3) {
            type = CUT_EDGE_NON_MANIFOLD;
        } else if (faces.size() == 2) {
            // 检查两个面的 mark 是否相同
            int mark0 = mesh->face[faces[0]].mark;
            int mark1 = mesh->face[faces[1]].mark;
            if (mark0 != mark1) {
                type = CUT_EDGE_MARK_DIFF;
            }
        }
        
        m_edgeTypes[edge] = type;
    }
}
```

- [ ] **Step 2: 在 JasMeshMarkAndCutSplit 中添加边信息管理器**

```cpp
// JasMeshMarkAndCutSplit.h
#include "tool/edge_info.h"

class JasMeshMarkAndCutSplit
{
public:
    // ... 现有代码 ...
    
private:
    CMeshOD* m_pMesh;
    MeshCutByMark::EdgeInfoManager m_edgeInfoManager; // 新增
    int m_newMarkCounter; // 新标记计数器
};
```

- [ ] **Step 3: 添加测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testBuildEdgeInfo() {
    // 创建一个简单的测试网格
    CMeshOD mesh;
    
    // 添加 4 个顶点
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 1, 0));
    auto v3 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 1, 0));
    
    // 添加 2 个三角形
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    auto f1 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v1, v3, v2);
    
    // 设置 mark
    mesh.face[0].mark = 1;
    mesh.face[1].mark = 2;
    
    // 构建边信息
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);
    
    // 验证边 (v1, v2) 是 mark 不同的边
    assert(edgeInfo.getEdgeType(1, 2) == MeshCutByMark::CUT_EDGE_MARK_DIFF);
    
    std::cout << "✓ testBuildEdgeInfo passed" << std::endl;
}
```

- [ ] **Step 4: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testBuildEdgeInfo passed`

- [ ] **Step 5: 提交代码**

```bash
git add tool/edge_info.h JasMeshMarkAndCutSplit.h tests/test_mesh_cut.cpp
git commit -m "feat: implement edge info building"
```

---

## Task 3: 实现切割边查找

**Files:**
- Modify: `JasMeshMarkAndCutSplit.cpp`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 实现 findCutEdges 方法**

```cpp
// JasMeshMarkAndCutSplit.cpp
std::vector<MeshCutByMark::CutEdge> JasMeshMarkAndCutSplit::findCutEdges(const std::vector<int>& curFaces) {
    std::vector<MeshCutByMark::CutEdge> cutEdges;
    
    for (int faceIdx : curFaces) {
        for (int j = 0; j < 3; j++) {
            int v0 = m_pMesh->face[faceIdx].V(j)->Index();
            int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();
            
            if (v0 > v1) std::swap(v0, v1);
            
            MeshCutByMark::CutEdgeType type = m_edgeInfoManager.getEdgeType(v0, v1);
            
            if (type != MeshCutByMark::CUT_EDGE_NONE) {
                MeshCutByMark::CutEdge edge;
                edge.v0 = v0;
                edge.v1 = v1;
                edge.faceIdx = faceIdx;
                edge.edgeIdx = j;
                edge.type = type;
                cutEdges.push_back(edge);
            }
        }
    }
    
    return cutEdges;
}
```

- [ ] **Step 2: 添加测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testFindCutEdges() {
    // 创建一个测试网格，包含 3 个三角形
    CMeshOD mesh;
    
    // 添加顶点
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 1, 0));
    auto v3 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 1, 0));
    
    // 添加三角形
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    auto f1 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v1, v3, v2);
    auto f2 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v2, v3); // 非流形边
    
    // 设置 mark
    mesh.face[0].mark = 1;
    mesh.face[1].mark = 1;
    mesh.face[2].mark = 1;
    
    // 构建边信息
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);
    
    // 查找切割边
    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    
    std::vector<int> curFaces = {0, 1, 2};
    auto cutEdges = splitter.findCutEdges(curFaces);
    
    // 验证找到非流形边
    bool foundNonManifold = false;
    for (const auto& edge : cutEdges) {
        if (edge.type == MeshCutByMark::CUT_EDGE_NON_MANIFOLD) {
            foundNonManifold = true;
            break;
        }
    }
    
    assert(foundNonManifold);
    std::cout << "✓ testFindCutEdges passed" << std::endl;
}
```

- [ ] **Step 3: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testFindCutEdges passed`

- [ ] **Step 4: 提交代码**

```bash
git add JasMeshMarkAndCutSplit.cpp tests/test_mesh_cut.cpp
git commit -m "feat: implement findCutEdges"
```

---

## Task 4: 实现折线连接

**Files:**
- Create: `tool/polyline.h`
- Modify: `JasMeshMarkAndCutSplit.h`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 创建折线数据结构**

```cpp
// tool/polyline.h
#ifndef POLYLINE_H
#define POLYLINE_H

#include <vector>
#include <unordered_map>
#include "edge_info.h"

namespace MeshCutByMark {

// 折线
struct Polyline {
    std::vector<int> vertexIndices;  // 折线的顶点序列
    int startFaceIdx;                // 首端点所在的面
    int startEdgeIdx;                // 首端点所在的边索引
    int endFaceIdx;                  // 尾端点所在的面
    int endEdgeIdx;                  // 尾端点所在的边索引
};

// 折线管理器
class PolylineManager {
public:
    // 将切割边连接成连续折线
    std::vector<Polyline> connectEdgesToPolylines(
        const std::vector<CutEdge>& cutEdges,
        CMeshOD* mesh
    );
    
private:
    // 构建端点→边的映射
    std::unordered_map<int, std::vector<int>> buildVertexToEdgesMap(
        const std::vector<CutEdge>& cutEdges
    );
    
    // 向一个方向扩展折线
    void extendPolyline(
        Polyline& polyline,
        int& currentVertex,
        const std::vector<CutEdge>& cutEdges,
        const std::unordered_map<int, std::vector<int>>& vertexToEdges,
        std::vector<bool>& used,
        bool forward
    );
};

} // namespace MeshCutByMark

#endif // POLYLINE_H
```

- [ ] **Step 2: 实现折线连接算法**

```cpp
// 在 tool/polyline.h 中添加实现
std::vector<Polyline> PolylineManager::connectEdgesToPolylines(
    const std::vector<CutEdge>& cutEdges,
    CMeshOD* mesh
) {
    std::vector<Polyline> polylines;
    
    if (cutEdges.empty()) return polylines;
    
    // 构建端点→边的映射
    auto vertexToEdges = buildVertexToEdgesMap(cutEdges);
    
    // 标记已使用的边
    std::vector<bool> used(cutEdges.size(), false);
    
    for (int i = 0; i < cutEdges.size(); i++) {
        if (used[i]) continue;
        
        Polyline polyline;
        used[i] = true;
        
        // 从这条边开始
        int startV = cutEdges[i].v0;
        int endV = cutEdges[i].v1;
        polyline.vertexIndices.push_back(startV);
        polyline.vertexIndices.push_back(endV);
        
        // 记录首尾端点信息
        polyline.startFaceIdx = cutEdges[i].faceIdx;
        polyline.startEdgeIdx = cutEdges[i].edgeIdx;
        polyline.endFaceIdx = cutEdges[i].faceIdx;
        polyline.endEdgeIdx = cutEdges[i].edgeIdx;
        
        // 向 startV 方向扩展
        extendPolyline(polyline, startV, cutEdges, vertexToEdges, used, false);
        
        // 向 endV 方向扩展
        extendPolyline(polyline, endV, cutEdges, vertexToEdges, used, true);
        
        polylines.push_back(polyline);
    }
    
    return polylines;
}

void PolylineManager::extendPolyline(
    Polyline& polyline,
    int& currentVertex,
    const std::vector<CutEdge>& cutEdges,
    const std::unordered_map<int, std::vector<int>>& vertexToEdges,
    std::vector<bool>& used,
    bool forward
) {
    while (true) {
        bool found = false;
        
        auto it = vertexToEdges.find(currentVertex);
        if (it == vertexToEdges.end()) break;
        
        for (int edgeIdx : it->second) {
            if (used[edgeIdx]) continue;
            
            used[edgeIdx] = true;
            
            int otherV = (cutEdges[edgeIdx].v0 == currentVertex) ? 
                         cutEdges[edgeIdx].v1 : 
                         cutEdges[edgeIdx].v0;
            
            if (forward) {
                polyline.vertexIndices.push_back(otherV);
                polyline.endFaceIdx = cutEdges[edgeIdx].faceIdx;
                polyline.endEdgeIdx = cutEdges[edgeIdx].edgeIdx;
            } else {
                polyline.vertexIndices.insert(polyline.vertexIndices.begin(), otherV);
                polyline.startFaceIdx = cutEdges[edgeIdx].faceIdx;
                polyline.startEdgeIdx = cutEdges[edgeIdx].edgeIdx;
            }
            
            currentVertex = otherV;
            found = true;
            break;
        }
        
        if (!found) break;
    }
}
```

- [ ] **Step 3: 添加测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testConnectEdgesToPolylines() {
    // 创建测试切割边
    std::vector<MeshCutByMark::CutEdge> cutEdges;
    
    // 边 1: v0-v1
    cutEdges.push_back({0, 1, 0, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});
    // 边 2: v1-v2
    cutEdges.push_back({1, 2, 1, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});
    // 边 3: v2-v3
    cutEdges.push_back({2, 3, 2, 0, MeshCutByMark::CUT_EDGE_MARK_DIFF});
    
    MeshCutByMark::PolylineManager polylineManager;
    CMeshOD mesh; // 空网格，仅用于测试
    
    auto polylines = polylineManager.connectEdgesToPolylines(cutEdges, &mesh);
    
    // 验证连接成一条折线
    assert(polylines.size() == 1);
    assert(polylines[0].vertexIndices.size() == 4);
    assert(polylines[0].vertexIndices[0] == 0);
    assert(polylines[0].vertexIndices[3] == 3);
    
    std::cout << "✓ testConnectEdgesToPolylines passed" << std::endl;
}
```

- [ ] **Step 4: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testConnectEdgesToPolylines passed`

- [ ] **Step 5: 提交代码**

```bash
git add tool/polyline.h tests/test_mesh_cut.cpp
git commit -m "feat: implement polyline connection"
```

---

## Task 5: 实现切割平面构造

**Files:**
- Create: `tool/cut_plane.h`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 创建切割平面头文件**

```cpp
// tool/cut_plane.h
#ifndef CUT_PLANE_H
#define CUT_PLANE_H

#include <vcg/space/plane3.h>
#include <vcg/space/point3.h>
#include "polyline.h"

namespace MeshCutByMark {

// 切割平面管理器
class CutPlaneManager {
public:
    // 构造切割平面
    vcg::Plane3d makeCutPlane(
        const Polyline& polyline,
        bool isStart,
        CMeshOD* mesh
    );
    
    // 用平面切割三角形
    void cutTriangleByPlane(
        int faceIdx,
        const vcg::Plane3d& plane,
        CMeshOD* mesh
    );
    
    // 检查端点是否在 mark 不同的边上
    bool isOnMarkDiffEdge(
        int faceIdx,
        int edgeIdx,
        CMeshOD* mesh
    );
    
private:
    // 计算有符号距离
    double signedDistance(const vcg::Point3d& point, const vcg::Plane3d& plane);
    
    // 计算线段与平面的交点
    vcg::Point3d intersectSegmentPlane(
        const vcg::Point3d& p0,
        const vcg::Point3d& p1,
        double d0,
        double d1,
        const vcg::Plane3d& plane
    );
};

} // namespace MeshCutByMark

#endif // CUT_PLANE_H
```

- [ ] **Step 2: 实现切割平面构造**

```cpp
// 在 tool/cut_plane.h 中添加实现
vcg::Plane3d CutPlaneManager::makeCutPlane(
    const Polyline& polyline,
    bool isStart,
    CMeshOD* mesh
) {
    // 获取端点信息
    int vertexIdx = isStart ? polyline.vertexIndices[0] : polyline.vertexIndices.back();
    int faceIdx = isStart ? polyline.startFaceIdx : polyline.endFaceIdx;
    
    // 获取折线方向
    vcg::Point3d dir;
    if (isStart) {
        vcg::Point3d v0 = mesh->vert[polyline.vertexIndices[0]].P();
        vcg::Point3d v1 = mesh->vert[polyline.vertexIndices[1]].P();
        dir = v0 - v1;  // 从 v1 指向 v0（向外延伸）
    } else {
        int n = polyline.vertexIndices.size();
        vcg::Point3d v0 = mesh->vert[polyline.vertexIndices[n-2]].P();
        vcg::Point3d v1 = mesh->vert[polyline.vertexIndices[n-1]].P();
        dir = v1 - v0;  // 从 v0 指向 v1（向外延伸）
    }
    dir.Normalize();
    
    // 获取三角形法向量
    vcg::Point3d N = mesh->face[faceIdx].N();
    
    // 切割平面法向量（垂直于边和法向量）
    vcg::Point3d C = dir ^ N;  // 叉积
    C.Normalize();
    
    // 构造平面（过端点，法向量为 C）
    vcg::Plane3d plane;
    plane.Init(mesh->vert[vertexIdx].P(), C);
    
    return plane;
}

double CutPlaneManager::signedDistance(const vcg::Point3d& point, const vcg::Plane3d& plane) {
    return vcg::SignedDistancePlanePoint(plane, point);
}

vcg::Point3d CutPlaneManager::intersectSegmentPlane(
    const vcg::Point3d& p0,
    const vcg::Point3d& p1,
    double d0,
    double d1,
    const vcg::Plane3d& plane
) {
    // 线段与平面求交
    double t = d0 / (d0 - d1);
    return p0 + (p1 - p0) * t;
}
```

- [ ] **Step 3: 添加测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testMakeCutPlane() {
    // 创建测试折线
    MeshCutByMark::Polyline polyline;
    polyline.vertexIndices = {0, 1, 2};
    polyline.startFaceIdx = 0;
    polyline.startEdgeIdx = 0;
    polyline.endFaceIdx = 1;
    polyline.endEdgeIdx = 0;
    
    // 创建测试网格
    CMeshOD mesh;
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(2, 0, 0));
    
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    mesh.face[0].N() = vcg::Point3d(0, 0, 1); // 法向量
    
    MeshCutByMark::CutPlaneManager cutPlaneManager;
    auto plane = cutPlaneManager.makeCutPlane(polyline, true, &mesh);
    
    // 验证平面不为空
    assert(plane.Direction().Norm() > 0.99);
    
    std::cout << "✓ testMakeCutPlane passed" << std::endl;
}
```

- [ ] **Step 4: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testMakeCutPlane passed`

- [ ] **Step 5: 提交代码**

```bash
git add tool/cut_plane.h tests/test_mesh_cut.cpp
git commit -m "feat: implement cut plane construction"
```

---

## Task 6: 实现区域标记

**Files:**
- Create: `tool/region_marker.h`
- Modify: `JasMeshMarkAndCutSplit.h`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 创建区域标记头文件**

```cpp
// tool/region_marker.h
#ifndef REGION_MARKER_H
#define REGION_MARKER_H

#include <vector>
#include <queue>
#include "edge_info.h"

namespace MeshCutByMark {

// 区域标记管理器
class RegionMarker {
public:
    // 初始化 newMark 属性
    void initNewMark(CMeshOD* mesh);
    
    // flood-fill 找连通区域
    std::vector<int> floodFill(
        int startFaceIdx,
        int targetMark,
        CMeshOD* mesh,
        const EdgeInfoManager& edgeInfo
    );
    
    // 拣选子区域（切割后）
    std::vector<std::vector<int>> extractSubRegions(
        const std::vector<int>& curFaces,
        CMeshOD* mesh
    );
    
    // 标记子区域
    void markSubRegions(
        const std::vector<std::vector<int>>& subRegions,
        CMeshOD* mesh,
        int& newMarkCounter
    );
    
private:
    // 检查边是否是切割边
    bool isCutEdge(int faceIdx, int edgeIdx, CMeshOD* mesh);
    
    // 检查面是否在 curFaces 中
    bool isInCurFaces(int faceIdx, const std::vector<int>& curFaces);
};

} // namespace MeshCutByMark

#endif // REGION_MARKER_H
```

- [ ] **Step 2: 实现区域标记算法**

```cpp
// 在 tool/region_marker.h 中添加实现
void RegionMarker::initNewMark(CMeshOD* mesh) {
    for (int i = 0; i < mesh->face.size(); i++) {
        if (!mesh->face[i].IsD()) {
            mesh->face[i].newMark = 0;
        }
    }
}

std::vector<int> RegionMarker::floodFill(
    int startFaceIdx,
    int targetMark,
    CMeshOD* mesh,
    const EdgeInfoManager& edgeInfo
) {
    std::vector<int> result;
    std::queue<int> queue;
    
    std::vector<bool> visited(mesh->face.size(), false);
    queue.push(startFaceIdx);
    visited[startFaceIdx] = true;
    
    while (!queue.empty()) {
        int faceIdx = queue.front();
        queue.pop();
        result.push_back(faceIdx);
        
        // 遍历三条边
        for (int j = 0; j < 3; j++) {
            // 跳过切割边
            if (isCutEdge(faceIdx, j, mesh))
                continue;
            
            // 获取邻接面
            int adjFaceIdx = -1;
            if (mesh->face[faceIdx].HasFFAdjacency()) {
                adjFaceIdx = mesh->face[faceIdx].FFp(j) - &mesh->face[0];
            }
            
            // 检查邻接面是否有效
            if (adjFaceIdx < 0 || visited[adjFaceIdx])
                continue;
            
            // 检查 mark 是否相同
            if (mesh->face[adjFaceIdx].mark != targetMark)
                continue;
            
            visited[adjFaceIdx] = true;
            queue.push(adjFaceIdx);
        }
    }
    
    return result;
}

std::vector<std::vector<int>> RegionMarker::extractSubRegions(
    const std::vector<int>& curFaces,
    CMeshOD* mesh
) {
    std::vector<std::vector<int>> subRegions;
    
    for (int faceIdx : curFaces) {
        if (mesh->face[faceIdx].newMark > 0)
            continue;  // 已标记
        
        // 从这个面开始 flood-fill
        std::vector<int> region;
        std::queue<int> queue;
        queue.push(faceIdx);
        mesh->face[faceIdx].newMark = -1;  // 临时标记为正在处理
        
        while (!queue.empty()) {
            int curFace = queue.front();
            queue.pop();
            region.push_back(curFace);
            
            // 遍历三条边
            for (int j = 0; j < 3; j++) {
                // 跳过切割边
                if (isCutEdge(curFace, j, mesh))
                    continue;
                
                // 获取邻接面
                int adjFaceIdx = -1;
                if (mesh->face[curFace].HasFFAdjacency()) {
                    adjFaceIdx = mesh->face[curFace].FFp(j) - &mesh->face[0];
                }
                
                if (adjFaceIdx < 0 || mesh->face[adjFaceIdx].newMark != 0)
                    continue;
                
                // 检查是否还在 curFaces 中
                if (!isInCurFaces(adjFaceIdx, curFaces))
                    continue;
                
                mesh->face[adjFaceIdx].newMark = -1;
                queue.push(adjFaceIdx);
            }
        }
        
        subRegions.push_back(region);
    }
    
    return subRegions;
}

void RegionMarker::markSubRegions(
    const std::vector<std::vector<int>>& subRegions,
    CMeshOD* mesh,
    int& newMarkCounter
) {
    for (const auto& region : subRegions) {
        for (int faceIdx : region) {
            mesh->face[faceIdx].newMark = newMarkCounter;
        }
        newMarkCounter++;
    }
}
```

- [ ] **Step 3: 添加测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testFloodFill() {
    CMeshOD mesh;
    
    // 创建 3 个三角形
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 1, 0));
    auto v3 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 1, 0));
    
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    auto f1 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v1, v3, v2);
    
    mesh.face[0].mark = 1;
    mesh.face[1].mark = 1;
    
    // 构建边信息
    MeshCutByMark::EdgeInfoManager edgeInfo;
    edgeInfo.buildEdgeInfo(&mesh);
    
    // 测试 flood-fill
    MeshCutByMark::RegionMarker regionMarker;
    auto result = regionMarker.floodFill(0, 1, &mesh, edgeInfo);
    
    // 验证找到所有三角形
    assert(result.size() == 2);
    
    std::cout << "✓ testFloodFill passed" << std::endl;
}
```

- [ ] **Step 4: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testFloodFill passed`

- [ ] **Step 5: 提交代码**

```bash
git add tool/region_marker.h tests/test_mesh_cut.cpp
git commit -m "feat: implement region marking"
```

---

## Task 7: 实现边界边提取

**Files:**
- Modify: `JasMeshMarkAndCutSplit.cpp`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 实现 extractBoundaryEdges 方法**

```cpp
// JasMeshMarkAndCutSplit.cpp
std::vector<std::vector<int>> JasMeshMarkAndCutSplit::extractBoundaryEdges(
    const std::vector<int>& regionFaces
) {
    std::vector<std::vector<int>> boundaries;
    
    // 找到所有边界边（只被一个面使用的边）
    std::vector<std::pair<int,int>> boundaryEdges;
    std::unordered_map<std::pair<int,int>, int, MeshCutByMark::EdgeHash> edgeCount;
    
    for (int faceIdx : regionFaces) {
        for (int j = 0; j < 3; j++) {
            int v0 = m_pMesh->face[faceIdx].V(j)->Index();
            int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();
            
            if (v0 > v1) std::swap(v0, v1);
            
            edgeCount[{v0, v1}]++;
        }
    }
    
    // 只保留边界边
    for (const auto& [edge, count] : edgeCount) {
        if (count == 1) {
            boundaryEdges.push_back(edge);
        }
    }
    
    // 构建边界边的邻接关系
    std::unordered_map<int, std::vector<int>> vertexToEdges;
    for (int i = 0; i < boundaryEdges.size(); i++) {
        vertexToEdges[boundaryEdges[i].first].push_back(i);
        vertexToEdges[boundaryEdges[i].second].push_back(i);
    }
    
    // 沿边界遍历形成闭合环
    std::vector<bool> used(boundaryEdges.size(), false);
    for (int i = 0; i < boundaryEdges.size(); i++) {
        if (used[i]) continue;
        
        std::vector<int> boundary;
        int startV = boundaryEdges[i].first;
        int curV = boundaryEdges[i].second;
        boundary.push_back(startV);
        used[i] = true;
        
        while (curV != startV) {
            boundary.push_back(curV);
            
            // 找到下一条边界边
            bool found = false;
            for (int edgeIdx : vertexToEdges[curV]) {
                if (!used[edgeIdx]) {
                    used[edgeIdx] = true;
                    curV = (boundaryEdges[edgeIdx].first == curV) ? 
                           boundaryEdges[edgeIdx].second : 
                           boundaryEdges[edgeIdx].first;
                    found = true;
                    break;
                }
            }
            
            if (!found) break; // 异常情况
        }
        
        if (boundary.size() >= 3) {
            boundaries.push_back(boundary);
        }
    }
    
    return boundaries;
}
```

- [ ] **Step 2: 添加测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testExtractBoundaryEdges() {
    CMeshOD mesh;
    
    // 创建一个三角形
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 1, 0));
    
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    
    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    
    std::vector<int> regionFaces = {0};
    auto boundaries = splitter.extractBoundaryEdges(regionFaces);
    
    // 验证找到 1 个边界（三角形有 3 条边界边）
    assert(boundaries.size() == 1);
    assert(boundaries[0].size() == 3);
    
    std::cout << "✓ testExtractBoundaryEdges passed" << std::endl;
}
```

- [ ] **Step 3: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testExtractBoundaryEdges passed`

- [ ] **Step 4: 提交代码**

```bash
git add JasMeshMarkAndCutSplit.cpp tests/test_mesh_cut.cpp
git commit -m "feat: implement boundary edge extraction"
```

---

## Task 8: 实现主算法 SplitMeshByMarkAndEdge

**Files:**
- Modify: `JasMeshMarkAndCutSplit.cpp`
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 实现主算法**

```cpp
// JasMeshMarkAndCutSplit.cpp
void JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs) {
    // 初始化新标记
    m_newMarkCounter = 1;
    m_regionMarker.initNewMark(m_pMesh);
    
    // 构建边信息
    m_edgeInfoManager.buildEdgeInfo(m_pMesh);
    
    // Phase 2: 延长线切割并标记新区域
    for (int i = 0; i < m_pMesh->face.size(); i++) {
        if (m_pMesh->face[i].IsD()) continue;
        if (m_pMesh->face[i].newMark > 0) continue;  // 已处理
        
        // 2.1 flood-fill 找连通区域
        int targetMark = m_pMesh->face[i].mark;
        std::vector<int> curFaces = m_regionMarker.floodFill(i, targetMark, m_pMesh, m_edgeInfoManager);
        
        // 2.2 找 curFaces 的切割边
        std::vector<MeshCutByMark::CutEdge> cutEdges = findCutEdges(curFaces);
        
        // 2.3 将切割边连接成连续折线
        std::vector<MeshCutByMark::Polyline> polylines = 
            m_polylineManager.connectEdgesToPolylines(cutEdges, m_pMesh);
        
        // 2.4 从端点延长切割
        for (const auto& polyline : polylines) {
            // 检查首端点
            if (!m_cutPlaneManager.isOnMarkDiffEdge(
                    polyline.startFaceIdx, polyline.startEdgeIdx, m_pMesh)) {
                vcg::Plane3d plane = m_cutPlaneManager.makeCutPlane(polyline, true, m_pMesh);
                // 切割 curFaces 中的所有三角形
                for (int faceIdx : curFaces) {
                    if (!m_pMesh->face[faceIdx].IsD()) {
                        m_cutPlaneManager.cutTriangleByPlane(faceIdx, plane, m_pMesh);
                    }
                }
            }
            
            // 检查尾端点
            if (!m_cutPlaneManager.isOnMarkDiffEdge(
                    polyline.endFaceIdx, polyline.endEdgeIdx, m_pMesh)) {
                vcg::Plane3d plane = m_cutPlaneManager.makeCutPlane(polyline, false, m_pMesh);
                // 切割 curFaces 中的所有三角形
                for (int faceIdx : curFaces) {
                    if (!m_pMesh->face[faceIdx].IsD()) {
                        m_cutPlaneManager.cutTriangleByPlane(faceIdx, plane, m_pMesh);
                    }
                }
            }
        }
        
        // 2.5 通过拣选得到切割后的子区域
        std::vector<std::vector<int>> subRegions = 
            m_regionMarker.extractSubRegions(curFaces, m_pMesh);
        
        // 2.6 每个子区域标记新 mark
        m_regionMarker.markSubRegions(subRegions, m_pMesh, m_newMarkCounter);
    }
    
    // Phase 3: 根据新标记提取多边形
    std::map<int, std::vector<int>> markToFaces;
    for (int i = 0; i < m_pMesh->face.size(); i++) {
        if (!m_pMesh->face[i].IsD()) {
            markToFaces[m_pMesh->face[i].newMark].push_back(i);
        }
    }
    
    // 输出结果
    for (const auto& [newMark, faces] : markToFaces) {
        // 提取边界边
        std::vector<std::vector<int>> boundaries = extractBoundaryEdges(faces);
        
        // 构造 splitReg
        splitReg reg;
        reg.mark = m_pMesh->face[faces[0]].mark;
        reg.newMark = newMark;
        reg.inTris = faces;
        reg.normal = m_pMesh->face[faces[0]].N();
        reg.boundlines = boundaries.empty() ? std::vector<int>() : boundaries[0];
        // TODO: 处理内边界（孔）
        
        retRegs.push_back(reg);
    }
}
```

- [ ] **Step 2: 添加完整测试用例**

```cpp
// tests/test_mesh_cut.cpp
void testSplitMeshByMarkAndEdge() {
    CMeshOD mesh;
    
    // 创建一个简单的测试网格
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 1, 0));
    auto v3 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 1, 0));
    
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    auto f1 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v1, v3, v2);
    
    mesh.face[0].mark = 1;
    mesh.face[1].mark = 2;
    
    // 测试分割
    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    
    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);
    
    // 验证输出
    assert(regions.size() == 2);
    
    std::cout << "✓ testSplitMeshByMarkAndEdge passed" << std::endl;
}
```

- [ ] **Step 3: 编译并运行测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: `✓ testSplitMeshByMarkAndEdge passed`

- [ ] **Step 4: 提交代码**

```bash
git add JasMeshMarkAndCutSplit.cpp tests/test_mesh_cut.cpp
git commit -m "feat: implement SplitMeshByMarkAndEdge main algorithm"
```

---

## Task 9: 集成测试

**Files:**
- Test: `tests/test_mesh_cut.cpp`

- [ ] **Step 1: 创建集成测试**

```cpp
// tests/test_mesh_cut.cpp
void testIntegration() {
    // 创建一个更复杂的测试网格
    CMeshOD mesh;
    
    // 添加顶点
    auto v0 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 0, 0));
    auto v1 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 0, 0));
    auto v2 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(0, 1, 0));
    auto v3 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(1, 1, 0));
    auto v4 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(2, 0, 0));
    auto v5 = vcg::tri::Allocator<CMeshOD>::AddVertex(mesh, vcg::Point3d(2, 1, 0));
    
    // 添加三角形
    auto f0 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v0, v1, v2);
    auto f1 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v1, v3, v2);
    auto f2 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v1, v4, v3);
    auto f3 = vcg::tri::Allocator<CMeshOD>::AddFace(mesh, v4, v5, v3);
    
    // 设置 mark
    mesh.face[0].mark = 1;
    mesh.face[1].mark = 1;
    mesh.face[2].mark = 2;
    mesh.face[3].mark = 2;
    
    // 测试分割
    JasMeshMarkAndCutSplit splitter;
    splitter.SetMainMesh(&mesh);
    
    std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
    splitter.SplitMeshByMarkAndEdge(regions);
    
    // 验证输出
    assert(regions.size() == 2);  // 两个 mark
    
    // 验证每个区域的三角形数量
    int totalFaces = 0;
    for (const auto& reg : regions) {
        totalFaces += reg.inTris.size();
    }
    assert(totalFaces == 4);  // 所有三角形都被分配
    
    std::cout << "✓ testIntegration passed" << std::endl;
}
```

- [ ] **Step 2: 运行所有测试**

```bash
cd d:\claudecode\MeshCutByLine
g++ -std=c++17 -I. -Ivcglib tests/test_mesh_cut.cpp JasMeshMarkAndCutSplit.cpp -o tests/test_mesh_cut
./tests/test_mesh_cut
```

Expected: 所有测试通过

- [ ] **Step 3: 提交代码**

```bash
git add tests/test_mesh_cut.cpp
git commit -m "test: add integration test"
```

---

## Task 10: 文档和清理

**Files:**
- Modify: `README.md` (if exists)

- [ ] **Step 1: 更新头文件注释**

```cpp
// JasMeshMarkAndCutSplit.h
/**
 * @brief MeshCutByMark - 将网格按 mark 分割成简单多边形
 * 
 * 核心功能：
 * 1. 按 mark 分组三角形
 * 2. 找到切割边（mark 不同、非流形、边界）
 * 3. 将切割边连接成折线
 * 4. 从端点延长切割
 * 5. 通过新标记机制得到简单多边形
 * 
 * 输入：带有 mark 属性的三角形网格
 * 输出：简单多边形区域列表
 */
```

- [ ] **Step 2: 提交最终代码**

```bash
git add -A
git commit -m "docs: add comments and finalize implementation"
```

---

## 自检清单

完成所有任务后，检查以下内容：

- [ ] 所有测试通过
- [ ] 代码编译无警告
- [ ] 边信息正确构建
- [ ] 折线正确连接
- [ ] 切割平面正确构造
- [ ] 区域标记正确
- [ ] 边界边正确提取
- [ ] 输出格式符合要求
- [ ] 简单多边形验证通过

---

## 执行选项

计划完成并保存到 `docs/superpowers/plans/2026-07-21-mesh-cut-by-mark.md`。

**两种执行方式：**

**1. Subagent-Driven（推荐）** - 每个任务分发一个新的 subagent，任务之间进行 review，快速迭代

**2. Inline Execution** - 在当前会话中执行任务，批量执行并设置检查点

**选择哪种方式？**
