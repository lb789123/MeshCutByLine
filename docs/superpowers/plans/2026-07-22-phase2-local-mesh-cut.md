# Phase 2.4 局部 mesh 切割改造 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用「提取局部 mesh → 调用 `JasMeshAddCutLines::AddCutLines` → 合并回主网格」的管线替换 Phase 2.4 的 `cutTriangleByPlane` 存根路径。

**Architecture:** 新建 `tool/local_mesh_cut.h` 中的 `LocalMeshCutManager`，把 A(提取局部 mesh)、B(构造 line+normal)、C(merge 回)、D(标分割边)、E(外部加点)、F(收尾) 拆成可单测的子步骤；`JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge` 的 Phase 2.4 改为调用 `cutRegion()`。cutter 为外部稳定黑盒，不在本仓实现。

**Tech Stack:** C++17、VCGlib（vendored）、MSVC `/utf-8`、raw `assert` 测试。

## Global Constraints

- C++17，MSVC `/utf-8`（源码含中文注释）。
- 所有 tool 头文件 header-only（inline 实现），命名空间 `MeshCutByMark`（`JasMeshAddCutLines` 是全局类，不在该命名空间）。
- 标量类型 `double`（`Point3m`=`vcg::Point3d`，`Plane3m`=`vcg::Plane3d`）。
- 测试无框架：`tests/test_mesh_cut.cpp` 里每个测试函数用 `assert`，末尾打印 `testX passed`；在 `main()` 里按序调用。新增测试必须加进 `main()`。
- 构建/测试（假设 `build/` 已 configure）：
  - 构建：`cmake --build build --config Release`
  - 运行：`./build/Release/test_mesh_cut.exe`（PowerShell 用 `.\build\Release\test_mesh_cut.exe`）
  - 全量 configure（首次）：`cmake -G "Visual Studio 17 2022" -A x64 -B build -S .`
- 新面判定靠「引用了新顶点（local 下标 ≥ Nv0）」，不靠下标范围；新面→被分裂原始面的映射**靠几何反推**：在 `extractLocalMesh` 用未改动的 `m_pMesh` 建 `globalEdgeToCurFace`（原始面的 global 顶点对→curFaces 面），新面里两个原顶点的 global 对查此表即得被分裂原始面。**不依赖** cutter 写任何属性（自包含、抗 cutter 原地改写）。

## File Structure

- **Create** `tool/local_mesh_cut.h` — `MeshCutByMark::LocalMeshCutManager`：A~F 全部子步骤 + `cutRegion()` 总装。header-only。
- **Modify** `tool/region_marker.h` — 新增 `growNewMark(size_t)`（resize `m_newMark`，新元素置 0，不重置已有）。
- **Modify** `JasMeshMarkAndCutSplit.h` — include `tool/local_mesh_cut.h`，加成员 `MeshCutByMark::LocalMeshCutManager m_localMeshCut;`。
- **Modify** `JasMeshMarkAndCutSplit.cpp` — Phase 2.4（399-427 行那段 for 循环）替换为 `m_localMeshCut.cutRegion(...)`。
- **Modify** `tests/test_mesh_cut.cpp` — 为每个子步骤加单测 + 1 个 plumbing 冒烟测试；在 `main()` 注册。
- **Modify** `tool/cut_mesh.h` — 保持接口不变；测试期在 test TU 内提供 `AddCutLines` 桩（见 Task 7）。

---

## Task 1: 局部 mesh 提取（步骤 A）

**Files:**
- Create: `tool/local_mesh_cut.h`
- Test: `tests/test_mesh_cut.cpp`

**Interfaces:**
- Consumes: `CMeshOD* mesh`、`const std::vector<int>& curFaces`。
- Produces: `LocalMeshCutManager::LocalMesh` 结构（含 `CMeshOD mesh`、`int Nv0`、`localToGlobalVert`、`localFaceToGlobal`、`seamExternal`）。

- [ ] **Step 1: 写失败测试**

追加到 `tests/test_mesh_cut.cpp`（在 `testExtractSubRegions` 之前即可）：

```cpp
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
    auto lm = mgr.extractLocalMesh(&mesh, curFaces);

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
```

并在 `main()` 末尾（`return 0;` 之前）加 `testExtractLocalMesh();`。

- [ ] **Step 2: 运行测试确认失败**

构建并运行：`cmake --build build --config Release && ./build/Release/test_mesh_cut.exe`
Expected: 编译失败（`LocalMeshCutManager` 未定义）。

- [ ] **Step 3: 写最小实现**

创建 `tool/local_mesh_cut.h`：

```cpp
// tool/local_mesh_cut.h
#ifndef LOCAL_MESH_CUT_H
#define LOCAL_MESH_CUT_H

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <algorithm>
#include "cmesh.h"
#include "polyline.h"
#include "region_marker.h"
#include "cut_mesh.h"

namespace MeshCutByMark {

class LocalMeshCutManager {
public:
    // 局部 mesh + 各种映射
    struct LocalMesh {
        CMeshOD mesh;
        int Nv0 = 0;                                  // 原顶点数（新顶点 local 下标 >= Nv0）
        std::vector<int> localToGlobalVert;           // localVertIdx -> globalVertIdx（仅 < Nv0）
        std::vector<int> localFaceToGlobal;           // localFaceIdx -> globalFaceIdx（仅原始面）
        // 边界缝：local 原始边的两个 local 顶点 -> 外部邻接面 global idx
        std::map<std::pair<int,int>, int> seamExternal;
        // 原始面的 global 顶点对 -> 该 curFaces 面 global idx（用未改动的 m_pMesh 建表，反推来源用）
        std::map<std::pair<int,int>, int> globalEdgeToCurFace;
    };

    // 步骤 A：从 m_pMesh 的 curFaces 提取局部 mesh
    LocalMesh extractLocalMesh(CMeshOD* mesh, const std::vector<int>& curFaces);

    // （后续 Task 实现）
    // void buildCutInput(...) / mergeBack(...) / markCutEdges(...) / propagateExternal(...) / cutRegion(...)
};

inline LocalMeshCutManager::LocalMesh LocalMeshCutManager::extractLocalMesh(
    CMeshOD* mesh, const std::vector<int>& curFaces)
{
    LocalMesh lm;
    lm.localToGlobalVert.clear();
    lm.localFaceToGlobal = curFaces;  // local face i <-> global curFaces[i]

    // 1) 收集去重顶点，建映射
    std::map<int,int> globalToLocal;
    for (int gf : curFaces) {
        for (int j = 0; j < 3; j++) {
            int gv = mesh->face[gf].V(j)->Index();
            if (globalToLocal.find(gv) == globalToLocal.end()) {
                int li = (int)lm.localToGlobalVert.size();
                globalToLocal[gv] = li;
                lm.localToGlobalVert.push_back(gv);
            }
        }
    }

    // 2) AddVertices + 拷坐标
    int nv = (int)lm.localToGlobalVert.size();
    vcg::tri::Allocator<CMeshOD>::AddVertices(lm.mesh, nv);
    for (int li = 0; li < nv; li++) {
        lm.mesh.vert[li].P() = mesh->vert[lm.localToGlobalVert[li]].P();
    }

    // 3) AddFace（顶点引用重映射）
    for (int gf : curFaces) {
        int la = globalToLocal[mesh->face[gf].V(0)->Index()];
        int lb = globalToLocal[mesh->face[gf].V(1)->Index()];
        int lc = globalToLocal[mesh->face[gf].V(2)->Index()];
        vcg::tri::Allocator<CMeshOD>::AddFace(
            lm.mesh, &lm.mesh.vert[la], &lm.mesh.vert[lb], &lm.mesh.vert[lc]);
        lm.mesh.face.back().IMark() = mesh->face[gf].IMark();
        lm.mesh.face.back().N() = mesh->face[gf].N();
    }

    lm.Nv0 = (int)lm.mesh.vert.size();

    // 4) 建 globalEdgeToCurFace：用未改动的 mesh 原始面建表（反推来源用）
    for (int gf : curFaces) {
        for (int j = 0; j < 3; j++) {
            int a = mesh->face[gf].V(j)->Index();
            int b = mesh->face[gf].V((j+1)%3)->Index();
            lm.globalEdgeToCurFace[std::minmax(a, b)] = gf;
        }
    }

    // 5) 抓边界缝：curFaces 边在原 mesh 里 FFp 指向 curFaces 外部的，记外部面
    std::set<int> inCur(curFaces.begin(), curFaces.end());
    for (int gf : curFaces) {
        for (int j = 0; j < 3; j++) {
            CFaceOD* adj = mesh->face[gf].FFp(j);
            if (adj == nullptr) continue;
            int adjIdx = static_cast<int>(adj - &mesh->face[0]);
            if (adjIdx < 0 || adjIdx == gf) continue;
            if (inCur.count(adjIdx)) continue;  // 内部边，非缝
            // 这是缝边：记录 local 顶点对 -> 外部面
            int ga = mesh->face[gf].V(j)->Index();
            int gb = mesh->face[gf].V((j+1)%3)->Index();
            int la = globalToLocal[ga], lb = globalToLocal[gb];
            auto key = std::minmax(la, lb);
            lm.seamExternal[{key.first, key.second}] = adjIdx;
        }
    }

    return lm;
}

} // namespace MeshCutByMark

#endif // LOCAL_MESH_CUT_H
```

- [ ] **Step 4: 运行测试确认通过**

构建并运行。Expected: 输出含 `testExtractLocalMesh passed`。

- [ ] **Step 5: 提交**

```bash
git add tool/local_mesh_cut.h tests/test_mesh_cut.cpp
git commit -m "feat: 实现 Phase 2.4 步骤 A——局部 mesh 提取"
```

---

## Task 2: 构造切割输入 line + normal（步骤 B）

**Files:**
- Modify: `tool/local_mesh_cut.h`
- Test: `tests/test_mesh_cut.cpp`

**Interfaces:**
- Consumes: `const Polyline& polyline`、`bool isStart`、`const LocalMesh& lm`、`CMeshOD* mesh`（取区域法向）。
- Produces: `struct CutInput { std::vector<vcg::Point3d> line; vcg::Point3d normal; }`。

- [ ] **Step 1: 写失败测试**

```cpp
void testBuildCutInput() {
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(mesh);

    MeshCutByMark::LocalMeshCutManager mgr;
    std::vector<int> curFaces = {0};
    auto lm = mgr.extractLocalMesh(&mesh, curFaces);

    // 折线 0->1，端点 v0 悬空
    MeshCutByMark::Polyline pl;
    pl.vertexIndices = {0, 1};
    pl.startFaceIdx = 0; pl.startEdgeIdx = 0;
    pl.endFaceIdx = 0; pl.endEdgeIdx = 0;

    auto ci = mgr.buildCutInput(pl, true, lm, &mesh);
    assert(ci.line.size() == 2);
    // 第一点 = 端点 v0
    assert((ci.line[0] - mesh.vert[0].P()).Norm() < 1e-9);
    // 方向 v0-v1 归一化
    vcg::Point3d D = (mesh.vert[0].P() - mesh.vert[1].P()); D.Normalize();
    vcg::Point3d seg = ci.line[1] - ci.line[0]; seg.Normalize();
    assert((seg - D).Norm() < 1e-9);
    // normal = 面法向 (0,0,1)
    assert(std::abs(ci.normal.Z() - 1.0) < 1e-9);
    std::cout << "testBuildCutInput passed" << std::endl;
}
```

注册到 `main()`。

- [ ] **Step 2: 运行确认失败**

`cmake --build build --config Release && ./build/Release/test_mesh_cut.exe`
Expected: 编译失败（`buildCutInput` 未定义）。

- [ ] **Step 3: 写实现**

在 `tool/local_mesh_cut.h` 的 class 内（`extractLocalMesh` 之后）加：

```cpp
    struct CutInput {
        std::vector<vcg::Point3d> line;
        vcg::Point3d normal;
    };

    // 步骤 B：构造延长段 line + 区域 normal
    CutInput buildCutInput(const Polyline& polyline, bool isStart,
                           const LocalMesh& lm, CMeshOD* mesh) {
        CutInput ci;
        const auto& vi = polyline.vertexIndices;
        int endpointIdx = isStart ? vi.front() : vi.back();

        vcg::Point3d P = mesh->vert[endpointIdx].P();
        vcg::Point3d D;
        if (isStart) {
            D = mesh->vert[vi[0]].P() - mesh->vert[vi[1]].P();
        } else {
            int n = (int)vi.size();
            D = mesh->vert[vi[n-1]].P() - mesh->vert[vi[n-2]].P();
        }
        D.Normalize();

        // L = localMesh 包围盒对角线
        vcg::Box3d box;
        for (int i = 0; i < (int)lm.mesh.vert.size(); i++) box.Add(lm.mesh.vert[i].P());
        double L = box.Diag();
        if (L < 1e-9) L = 1.0;

        ci.line.push_back(P);
        ci.line.push_back(P + D * L);

        // normal = 区域法向（取第一个 curFaces 面法向）
        ci.normal = mesh->face[lm.localFaceToGlobal[0]].N();
        if (ci.normal.Norm() < 1e-9) ci.normal = vcg::Point3d(0, 0, 1);
        return ci;
    }
```

- [ ] **Step 4: 运行确认通过**

Expected: `testBuildCutInput passed`。

- [ ] **Step 5: 提交**

```bash
git add tool/local_mesh_cut.h tests/test_mesh_cut.cpp
git commit -m "feat: 实现 Phase 2.4 步骤 B——构造切割 line+normal"
```

---

## Task 3: merge 回主网格（步骤 C）

**Files:**
- Modify: `tool/local_mesh_cut.h`
- Test: `tests/test_mesh_cut.cpp`

**Interfaces:**
- Consumes: `CMeshOD* mesh`、`LocalMesh& lm`（cutter 已切过）、`int targetMark`。
- Produces: `struct MergeResult { std::vector<int> newFaceGlobals; std::vector<int> vertLocalToGlobal; }`；同时把新顶点 append 进 `mesh`，被切原始面槽位**原位改写**（不 `SetD`），额外分片 append。

> 约定：cutter 把被切原始面第一个分片写回原槽位（`i < Nf0` 且引用新顶点），其余分片 append（`i ≥ Nf0`）；merge 据此原位改写全局面，不依赖几何反推或来源属性。

- [ ] **Step 1: 写失败测试**

```cpp
void testMergeBack() {
    // 主网格：1 个三角形 (v0,v1,v2)
    CMeshOD mesh;
    vcg::tri::Allocator<CMeshOD>::AddVertices(mesh, 3);
    mesh.vert[0].P() = Point3m(0,0,0);
    mesh.vert[1].P() = Point3m(1,0,0);
    mesh.vert[2].P() = Point3m(0,1,0);
    vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh.vert[0], &mesh.vert[1], &mesh.vert[2]);
    mesh.face[0].IMark() = 5;

    MeshCutByMark::LocalMeshCutManager mgr;
    auto lm = mgr.extractLocalMesh(&mesh, {0});
    // 模拟 cutter：把 local 面0 分裂——在边 (v0,v1) 中点加新顶点 nv，
    // 用面 (v0,nv,v2) 替换 face0，加面 (nv,v1,v2)。无需写来源属性。
    vcg::tri::Allocator<CMeshOD>::AddVertices(lm.mesh, 1);
    int nv = (int)lm.mesh.vert.size() - 1;  // 新顶点 local 下标 (>= Nv0)
    lm.mesh.vert[nv].P() = vcg::Point3d(0.5, 0, 0);
    lm.mesh.face[0].V(1) = &lm.mesh.vert[nv];          // face0 改成 (v0,nv,v2)
    vcg::tri::Allocator<CMeshOD>::AddFace(lm.mesh,
        &lm.mesh.vert[nv], &lm.mesh.vert[1], &lm.mesh.vert[2]);  // 新面 (nv,v1,v2)

    auto res = mgr.mergeBack(&mesh, lm, /*targetMark*/ 5);

    // 主网格：face0 槽位被原位改写（不 SetD，拓扑连续）；额外分片 append 1 个
    assert(!mesh.face[0].IsD());
    assert(mesh.face[0].V(1)->Index() == 3);  // 改写为 (v0, nv, v2)
    // 新面继承 mark=5
    int aliveNew = 0;
    for (int i = 1; i < (int)mesh.face.size(); i++) {
        if (!mesh.face[i].IsD() && mesh.face[i].IMark() == 5) aliveNew++;
    }
    assert(aliveNew == 2);
    // 顶点 append 了一个新顶点
    assert((int)mesh.vert.size() == 4);
    std::cout << "testMergeBack passed" << std::endl;
}
```

注册到 `main()`。

- [ ] **Step 2: 运行确认失败**

Expected: 编译失败（`mergeBack` 未定义）。

- [ ] **Step 3: 写实现**

在 class 内加：

```cpp
    struct MergeResult {
        std::vector<int> newFaceGlobals;       // append 进 mesh 的额外分片 global 下标
        std::vector<int> vertLocalToGlobal;    // localVertIdx -> globalVertIdx（含新顶点）
    };

    // 判断 local 面是否引用新顶点（local 下标 >= Nv0）
    static bool faceHasNewVert(const LocalMesh& lm, int localFaceIdx) {
        CFaceOD& f = lm.mesh.face[localFaceIdx];
        for (int j = 0; j < 3; j++) {
            int lv = static_cast<int>(f.V(j) - &lm.mesh.vert[0]);
            if (lv >= lm.Nv0) return true;
        }
        return false;
    }

    // 反推新面来自哪个原始面（直接返回 global 面下标）：
    // 取新面里 < Nv0 的原顶点 -> global，用 globalEdgeToCurFace 查它们构成的边属于哪个 curFaces 面。
    // 用建表时的未改动 m_pMesh 拓扑，不受 cutter 原地改写 local 面的影响。
    static int deriveOriginGlobal(const LocalMesh& lm, int localFaceIdx) {
        CFaceOD& f = lm.mesh.face[localFaceIdx];
        int origs[3], no = 0;
        for (int j = 0; j < 3; j++) {
            int lv = static_cast<int>(f.V(j) - &lm.mesh.vert[0]);
            if (lv < lm.Nv0 && lv < (int)lm.localToGlobalVert.size())
                origs[no++] = lm.localToGlobalVert[lv];
        }
        for (int a = 0; a < no; a++)
            for (int b = a + 1; b < no; b++) {
                auto it = lm.globalEdgeToCurFace.find(std::minmax(origs[a], origs[b]));
                if (it != lm.globalEdgeToCurFace.end()) return it->second;
            }
        return -1;
    }

    // 步骤 C
    MergeResult mergeBack(CMeshOD* mesh, LocalMesh& lm, int targetMark) {
        MergeResult res;

        // 1) append 所有新顶点（local >= Nv0）到 *mesh*，一次性批量加（避免多次 realloc）
        int numNew = static_cast<int>(lm.mesh.vert.size()) - lm.Nv0;
        int firstNewG = static_cast<int>(mesh->vert.size());
        if (numNew > 0) vcg::tri::Allocator<CMeshOD>::AddVertices(*mesh, numNew);
        res.vertLocalToGlobal = lm.localToGlobalVert;  // < Nv0 部分
        for (int k = 0; k < numNew; k++) {
            mesh->vert[firstNewG + k].P() = lm.mesh.vert[lm.Nv0 + k].P();
            res.vertLocalToGlobal.push_back(firstNewG + k);
        }

        // 2) 遍历 local 面：新面（含新顶点）append，未动原始面跳过
        for (int i = 0; i < (int)lm.mesh.face.size(); i++) {
            if (lm.mesh.face[i].IsD()) continue;
            if (!faceHasNewVert(lm, i)) continue;  // 未动原始面，保持

            int ga = res.vertLocalToGlobal[static_cast<int>(lm.mesh.face[i].V(0) - &lm.mesh.vert[0])];
            int gb = res.vertLocalToGlobal[static_cast<int>(lm.mesh.face[i].V(1) - &lm.mesh.vert[0])];
            int gc = res.vertLocalToGlobal[static_cast<int>(lm.mesh.face[i].V(2) - &lm.mesh.vert[0])];
            vcg::tri::Allocator<CMeshOD>::AddFace(
                *mesh, &mesh->vert[ga], &mesh->vert[gb], &mesh->vert[gc]);
            int newG = static_cast<int>(mesh->face.size()) - 1;
            mesh->face[newG].IMark() = targetMark;
            res.newFaceGlobals.push_back(newG);

            // 槽位语义：i < Nf0 → 原位改写对应全局面；i >= Nf0 → append
            if (i < (int)lm.localFaceToGlobal.size()) {
                int g = lm.localFaceToGlobal[i];
                if (g >= 0 && g < (int)mesh->face.size() && !mesh->face[g].IsD()) {
                    mesh->face[g].V(0) = &mesh->vert[ga];
                    mesh->face[g].V(1) = &mesh->vert[gb];
                    mesh->face[g].V(2) = &mesh->vert[gc];
                    mesh->face[g].IMark() = targetMark;
                } else {
                    vcg::tri::Allocator<CMeshOD>::AddFace(
                        *mesh, &mesh->vert[ga], &mesh->vert[gb], &mesh->vert[gc]);
                    mesh->face.back().IMark() = targetMark;
                    res.newFaceGlobals.push_back(static_cast<int>(mesh->face.size()) - 1);
                }
            } else {
                vcg::tri::Allocator<CMeshOD>::AddFace(
                    *mesh, &mesh->vert[ga], &mesh->vert[gb], &mesh->vert[gc]);
                int newG = static_cast<int>(mesh->face.size()) - 1;
                mesh->face[newG].IMark() = targetMark;
                res.newFaceGlobals.push_back(newG);
            }
        }
        return res;
    }
```

> `deriveOriginGlobal` 用 `globalEdgeToCurFace`（建表于未改动的 `m_pMesh`），所以即使 cutter 原地改写了 local 面、甚至移除了原始面槽位，反推仍正确。每个被分裂的原始三角形至少有一个含「完整原边」的切片（2 个原顶点），必能命中。`static_cast<int>` 规避 VCG 顶点指针差值的 `ptrdiff_t` 警告。

- [ ] **Step 4: 运行确认通过**

Expected: `testMergeBack passed`。

- [ ] **Step 5: 提交**

```bash
git add tool/local_mesh_cut.h tests/test_mesh_cut.cpp
git commit -m "feat: 实现 Phase 2.4 步骤 C——merge 回主网格"
```

---

## Task 4: 标分割边（步骤 D）

**Files:**
- Modify: `tool/local_mesh_cut.h`
- Test: `tests/test_mesh_cut.cpp`

**Interfaces:**
- Consumes: `CMeshOD* mesh`（已重算 FF）、`const std::vector<std::vector<int>>& cutLines`（每条折线=新顶点 global 下标序列）、`const LocalMesh& lm`、`const Polyline& polyline`（取端点）。
- Produces: 把切割路径上的 face-edge 的 `FFp` 置自指（断开）。

- [ ] **Step 1: 写失败测试**

```cpp
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
```

注册到 `main()`。

- [ ] **Step 2: 运行确认失败**

Expected: 编译失败（`markCutEdges` 未定义）。

- [ ] **Step 3: 写实现**

在 class 内加：

```cpp
    // 步骤 D：把 cutLines 上的边标成边界（FFp 自指）
    void markCutEdges(CMeshOD* mesh, const std::vector<std::vector<int>>& cutLines) {
        // 收集所有要标记的 global 顶点对
        std::set<std::pair<int,int>> edges;
        for (const auto& cl : cutLines) {
            for (size_t i = 0; i + 1 < cl.size(); i++) {
                edges.insert(std::minmax(cl[i], cl[i+1]));
            }
        }
        if (edges.empty()) return;

        // 遍历面边，命中则两侧 FFp 自指
        for (int f = 0; f < (int)mesh->face.size(); f++) {
            if (mesh->face[f].IsD()) continue;
            for (int e = 0; e < 3; e++) {
                int a = mesh->face[f].V(e)->Index();
                int b = mesh->face[f].V((e+1)%3)->Index();
                if (edges.count(std::minmax(a,b))) {
                    CFaceOD* adj = mesh->face[f].FFp(e);
                    mesh->face[f].FFp(e) = &mesh->face[f];
                    mesh->face[f].FFi(e) = e;
                    if (adj != nullptr && adj != &mesh->face[f]) {
                        // 断开对面的同一条边
                        for (int e2 = 0; e2 < 3; e2++) {
                            if (adj->FFp(e2) == &mesh->face[f]) {
                                adj->FFp(e2) = adj;
                                adj->FFi(e2) = e2;
                            }
                        }
                    }
                }
            }
        }
    }
```

- [ ] **Step 4: 运行确认通过**

Expected: `testMarkCutEdges passed`。

- [ ] **Step 5: 提交**

```bash
git add tool/local_mesh_cut.h tests/test_mesh_cut.cpp
git commit -m "feat: 实现 Phase 2.4 步骤 D——标记分割边"
```

---

## Task 5: 外部邻接加点（步骤 E）

**Files:**
- Modify: `tool/local_mesh_cut.h`
- Test: `tests/test_mesh_cut.cpp`

**Interfaces:**
- Consumes: `CMeshOD* mesh`、`const LocalMesh& lm`、`const MergeResult& merge`（新顶点的 global 映射，见下）。
- Produces：对落在缝边上的新顶点，把外部邻接面在加点处一分为二（`SetD` 原外部面 + 加 2 面）。

> 依赖 Task 3 的 `MergeResult::vertLocalToGlobal`（已在 Task 3 加入并赋值）。本 Task 直接用它。

- [ ] **Step 1: 写失败测试**

```cpp
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
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(mesh);

    MeshCutByMark::LocalMeshCutManager mgr;
    auto lm = mgr.extractLocalMesh(&mesh, {0});
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
    std::cout << "testPropagateExternal passed" << std::endl;
}
```

注册到 `main()`。

- [ ] **Step 2: 运行确认失败**

Expected: 编译失败（`propagateExternal` 未定义）。

- [ ] **Step 3: 写实现**

在 class 内加：

```cpp
    // 步骤 E：缝边上的新顶点 -> 把外部邻接面在加点处一分为二
    void propagateExternal(CMeshOD* mesh, const LocalMesh& lm, const MergeResult& merge) {
        // 对每个新顶点（local >= Nv0）：判断是否落在某条缝边上
        for (int lv = lm.Nv0; lv < (int)lm.mesh.vert.size(); lv++) {
            vcg::Point3d p = lm.mesh.vert[lv].P();
            // 找它落在哪条缝边（local 顶点对）上
            for (const auto& kv : lm.seamExternal) {
                int la = kv.first.first, lb = kv.first.second;
                int extG = kv.second;
                if (mesh->face[extG].IsD()) continue;
                vcg::Point3d pa = lm.mesh.vert[la].P();
                vcg::Point3d pb = lm.mesh.vert[lb].P();
                if (!pointOnSegment(p, pa, pb)) continue;

                // 新顶点 global
                int gv = (lv < (int)merge.vertLocalToGlobal.size()) ? merge.vertLocalToGlobal[lv] : -1;
                if (gv < 0) continue;

                // 外部面的三个顶点，找出缝边两端的 global 下标
                int ga = lm.localToGlobalVert.size() > la ? lm.localToGlobalVert[la] : -1;
                int gb = lm.localToGlobalVert.size() > lb ? lm.localToGlobalVert[lb] : -1;
                splitExternalFace(mesh, extG, ga, gb, gv);
                break;  // 该新顶点已处理
            }
        }
    }

    static bool pointOnSegment(const vcg::Point3d& p, const vcg::Point3d& a, const vcg::Point3d& b) {
        vcg::Point3d ab = b - a, ap = p - a;
        double t = ap.Dot(ab) / ab.Dot(ab);
        if (t < -1e-9 || t > 1 + 1e-9) return false;
        vcg::Point3d proj = a + ab * t;
        return (proj - p).Norm() < 1e-7;
    }

    // 把外部面 extG 沿 (ga, gb) 边在 gv 处一分为二。用下标访问，不跨 AddFace 持引用。
    static void splitExternalFace(CMeshOD* mesh, int extG, int ga, int gb, int gv) {
        // 先读出所需信息（AddFace 可能 realloc mesh->face，使引用失效）
        int idx[3] = {
            mesh->face[extG].V(0)->Index(),
            mesh->face[extG].V(1)->Index(),
            mesh->face[extG].V(2)->Index() };
        int gc = -1;
        for (int j = 0; j < 3; j++) if (idx[j] != ga && idx[j] != gb) gc = idx[j];
        int imark = mesh->face[extG].IMark();
        if (gc < 0) return;

        // 新面 A: (ga, gv, gc)
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh->vert[ga], &mesh->vert[gv], &mesh->vert[gc]);
        mesh->face.back().IMark() = imark;
        // 新面 B: (gv, gb, gc)
        vcg::tri::Allocator<CMeshOD>::AddFace(mesh, &mesh->vert[gv], &mesh->vert[gb], &mesh->vert[gc]);
        mesh->face.back().IMark() = imark;
        // 原 extG 标记删除（按下标，安全）
        mesh->face[extG].SetD();
    }
```

- [ ] **Step 4: 运行确认通过**

Expected: `testPropagateExternal passed`。

- [ ] **Step 5: 提交**

```bash
git add tool/local_mesh_cut.h tests/test_mesh_cut.cpp
git commit -m "feat: 实现 Phase 2.4 步骤 E——外部邻接加点"
```

---

## Task 6: RegionMarker::growNewMark + 收尾（步骤 F）

**Files:**
- Modify: `tool/region_marker.h`
- Modify: `tool/local_mesh_cut.h`
- Test: `tests/test_mesh_cut.cpp`

**Interfaces:**
- Consumes: `RegionMarker& regionMarker`、`CMeshOD* mesh`、`std::vector<int>& curFaces`、`const MergeResult& merge`。
- Produces：resize `m_newMark`、重建 `curFaces`、重算 `FaceFace`。

- [ ] **Step 1: 给 RegionMarker 加 growNewMark**

编辑 `tool/region_marker.h`，在 `setNewMark` 之后（public 区）加声明与 inline 实现：

```cpp
    // 扩容 m_newMark 到 newSize，新增元素置 0（不重置已有）
    void growNewMark(size_t newSize) {
        if (newSize > m_newMark.size()) m_newMark.resize(newSize, 0);
    }
```

- [ ] **Step 2: 写失败测试**

```cpp
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
```

注册到 `main()`，构建运行确认通过。

- [ ] **Step 3: 写 finalize 实现**

在 `tool/local_mesh_cut.h` 的 class 内加：

```cpp
    // 步骤 F：resize m_newMark、重建 curFaces、重算 FF。在 D 之前调用 FF，D 之后不改拓扑。
    // 注意调用顺序：cutRegion 里先 finalizeTopology（重算 FF）→ 再 markCutEdges。
    void finalizeGrow(RegionMarker& regionMarker, CMeshOD* mesh) {
        regionMarker.growNewMark(mesh->face.size());
        vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*mesh);
        vcg::tri::UpdateNormal<CMeshOD>::PerFace(*mesh);
    }

    // 重建 curFaces：保留原位改写后的原下标（仍有效），加入 append 的额外分片
    static void rebuildCurFaces(std::vector<int>& curFaces, CMeshOD* mesh,
                                const MergeResult& merge) {
        std::vector<int> out;
        for (int gf : curFaces) {
            if (gf < (int)mesh->face.size() && !mesh->face[gf].IsD()) out.push_back(gf);
        }
        for (int nf : merge.newFaceGlobals) out.push_back(nf);
        curFaces = out;
    }
```

并在 `local_mesh_cut.h` 顶部 include 区确保有：
```cpp
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
```

- [ ] **Step 4: 写测试**

```cpp
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
```

注册、构建、确认通过。

- [ ] **Step 5: 提交**

```bash
git add tool/region_marker.h tool/local_mesh_cut.h tests/test_mesh_cut.cpp
git commit -m "feat: 实现 Phase 2.4 步骤 F——growNewMark + 收尾"
```

---

## Task 7: cutRegion 总装 + 接入 Phase 2.4 + 桩 cutter

**Files:**
- Modify: `tool/local_mesh_cut.h`（加 `cutRegion`）
- Modify: `JasMeshMarkAndCutSplit.h` / `.cpp`（接入）
- Modify: `tests/test_mesh_cut.cpp`（桩 cutter + 冒烟测试）

**Interfaces:**
- Consumes: `CMeshOD* mesh`、`std::vector<int>& curFaces`、`const std::vector<Polyline>& polylines`、`int targetMark`、`RegionMarker& regionMarker`。
- Produces：执行 A→B→[cutter]→C→E→F(FF)→D→F(curFaces)。

- [ ] **Step 1: 在 local_mesh_cut.h 加 cutRegion**

在 class 内加：

```cpp
    // 总装：对一个区域跑 A→B→cutter→C→E→F(FF)→D→F(curFaces)
    void cutRegion(CMeshOD* mesh,
                   std::vector<int>& curFaces,
                   const std::vector<Polyline>& polylines,
                   int targetMark,
                   RegionMarker& regionMarker)
    {
        LocalMesh lm = extractLocalMesh(mesh, curFaces);

        // B + cutter：对每条 NON_MANIFOLD 折线的每个悬空端点。
        // 记录端点 global 顶点 + cutLine（local 新顶点下标），merge 后转 global 并把端点拼到最前。
        struct PendingCut { int endpointGlobal; std::vector<int> cutLineLocal; };
        std::vector<PendingCut> pending;
        JasMeshAddCutLines cutter;
        MeshCutByMark::CutPlaneManager cpm;  // 复用 isOnMarkDiffEdge
        for (const auto& pl : polylines) {
            if (pl.type != CUT_EDGE_NON_MANIFOLD) continue;
            if (!cpm.isOnMarkDiffEdge(pl.startFaceIdx, pl.startEdgeIdx, mesh)) {
                auto ci = buildCutInput(pl, true, lm, mesh);
                std::vector<int> cl;
                cutter.AddCutLines(&lm.mesh, ci.normal, ci.line, cl);
                if (!cl.empty()) pending.push_back({pl.vertexIndices.front(), cl});
            }
            if (!cpm.isOnMarkDiffEdge(pl.endFaceIdx, pl.endEdgeIdx, mesh)) {
                auto ci = buildCutInput(pl, false, lm, mesh);
                std::vector<int> cl;
                cutter.AddCutLines(&lm.mesh, ci.normal, ci.line, cl);
                if (!cl.empty()) pending.push_back({pl.vertexIndices.back(), cl});
            }
        }

        // C：merge 回主网格（新顶点此时 append 进 mesh，得到 vertLocalToGlobal）
        MergeResult merge = mergeBack(mesh, lm, targetMark);

        // cutLines：端点 global + 新顶点 global，拼成完整切割路径（spec D.1）
        std::vector<std::vector<int>> cutLines;
        for (const auto& pc : pending) {
            std::vector<int> g;
            g.push_back(pc.endpointGlobal);  // 端点（原顶点，global）
            for (int lv : pc.cutLineLocal) {
                if (lv >= 0 && lv < (int)merge.vertLocalToGlobal.size())
                    g.push_back(merge.vertLocalToGlobal[lv]);
            }
            if (g.size() >= 2) cutLines.push_back(g);
        }

        // E：外部加点（在重算 FF 之前）
        propagateExternal(mesh, lm, merge);

        // F1：resize m_newMark + 重算 FF/normal
        finalizeGrow(regionMarker, mesh);

        // D：标分割边（cutLines 已是 global 顶点）
        markCutEdges(mesh, cutLines);

        // F2：重建 curFaces
        rebuildCurFaces(curFaces, mesh, merge);
    }
```

> cutLine（cutter 输出）只含新顶点；端点（原顶点）由 `pending.endpointGlobal` 拼到路径最前，这样 `markCutEdges` 能标到端点→首新顶点那条边（spec D.1）。端点本身是折线端点（global），无需经过 `vertLocalToGlobal`。

- [ ] **Step 2: 接入 JasMeshMarkAndCutSplit**

编辑 `JasMeshMarkAndCutSplit.h`：在 include 区加 `#include "tool/local_mesh_cut.h"`；在 private 成员加：
```cpp
    MeshCutByMark::LocalMeshCutManager m_localMeshCut;
```

编辑 `JasMeshMarkAndCutSplit.cpp` 的 Phase 2.4（399-427 行整段 `for (const auto& polyline : polylines) {...}`）替换为：
```cpp
        // 2.4 从端点延长切割：局部 mesh + cutter + 合并回主网格（targetMark 在 383 行已定义）
        m_localMeshCut.cutRegion(m_pMesh, curFaces, polylines, targetMark, m_regionMarker);
```

- [ ] **Step 3: 提供桩 cutter（仅测试链接用）**

在 `tests/test_mesh_cut.cpp` 顶部（include 之后）加一个最小桩，让测试可链接（**不写来源属性**，mergeBack 靠几何反推）：

```cpp
#include "tool/cut_mesh.h"
// 桩 cutter（plumbing 测试用；真实 cutter 由外部提供，生产构建链接真实实现）
void JasMeshAddCutLines::AddCutLines(CMeshOD* pMesh, vcg::Point3d& normal,
                                     std::vector<vcg::Point3d>& line,
                                     std::vector<int>& cutLine) {
    // 最简：在第一个面的边 (V0,V1) 中点加一个新顶点，把该面一分为二
    if (pMesh->face.empty() || line.size() < 2) return;
    if (!pMesh->face.IsFFAdjacencyEnabled()) { pMesh->face.EnableFFAdjacency(); }
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*pMesh);

    CFaceOD& f0 = pMesh->face[0];
    int a = f0.V(0)->Index(), b = f0.V(1)->Index(), c = f0.V(2)->Index();
    int nv = (int)pMesh->vert.size();
    vcg::tri::Allocator<CMeshOD>::AddVertices(*pMesh, 1);
    pMesh->vert[nv].P() = (pMesh->vert[a].P() + pMesh->vert[b].P()) * 0.5;
    // face0 改写成 (a, nv, c)
    f0.V(1) = &pMesh->vert[nv];
    // 新增面 (nv, b, c)
    vcg::tri::Allocator<CMeshOD>::AddFace(*pMesh, &pMesh->vert[nv], &pMesh->vert[b], &pMesh->vert[c]);
    cutLine = { nv };
    (void)normal;
}
```

> 注意：本仓 `cut_mesh.h` 没有 `AddCutLines` 的 inline 实现，所以测试 TU 必须提供定义才能链接。生产构建由外部 cutter 提供同名符号——届时不要链接此桩（或将桩放进单独的 test-only TU）。

- [ ] **Step 4: 写冒烟测试**

```cpp
void testCutRegionPlumbing() {
    // 区域 2 个三角形 mark=1，构造一条 NON_MANIFOLD 折线端点悬空
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
    pl.vertexIndices = {0, 1};
    pl.startFaceIdx = 0; pl.startEdgeIdx = 0;
    pl.endFaceIdx = 0;   pl.endEdgeIdx = 0;  // 端点不在 mark-diff 边 -> 触发切割

    MeshCutByMark::LocalMeshCutManager mgr;
    mgr.cutRegion(&mesh, curFaces, {pl}, /*targetMark*/1, rm);

    // 真实 cutter 为保守黑盒：切割后拓扑连续、原面不 IsD；切不动时 no-op 也合法。
    // plumbing 只验证管线跑通与 curFaces 有效。
    assert(!curFaces.empty());
    for (int fi : curFaces) {
        assert(fi >= 0 && fi < (int)mesh.face.size());
        assert(!mesh.face[fi].IsD());
    }
    std::cout << "testCutRegionPlumbing passed" << std::endl;
}
```

注册到 `main()`。

- [ ] **Step 5: 构建、运行全部测试**

`cmake --build build --config Release && ./build/Release/test_mesh_cut.exe`
Expected: 全部 `passed`，无断言失败。

- [ ] **Step 6: 提交**

```bash
git add tool/local_mesh_cut.h JasMeshMarkAndCutSplit.h JasMeshMarkAndCutSplit.cpp tests/test_mesh_cut.cpp
git commit -m "feat: Phase 2.4 接入局部 mesh 切割管线"
```

---

## Task 8: 清理旧路径 + 更新文档

**Files:**
- Modify: `tool/cut_plane.h`（保留 `makeCutPlane`/`isOnMarkDiffEdge`/`signedDistance`，删 `cutTriangleByPlane`）
- Modify: `JasMeshMarkAndCutSplit.cpp`（已无 `cutTriangleByPlane` 调用，确认）
- Modify: `tests/test_mesh_cut.cpp`（移除/保留 `testSignedDistanceAndIntersection`，不影响）
- Modify: `CLAUDE.md`、`README.md`

- [ ] **Step 1: 删除 cutTriangleByPlane**

编辑 `tool/cut_plane.h`：删除 `cutTriangleByPlane` 的声明（22-26 行）与实现（136-156 行）。保留 `makeCutPlane`、`isOnMarkDiffEdge`、`signedDistance`、`intersectSegmentPlane`（cutRegion 复用 `isOnMarkDiffEdge`）。

- [ ] **Step 2: 构建确认无引用残留**

`cmake --build build --config Release`
Expected: 编译通过（无 `cutTriangleByPlane` 未定义引用）。若报错，搜索残留调用并清除。

- [ ] **Step 3: 更新 CLAUDE.md**

把「Known Limitations」里的 `cutTriangleByPlane` 条目替换为：
```
- Phase 2.4 延长切割依赖外部 `JasMeshAddCutLines::AddCutLines`（稳定黑盒，不在本仓实现）；测试用桩 cutter 做 plumbing 验证
```
在 Architecture 的 Phase 2 描述里补充：`LocalMeshCutManager`（`tool/local_mesh_cut.h`）负责提取局部 mesh → cutter → 合并回主网格。

- [ ] **Step 4: 更新 README.md**

把「已知限制」第 1 条「cutTriangleByPlane 未实现」改为「Phase 2.4 延长切割已改为局部 mesh + 外部 cutter 管线」；在「未来改进」里勾掉对应项。

- [ ] **Step 5: 提交**

```bash
git add tool/cut_plane.h CLAUDE.md README.md
git commit -m "chore: 移除 cutTriangleByPlane 存根，更新文档"
```

---

## 验收

- 全部测试通过：`./build/Release/test_mesh_cut.exe`（原 21 + 新增 7 个：`testExtractLocalMesh`、`testBuildCutInput`、`testMergeBack`、`testMarkCutEdges`、`testPropagateExternal`、`testGrowNewMark`、`testRebuildCurFaces`、`testCutRegionPlumbing`）。
- 接入真实 cutter 后（替换桩），含 `NON_MANIFOLD` 悬空端点的网格能正确分出 ≥2 个子区域、网格 watertight。
