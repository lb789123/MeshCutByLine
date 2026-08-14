# MeshCutByLine 修复方案（正确性优先）

> 范围：`MeshCutByLine` 主仓库 + `external/cgalLocalMeshCut` 子模块。
> 基准：当前 `main`（含两套实现：主流程走 `CutFacesExact`/`mergeLocalCut`/`stitchAllSeams`，旧 `cutRegion` 链路已脱离主流程）。
> 目标：把「问题分析」中列出的正确性/健壮性问题按优先级落成可提交的改动。

## 0. 目标与验收标准

修复后必须满足：

1. **无 newMark=0 垃圾区**：`SplitMeshByMarkAndEdge` 输出的每个面都属于某个 `splitReg`，`splitReg.newMark >= 1`，不存在「newMark=0 的孤儿面」被合并进一个假区域。
2. **`splitReg.mark` 恒等于原始平面标记**：切开子区的 `mark` 与输入 `IMark` 一致，不被局部重标污染。
3. **缝合用精确坐标**：`stitchAllSeams` 的去重键是 `ExactPoint`，不再经过 `double` 往返。
4. **非流形输入不崩溃、不静默丢面**：非流形边进 corefine 前劈开；star vertex 保守跳过并告警。
5. **每条缝边的所有外部邻接面都被细分**（非流形缝边不漏缝）。
6. **切割线经过已有全局顶点时不产生重复顶点**。
7. 每个步骤单独可编译、测试通过；关键回归在 Debug 与 Release 下都拦截（不依赖裸 `assert`）。

---

## 1. 修复总览

| 步骤 | 问题 | 主要改动文件 | 新增/修改测试 |
|---|---|---|---|
| 1 | 1.1 非流形边面被丢弃 → newMark=0 垃圾区 | `JasMeshLocalMarkAndCutSplit.cpp`、`JasMeshLocalMarkAndCutSplitInternal.h`、`tool/local_mesh_cut.h` | `testNonManifoldEdgeRegion` |
| 2 | 1.2 `splitReg.mark` 被局部重标污染 | `tool/local_mesh_cut.h` | `testSplitRegMarkPreserved` |
| 3 | 1.4 缝合按 double 去重 | `tool/local_cut_result.h`、`tool/local_mesh_cut.h` | 更新 `testStitchAllSeams` + `testSeamExactDedup` |
| 4 | 1.3 / 1.5 / 1.6 健壮性 | `JasMeshLocalMarkAndCutSplit.cpp`、`JasMeshLocalMarkAndCutSplitInternal.h`、`tool/local_mesh_cut.h` | `testStarVertexSkip`、`testSeamMultiExternal`、`testExistingVertexReuse` |
| 5 | 3.x 性能 | `JasMeshLocalMarkAndCutSplit.cpp`、`tool/local_mesh_cut.h`、`tool/polyline.h`、`tool/region_marker.h` | 基准脚本（可选） |
| 6 | 4.x 代码清理 | 删除 `cutRegion` 旧链路、`tool/cut_plane.h`、死代码 | 删除对应旧测试 |
| 7 | 测试加固 | `tests/test_mesh_cut.cpp`、子模块 tests | Release 安全断言 + 回归 |

---

## 2. 步骤 1：修复非流形边面被丢弃（问题 1.1）

### 根因

- `JasMeshLocalMarkAndCutSplit.cpp:736-747`（`CutFacesExact`）：`result.mesh.add_face(...)` 失败（非流形边）时只 `result.dropped_input_face_count++`，**不记录、不补偿**。
- 被丢弃的面不进入 `ExactMesh` → `mergeLocalCut` 不给它赋 newMark → `initNewMark` 后它恒为 0 → Phase 3 把所有 `newMark==0` 的面合成一个假区域（`JasMeshMarkAndCutSplit.cpp:531-536`）。

### 方案（两段式，先做正确性护栏，再做根治）

#### 1a. 进 corefine 前劈开非流形边（根治，推荐）

在 `CutFacesExact` 的 `add_face` 之前，对区域内的非流形边（3+ 面共享的边）复制端点顶点，使局部 `ExactMesh` 流形：

1. **第一遍**：遍历 `face_indices`，收集每张面的三个全局顶点三元组，并按 `std::minmax(global_a, global_b)` 建立 `edge -> vector<region-face-index>` 映射。
2. **识别**：`edge_faces.size() >= 3` 的边是非流形边。对每条非流形边，保留前 2 张面使用原顶点，第 3 张及以后的面各用一对克隆顶点。
3. **克隆**：对每个需要拆的面，`result.mesh.add_vertex(ToExact(coord))` 两次（两个端点），并设置
   `result.vertex_global_map[clone] = 原全局顶点下标`（**映射回同一全局顶点**）。
4. **第二遍**：`add_face` 时用（可能克隆过的）顶点句柄。这样 `mergeLocalCut` 写回时仍引用同一全局顶点，全局网格保持非流形（这是输入允许的），局部 `ExactMesh` 却已流形，corefine 不会再丢面。

关键约束：克隆顶点必须把 `vertex_global_map` 指回**原全局顶点**（不是 -1），否则 `mergeLocalCut` 会追加重复全局顶点。`mergeLocalCut` 里 `original_index >= 0` 分支（`local_mesh_cut.h:811-814`）已天然复用该全局顶点。

#### 1b. 护栏：消费 `dropped_input_face_count`（必须，无论 1a 是否完成）

即使 1a 生效，也要保证 `dropped_input_face_count > 0` 时不再静默：

- `JasMeshLocalMarkAndCutSplitInternal.h` 的 `ExactCutResult` 增加字段：
  ```cpp
  std::vector<int> dropped_faces;   // 被 CGAL add_face 拒绝的全局面下标
  ```
- `CutFacesExact` 里 `add_face` 失败分支改为同时 `result.dropped_faces.push_back(face_index);`。
- `mergeLocalCut`（`local_mesh_cut.h`）在 newMark 赋值循环之后补一段：
  ```cpp
  // 被 CGAL 拒绝的非流形面：未切割、保留原几何，但必须分配 newMark，
  // 否则掉进 newMark=0 垃圾区。
  for (int globalFaceIndex : exact.dropped_faces) {
      if (globalFaceIndex < 0 || globalFaceIndex >= (int)mesh->face.size() ||
          mesh->face[globalFaceIndex].IsD())
          continue;
      regionMarker.setNewMark(globalFaceIndex, newMarkCounter++);
  }
  ```
- Phase 3（`JasMeshMarkAndCutSplit.cpp:531`）加防御：分组时若仍有 `newMark==0` 的面，Debug 下抛错、Release 下各自分配递增 newMark，绝不合并成一个假区域。

### 测试

- 复用 `testSplitMeshNormals` 的网格（`tests/test_mesh_cut.cpp:2097`，已含 3 面共享边 (1,2) 的非流形边），新增 `testNonManifoldEdgeRegion`：
  - 跑 `SplitMeshByMarkAndEdge`；
  - 断言每个 `splitReg.newMark >= 1`；
  - 断言不存在任何 `region.mark` 为 0 或异常值；
  - 断言「所有非删除面的 `getNewMark` 均 > 0」（遍历 `mesh.face`，校验 `regionMarker` 或等价检查）。

---

## 3. 步骤 2：修复 `splitReg.mark` 被局部重标污染（问题 1.2）

### 根因

- `SplitExactMeshByCut`（`JasMeshLocalMarkAndCutSplit.cpp:603-682`）把 `face_mark_map` 重标为局部号（首块保留原 mark、其余 `next_mark++`）。
- `mergeLocalCut` 把 `IMark() = exact.face_mark_map[...]`（`local_mesh_cut.h:864-865`、`:875`）写回全局。
- Phase 3 用 `face[faces[0]].IMark()` 填 `splitReg.mark`（`JasMeshMarkAndCutSplit.cpp:551`），于是读到局部号。

### 改动（`tool/local_mesh_cut.h`，`mergeLocalCut`）

`IMark` 恒等于本区域原始 mark（=`result.targetMark`，因为 `floodFill` 只收集 `IMark==targetMark` 的面）：

- 第 864-865 行：
  ```cpp
  mesh->face[globalFaceIndex].IMark() = exact.face_mark_map[faceIndex];
  ```
  改为
  ```cpp
  mesh->face[globalFaceIndex].IMark() = result.targetMark;
  ```
- 第 875 行：
  ```cpp
  newFace->IMark() = exact.face_mark_map[faceIndex];
  ```
  改为
  ```cpp
  newFace->IMark() = result.targetMark;
  ```

局部子区域划分仍由 `localMarkToGlobalMark`（`local_mesh_cut.h:911-933`）经 `face_mark_map` 映射到 `newMark`，不受影响——`face_mark_map` 只是「局部号 → newMark」的中间键，不再污染 `IMark`。

### 测试

- 新增 `testSplitRegMarkPreserved`：对多 mark、且某个 mark 区域被切开成多子区的网格跑 `SplitMeshByMarkAndEdge`，断言每个 `region.mark` 等于该区域面在输入时的 `IMark`（可在输入时给每张面记录原始 mark，输出后按 `inTris[0]` 反查）。

---

## 4. 步骤 3：缝合去重改为精确坐标（问题 1.4）

### 根因

- `SeamCutPoint.point` 是 `vcg::Point3d`（double），`mergeLocalCut` 用 `CGAL::to_double` 填充（`local_mesh_cut.h:900-903`）。
- `stitchAllSeams` 用 `std::map<std::tuple<double,double,double>, int>` 去重（`local_mesh_cut.h:522`、`:539-551`），违反设计文档 §7.2「不得转回 double 再比较」。

### 改动

1. `tool/local_cut_result.h` 的 `SeamCutPoint` 增加字段（该文件已包含 `JasMeshLocalMarkAndCutSplitInternal.h`，`jaslmc::ExactPoint` 可见）：
   ```cpp
   jaslmc::ExactPoint exactPoint;   // 精确坐标，用于缝合去重
   ```
   保留 `point`（double）用于调试/打印。

2. `tool/local_mesh_cut.h` 的 `mergeLocalCut`，在填充 `SeamCutPoint` 处（`:897-904`）增加：
   ```cpp
   cutPoint.exactPoint = point.point;   // ExactSeamPoint::point 即 ExactPoint
   ```

3. `tool/local_mesh_cut.h` 的 `stitchAllSeams`：
   ```cpp
   // 旧
   std::map<std::tuple<double, double, double>, int> pointToVertex;
   const auto coordKey = std::make_tuple(point.point.X(), point.point.Y(), point.point.Z());
   // 新
   std::map<jaslmc::ExactPoint, int> pointToVertex;
   // key 直接用 point.exactPoint（ExactPoint 有 operator<，可作 map key）
   ```
   其余合并/重写逻辑不变。

### 测试

- **必须同步修改** `testStitchAllSeams`（`tests/test_mesh_cut.cpp:1975-1987`）：手工构造的 `SeamCutPoint` 需同时设置 `exactPoint`（例如 `pointA.exactPoint = jaslmc::ExactPoint(1, 0.5, 0);`），否则 `map<ExactPoint,...>` 键为空点导致去重失败。
- 新增 `testSeamExactDedup`：构造两个相邻区域共享一条缝边，两侧切点坐标在 double 下相等、在精确核下相等的场景（可直接复用 `testStitchAllSeams` 网格），断言缝合后网格无 `boundaryEdges` 裂缝、且不存在两对「不同全局顶点索引但坐标精确相等」的顶点。

---

## 5. 步骤 4：健壮性三连（问题 1.3 / 1.5 / 1.6）

### 4a. star vertex 校验 + 保守跳过（问题 1.3）

- 在 `CutFacesExact` 建好局部 `ExactMesh` 后、corefine 前，检测 star vertex（顶点扇面不连通）。
- 复用 `tests/test_mesh_cut.cpp:1494` 的 `CollectTempManifoldStats` 逻辑思路，在子模块内实现一个 `CountNonManifoldVertices(const CMeshOD&, const std::vector<int>& face_indices)`（或直接在 `CutFacesExact` 内联检测）。
- 检出后：该区域**保守 no-op**——`CutFacesExact` 返回 `false`，`prepareLocalCut` 里对 `false` 的处理为：仍让该区域所有面在 `mergeLocalCut` 里赋同一 newMark（作为单个未切区域），并打印告警。这与 README「保守 no-op」哲学一致，且是 `testTempStarVertexCorefine`（当前被注释、Release 会崩）的正确替代行为。
- 后续可选根治：按扇面复制 star 顶点（拆点）后再进 corefine，作为独立后续任务。

### 4b. 缝边外部邻接面改为一对多（问题 1.5）

- `JasMeshLocalMarkAndCutSplitInternal.h` 的 `ExactSeam`：
  ```cpp
  // 旧 int external_face;
  std::vector<int> external_faces;   // 该缝边的全部外部邻接面
  ```
- `CutFacesExact` 的 `seam_external`（`JasMeshLocalMarkAndCutSplit.cpp:750`）：
  ```cpp
  // 旧 std::map<std::pair<int,int>, int>
  std::map<std::pair<int, int>, std::vector<int>> seam_external;
  // 填充改为 seam_external[key].push_back(adjacent_index);（去重）
  ```
- `tool/local_cut_result.h` 的 `SeamCutLine`：
  ```cpp
  // 旧 int externalFaceIndex;
  std::vector<int> externalFaceIndices;
  ```
- `mergeLocalCut`（`local_mesh_cut.h:882-907`）填充 `seamLine.externalFaceIndices`（来自 `seam.external_faces`）。
- `stitchAllSeams`（`local_mesh_cut.h:527-536`）改为遍历 `seam.externalFaceIndices` 逐个 `seamExternalFaces[seamKey].insert(...)`。

### 4c. 恢复「已有全局顶点精确坐标索引」（问题 1.6）

- 当前 `mergeLocalCut`（`local_mesh_cut.h:806-830`）只有 `new_point_to_index`（本次调用内去重），未对区域外已存在的全局顶点建索引；延长切割线穿过区域外已有顶点时会追加坐标相同的重复顶点。
- 方案：在 `SplitMeshByMarkAndEdge` 进入 `mergeLocalCut` 串行循环前，构建一次全量索引
  `std::map<jaslmc::ExactPoint, int> existing_point_to_index`（跳过 `IsD`），并**按引用**传给 `mergeLocalCut`；`mergeLocalCut` 每次追加新顶点后同时更新该索引。这样正确性与性能兼得（避免每区域 O(V log V) 重建）。
- 在 `mergeLocalCut` 的顶点解析循环里，新顶点分支先查 `existing_point_to_index`，命中则 `vertex_to_global[...] = 已有全局顶点`，否则才 `AddVertices` 并回填索引。

### 测试

- `testStarVertexSkip`：构造 star vertex 网格，跑 `SplitMeshByMarkAndEdge`，断言不崩溃、输出区域完整（面数守恒）、且该区域作为单个未切区域存在。
- `testSeamMultiExternal`：构造非流形缝边（2 个外部邻接面），断言两个外部面都被细分、边界边计数正确。
- `testExistingVertexReuse`：切割线精确穿过一个「区域外已存在」的全局顶点，跑完断言「无坐标精确相等、索引不同的顶点对」（复用 `CollectTempManifoldStats` 的 `coincidentVertexPairCount`）。

---

## 6. 步骤 5：性能优化（3.x）

> 此步骤与正确性解耦，可最后做或分提交，均不改变输出。

### 5a. `find_exact_vertex` 线性扫描 → 索引（3.2）

- `CutFacesExact`（`JasMeshLocalMarkAndCutSplit.cpp:785-795`）的 lambda 改为维护
  `std::map<ExactPoint, ExactMesh::Vertex_index> point_to_vertex`，在 `add_vertex` 处（`:723-728`）同步插入；`extension_at_start/end` 判断与折线顶点解析都查该 map，`O(P log V)`。
- 同理优化 `JasMeshAddCutLines.cpp:25-36` 的 `FindCutVertex`（子模块，旧路径仍用），改为会话级 `point_to_vertex` 索引（`LocalCutSession` 持有，`AddCutLine`/`Commit` 复用）。

### 5b. `SplitExactMeshByCut` 的 visited 与逐 mark 扫描（3.1）

- `visited` 从 `std::map<Face_index, char>` 改为 `std::vector<char>` + 代数标记（generation counter），避免每 mark 重建/清零。
- 把「逐 mark 全量扫描 `mesh.faces()` 找起点」改为「一次遍历面，按 mark 分组到 `map<mark, vector<Face_index>>`，再逐 mark flood-fill」，消除 `O(mark × F)` 重扫。

### 5c. 去掉逐区域全量 `FaceFace`（3.3）

- `mergeLocalCut` 目前调 `finalizeGrow`（`local_mesh_cut.h:909` → `:625-631`），内含 `FaceFace`+`PerFace`。由于 `SplitMeshByMarkAndEdge` 在 `stitchAllSeams` 之后已做一次全量 `FaceFace`+`PerFace`（`JasMeshMarkAndCutSplit.cpp:526-527`），`mergeLocalCut` 里只需保留 `regionMarker.growNewMark(mesh->face.size())`，删除 `FaceFace`/`PerFace` 重算。
- 注意：`stitchAllSeams` 的 `splitExternalFaceMulti` 依赖 `faceHasEdge`（只看顶点下标，不依赖 FF），故删除中途 FF 重算是安全的。

### 5d. 折线连接线性化（3.4）

- `tool/polyline.h` 的 `extendPolyline` 前端插入（`:268`）改为「收集反向链后一次性写回」，避免 `O(n²)`。
- `tryMergePolylines`（`:285-411`）改为按「端点 → 折线」建索引，每轮批量合并、不反复从头重扫，把最坏 `O(n³)` 降到约 `O(n log n)`。

### 5e. `floodFill` 复用访问标记（3.5）

- `tool/region_marker.h` 的 `floodFill`（`:130`）把 `std::vector<bool> visited`（每次分配清零）改为调用方传入的可复用 `std::vector<int>` 代数标记，或 `RegionMarker` 内部持有。

### 测试/验证

- 不新增功能断言；建议加一个可选的大网格基准（如 `BuildTempGridMesh(64, ...)`）打印耗时，人工对比改动前后，不进 CI 断言。

---

## 7. 步骤 6：代码清理（4.x）

### 6a. 删除旧 `cutRegion` 链路（4.1）

删除 `tool/local_mesh_cut.h` 中不再被主流程调用的成员：
`LocalMesh` 结构、`CutInput`、`MergeResult`、`extractLocalMesh`、`buildCutInput`、`markCutEdges`、`mergeBack`、`propagateExternal`、`propagateLocalRegionMarks`、`rebuildCurFaces`、`finalizeGrow`、`faceHasNewVert`、`pointOnSegment`、`cutRegion`。

保留（`stitchAllSeams` 仍依赖）：`splitExternalFaceMulti`、`faceHasEdge`、`prepareLocalCut`、`mergeLocalCut`、`stitchAllSeams`。

同时删除 `JasMeshMarkAndCutSplit.cpp:2` 对 `JasMeshLocalMarkAndCutSplitInternal.h` 的旧路径依赖检查，以及 `AddCutLinesBatch` 旧重载在子模块中仅被旧路径使用的部分（视子模块自身测试决定去留）。

### 6b. 删除死代码（4.2）

- `tool/cut_plane.h`（`CutPlaneManager`）整体删除；对应测试 `testMakeCutPlane`、`testMakeCutPlaneLongPolyline`、`testIsOnMarkDiffEdge`、`testSignedDistanceAndIntersection` 删除。
- `JasMeshMarkAndCutSplit.cpp:281-295` 的 `debugWriteSubRegionsOFF` 删除（含 `.h` 声明）。
- `JasMeshMarkAndCutSplit.h:104` 的 `m_edgeMarks` 删除。
- `tool/edge_info.h` 的 `getAdjacentFaces` 保留与否：当前仅测试用，若删则同步删测试（低优先，可保留并加 `[[maybe_unused]]` 说明）。

### 6c. 测试框架（4.3，见步骤 7）

---

## 8. 步骤 7：测试加固（4.3）

### 7a. Release 安全的断言

在 `tests/test_mesh_cut.cpp` 顶部加一个在 Release 下也生效的宏（替代裸 `assert`）：

```cpp
#define REQUIRE(cond) \
    do { if (!(cond)) { \
        std::cerr << "REQUIRE failed: " << #cond << " @ " << __FILE__ \
                  << ":" << __LINE__ << std::endl; \
        std::abort(); \
    } } while (0)
```

将步骤 1-4 新增的关键断言用 `REQUIRE` 书写，确保 MSVC Release（`NDEBUG`）下依然拦截。

（可选、更彻底：迁到子模块已用的 Catch2，与 `cgalLocalMeshCut` 风格一致；代价是主仓库 CMake 引入 Catch2。建议作为独立提交。）

### 7b. 固化的流形校验

把 `CollectTempManifoldStats`（`tests/test_mesh_cut.cpp:1494`）保留，并在 `testNonManifoldEdgeRegion`、`testSeamMultiExternal`、`testExistingVertexReuse` 里用它断言 `nonManifoldEdgeCount`/`coincidentVertexPairCount` 符合预期，而非仅打印。

### 7c. 启用 star vertex 回归

解除 `testTempStarVertexCorefine()` 注释（`tests/test_mesh_cut.cpp:2182`），改为在步骤 4a 完成后，断言「不崩溃 + 面数守恒 + 区域为单个未切区域」，并在 `main()` 恢复调用。

---

## 9. 实施顺序、风险与回滚

建议按 `1 → 2 → 3 → 4a → 4b → 4c → 7(部分) → 5 → 6` 顺序提交，每个步骤一个 commit，且每步测试通过：

1. **步骤 2 最先做**（改动最小、纯局部、立刻修复输出契约）——单文件、低风险。
2. **步骤 1**（含 1a 劈边 + 1b 护栏）——核心，风险最高，需配 `testNonManifoldEdgeRegion`。
3. **步骤 3**（精确缝合）——注意必须同步改 `testStitchAllSeams` 的 `exactPoint`。
4. **步骤 4a/4b/4c**——各自独立，可任意序。
5. **步骤 5/6**——最后，纯优化/清理，不影响输出。

风险与回滚：

- 步骤 1a 劈边较复杂，若实现期发现 corefine 对克隆顶点的 `f:source` 传播有意外，可先只上 1b 护栏（保守：被丢弃面单独/整体赋 newMark），把 1a 拆为后续独立任务，不阻塞正确性护栏。
- 步骤 3 若发现「两侧独立计算出的精确点不严格相等」的几何场景（法向不同导致墙不同），需在缝合处把切点**参数化吸附**到缝边（用 `ExactSeamPoint.t` 对齐）作为补充——此风险已在问题分析中标注，实现时用 `testSeamExactDedup` 验证。
- 每步保留旧函数名/签名兼容，避免一次性大面积破坏编译；步骤 6 的删除放在最后、单独 commit，便于回滚。

---

## 10. 测试用例清单（汇总）

| 测试名 | 文件 | 验证点 |
|---|---|---|
| `testNonManifoldEdgeRegion` | `tests/test_mesh_cut.cpp` | 非流形边区域不丢面、无 newMark=0、mark 正确 |
| `testSplitRegMarkPreserved` | 同上 | `splitReg.mark` == 原始 IMark |
| `testSeamExactDedup` | 同上 | 精确坐标缝合后无裂缝、无重合顶点 |
| `testStitchAllSeams`（改） | 同上 | 补充 `exactPoint` 构造后行为不变 |
| `testStarVertexSkip` | 同上 | star vertex 保守跳过、不崩溃、面守恒 |
| `testSeamMultiExternal` | 同上 | 非流形缝边多外部面全部细分 |
| `testExistingVertexReuse` | 同上 | 经过已有顶点不产生重复顶点 |
| 子模块 `test_cut3d` / `test_add_cut_lines`（可选） | `external/cgalLocalMeshCut/tests/` | `dropped_input_face_count` 的显式语义、劈边后为 0 |
| 基准（可选） | 同上 | 大网格耗时对比 |
