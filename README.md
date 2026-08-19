# MeshCutByLine

将三角形网格按 per-face `mark` 标记分割成**简单多边形区域**的 C++17 库。

输入：每个三角形带 `IMark()`（所属平面/分组标记）的三角网格。
输出：`std::vector<splitReg>` —— 每个区域含原 mark、新 newMark、三角形列表、
法向与全部边界环（第 0 圈外圈、其余为洞）。

实际的局部切割委托给**外部 cutter**：`external/cgalLocalMeshCut`（git submodule），
基于 CGAL corefine 与精确核（EPEC）完成真实切割；本仓库只做编排 ——
提取局部区域 → 切割 → 合并回全局 → 提取子区域 → 输出多边形边界环。

---

## 1. 问题与思路

### 问题

三角形按所属平面做 mark 标记后，直接按 mark 分组提取边界并不能得到简单多边形，
因为真实网格存在：

- **非流形边**：被 3 个及以上三角形共享的边（mark 相同却应分开）；
- **孔洞**：边界不闭合，一个 mark 区域可能是多连通的（带洞）。

### 解决方案：延长线切割

1. 按 mark 收集连通区域的三角形（切割边不可跨越）；
2. 找出区域内的切割边（mark 不同 / 非流形 / 边界）并连接成折线；
3. 非流形折线作为切割线，端点不在区域边界上时向两端延长（区域包围盒对角线长度）；
4. 用切割线切开区域，使每个碎片成为单边界环的简单多边形；
5. 通过 newMark 机制给碎片编号并提取边界。

---

## 2. 总体架构

### 分层

```
┌─────────────────────────────────────────────────────────────┐
│ JasMeshMarkAndCutSplit（主类）                                │
│  Phase 1 边分类 → Phase 2 局部切割/合并/缝合 → Phase 3 提取   │
│  + 跨区域切点共形 conformSharedEdgeCutPoints                  │
├─────────────────────────────────────────────────────────────┤
│ tool/ 编排层（header-only，namespace MeshCutByMark）          │
│  EdgeInfoManager    边→面映射与边分类                         │
│  PolylineManager    切割边 → 折线                             │
│  RegionMarker       flood-fill + newMark 读写                 │
│  LocalMeshCutManager 两条切割路径的 prepare(并行)/merge(串行)  │
│                      + stitchAllSeams 统一缝合                │
├─────────────────────────────────────────────────────────────┤
│ external/cgalLocalMeshCut（submodule，链接 cglmcut）          │
│  jaslmc::CutFacesExact / CreateExactMesh / CutMeshExact      │
│  jaslmc::SubRegionBoundary —— CGAL corefine + 精确核黑盒     │
├─────────────────────────────────────────────────────────────┤
│ CMeshOD（VCGlib TriMesh，OCF 组件，double 标量）              │
└─────────────────────────────────────────────────────────────┘
```

### 文件结构

```
MeshCutByLine/
├── JasMeshMarkAndCutSplit.h      # 主类：splitReg 输出结构、CutPathMode、调试接口
├── JasMeshMarkAndCutSplit.cpp    # 主流程 SplitMeshByMarkAndEdge + 跨区域切点共形 pass
├── tool/                         # 编排层（全部 header-only）
│   ├── edge_info.h               # Phase 1：边→面映射、CutEdgeType 分类
│   ├── polyline.h                # 切割边 → 连续折线（合并/成环/去重）
│   ├── region_marker.h           # flood-fill 连通区域、newMark 容器
│   ├── local_mesh_cut.h          # 两条路径的 prepare/merge + stitchAllSeams
│   ├── local_cut_result.h        # 局部切割结果契约（LocalCutResult / SeamCutLine）
│   ├── polygon_mesh.h            # 多边形路径合并辅助（质心归属、分组环提取）
│   ├── cut_mesh.h                # 外部 cutter 契约入口（仅含 submodule 头）
│   ├── cmesh.h / cmesh.cpp       # CMeshOD 网格类型定义
│   ├── CMeshODStruct.h           # OCF 组件定义
│   └── base_types.h              # 基础类型
├── external/cgalLocalMeshCut/    # git submodule：CGAL corefine 精确切割库
├── tests/test_mesh_cut.cpp       # 测试（37 个，见 §9）
├── vcglib/                       # VCGlib 依赖（含 eigenlib）
├── docs/                         # 历史设计/计划/排错记录（见 §13）
└── CMakeLists.txt
```

### 数据流总览

```
CMeshOD（mark 已设置）
   │
   ▼ Phase 1：buildEdgeInfo（边→面映射 + 分类）
   ▼ Phase 2 阶段1（串行）：floodFill → findCutEdges → connectEdgesToPolylines
       ⇒ regionTasks[]{targetMark, curFaces, polylines}
   ▼ 路由（串行）：按 CutPathMode + 星形顶点/带洞检测 ⇒ 每任务选路径
   ▼ Phase 2 阶段2（并行 OpenMP）：prepareLocalCut[Polygon]（只读全局）
       ⇒ LocalCutResult[]{ExactMesh 切割结果 + 映射}
   ▼ Phase 2 阶段3（串行）：mergeLocalCut[Polygon] 写回全局 + 分配 newMark
   ▼ Phase 2 阶段4（串行，仅旧路径结果）：stitchAllSeams 缝合接缝
   ▼ Phase 3：按 newMark 分组 → polyLoops / SubRegionBoundary 提取边界环
   ▼ conformSharedEdgeCutPoints：共享边切点 splice 进所有输出环
   │
   ▼
std::vector<splitReg>（retRegs）
```

---

## 3. 主流程详解：`SplitMeshByMarkAndEdge`

### Phase 1 —— 初始化与边分类

- `initNewMark`：newMark 容器清零；
- `buildEdgeInfo`（`tool/edge_info.h`）：遍历全部三角形建立「边 → 邻接面」映射，
  分类每条边：

```cpp
enum CutEdgeType {
    CUT_EDGE_NONE,           // 普通边
    CUT_EDGE_MARK_DIFF,      // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD,   // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY        // 只被 1 个三角形使用（孔洞/外边界）
};
```

- 确保 FF 邻接可用、重算拓扑与面法向。

### Phase 2 阶段 1（串行）—— 收集区域任务

对每个未访问面 flood-fill（同 mark 且**不跨越切割边**），得到连通面集 `curFaces`；
`findCutEdges(curFaces)` 找切割边，`connectEdgesToPolylines` 连成折线：

- **MARK_DIFF + BOUNDARY** 折线互相合并（可闭合成环）—— 它们只定义区域边界；
- **NON_MANIFOLD** 折线独立保留 —— 它们是唯一的**切割线**。

### 路由（串行）—— 每任务选择切割路径

```
usePolygonPath = polygonMode                        // SetCutPathMode 选定
              && !hasGlobalStarVert                 // 全局无非流形（星形）顶点
              && extractBoundaryEdges(curFaces).size() == 1   // 区域单边界环（不带洞）
```

多边形路径的两个正确性回退：命中即该区域走三角形路径（见 §4）。

### Phase 2 阶段 2（并行，`#pragma omp parallel for`）—— 局部独立切割

只读全局网格，写入各自独立的 `LocalCutResult`，无任何全局写：

- **三角形路径** `prepareLocalCut` → `jaslmc::CutFacesExact`：
  从全局面集直接构建 `ExactMesh`（面带 `f:mark`/`f:global`，顶点带 `v:g`），
  逐条切割线 CGAL corefine，按「切割边不可跨越」重新分区 `f:mark`，
  收集缝边（seam 边 + 对侧外部邻接面）；
- **多边形路径** `prepareLocalCutPolygon` → `jaslmc::CreateExactMesh` +
  `jaslmc::CutMeshExact`：把区域边界环围成**单个 n 边形**再逐线切割，
  结果在 `exact.polyResult`（无 `f:global`/`v:g`、无缝边）。

两条路径的切割线构造相同：取 NON_MANIFOLD 折线顶点的精确坐标 `ExactPoint`，
端点不在区域边界顶点集内时按区域包围盒对角线长度向两端延长；
法向取区域首面法向。

### Phase 2 阶段 3（串行）—— 写回全局并分配 newMark

先构建全网格「精确坐标 → 全局顶点」索引 `existingPointToVertex`（跨区域复用，
后续每次追加顶点同步更新）—— 切割线经过已有顶点时复用，避免坐标相同、
索引不同的重复顶点。然后逐结果合并：

- `mergeLocalCut`（三角形路径）：新顶点 append、被切原面槽位用第一个分片
  原位重写、其余分片 append，`IMark = f:mark`；缝边切点映射为全局顶点；
  局部 mark → 全局 newMark（跳过的任务整区一个 newMark）；
- `mergeLocalCutPolygon`（多边形路径）：**不切分、不改写任何全局面**，
  只做三件事 ——
  1. polyResult 顶点按精确坐标映射到全局顶点，切点按需追加为**孤立顶点**
     （不被任何面引用，记录进 `orphanCutPoints`），使边界环可引用真实全局下标；
  2. 区域原始面按**质心落在哪个分片**（精确 point-in-polygon）分配给各组，
     划分保证每个面恰好属于一个 newMark，绝不产生 newMark=0 孤儿面；
  3. 空组不分配 newMark（窄片丢弃），记录各 newMark 的边界环进 `polyLoops`。

### Phase 2 阶段 4（串行，仅存在旧路径结果时）—— 统一缝合

`stitchAllSeams`：按精确坐标合并缝边两侧切点为同一全局顶点；
对未切侧的外部邻接面做**纯分割**（`splitExternalFaceMulti`，同缝边多顶点
一次分割成 n+1 个子面，缝边→邻居映射同步更新，后续切点继续传播）；
端点重合的新顶点跳过。随后重算 FF 拓扑与法向（多边形路径网格面未动，重算无害）。

### Phase 3 —— 提取多边形

1. 按 newMark 分组全部存活面；防御：任何 newMark==0 的面逐面分配递增 newMark，
   绝不合并成一个假区域；
2. 每组构造 `splitReg`：
   - 边界环优先用多边形路径合并阶段记录的 `polyLoops`（含切点孤立顶点），
     否则在全局网格上 `jaslmc::SubRegionBoundary` 提取；
   - 法向取组内三角形法向累加后归一化（与内部三角形方向一致）；
   - `boundlines = boundaries[0]`（外圈，向后兼容），完整环在 `boundaries`；
3. **跨区域切点共形** `conformSharedEdgeCutPoints`（输出循环之后）：
   多边形路径各区域独立切割，切点只出现在「拥有该切割线的区域」的环上 ——
   邻域不细分共享边（T 形结）；两侧独立计算的切点精确坐标又不相等。
   该 pass 分三步修复：
   1. **定位**：切点孤立顶点匹配到所属区域边界边，旧路径缝点登记缝边
      （兼容混合模式）→「边 → 切点集」；
   2. **规范化**：同边近重合切点（距离平方 < 1e-12）合并保留下标最小者，
      其余重定向，未被面引用的重复孤立顶点删除；
   3. **splice**：规范切点按精确参数顺序插入所有经过其所在线段的输出环
      （整边或已细分子段），邻接区域由此共享同一顶点细分序列。
   纯旧路径运行时切点已是环上顶点、严格内部判定不会重复插入 —— 天然幂等。

---

## 4. 两条切割路径

外部参数 `SetCutPathMode` 选择（默认 `CUT_PATH_POLYGON`）；两条路径输出相同的
`retRegs`（`splitReg` 列表），仅切割策略不同：

| | 三角形路径 `CUT_PATH_TRIANGLE` | 多边形路径 `CUT_PATH_POLYGON`（默认） |
|---|---|---|
| 并行 prepare | `prepareLocalCut` → `CutFacesExact`（corefine 真切三角形） | `prepareLocalCutPolygon` → 边界环围成单 n 边形 + `CutMeshExact`（2D 多边形分割） |
| 串行 merge | `mergeLocalCut`：顶点 append、面重写/追加 | `mergeLocalCutPolygon`：切点追加为孤立顶点，**面完全不动** |
| 接缝处理 | `stitchAllSeams`（缝点合并 + 外部面纯分割） | 无缝；`conformSharedEdgeCutPoints` 统一切点 |
| 全局网格 | 面被真实切分，面数增长 | 面数不变 |
| 精度 | 精确切割、精确归属 | 近似：跨越三角形按质心归属分片，窄片可能丢失 |

**多边形路径的回退**（命中时该区域自动走三角形路径保证正确性）：

1. **全局星形顶点**（非流形顶点，其面扇不连通）：corefine 会崩溃，保守处理；
2. **区域带洞**（`extractBoundaryEdges` 得到多个边界环）：`CreateExactMesh`
   只追踪一个边界环，围 n 边形会把洞填上。

---

## 5. 关键数据结构

### 输出：`splitReg`（JasMeshMarkAndCutSplit.h）

```cpp
struct splitReg {
    int mark;                    // 原始平面标记（= 组内面的原始 IMark）
    int newMark;                 // 新标记（简单多边形 ID）
    std::vector<int> inTris;     // 包含的三角形索引
    vcg::Point3d normal;         // 法向量（组内法向累加归一化）
    std::vector<int> boundlines; // 边界顶点索引序列（第 0 圈，向后兼容）
    std::vector<std::vector<int>> boundaries; // 全部边界环（第 0 圈外圈，其余为洞）
};
```

### 局部切割结果：`LocalCutResult`（tool/local_cut_result.h）

并行阶段产出、串行阶段消费的契约：

```cpp
struct LocalCutResult {
    std::vector<int> faceGlobals;   // 该局部单元的全局面下标
    int targetMark;                 // 区域原 mark
    jaslmc::ExactCutResult exact;   // 切好且分区好的局部 ExactMesh 与映射
    std::vector<SeamCutLine> seams; // 三角形路径：映射后的全局缝边切点表

    bool usePolygonPath = false;    // 路由结果
    // 多边形路径：追加的切点孤立顶点（全局下标 + 精确坐标），供共形 pass 使用
    std::vector<std::pair<int, jaslmc::ExactPoint>> orphanCutPoints;
    // 多边形路径：newMark -> 边界环（全局顶点下标）；空则 Phase 3 回退 SubRegionBoundary
    std::map<int, std::vector<std::vector<int>>> polyLoops;
};
```

### 折线：`Polyline`（tool/polyline.h）

`type`（CutEdgeType，合并后为 MARK_DIFF/BOUNDARY 或 NON_MANIFOLD）+
`vertexIndices`（有序顶点序列）。仅 NON_MANIFOLD 折线成为切割线。

---

## 6. 外部 cutter 契约（submodule）

本仓库 `tool/cut_mesh.h` 只包含 `JasMeshAddCutLines.h`，实现链接自 `cglmcut`。
用到的 `jaslmc` 命名空间接口：

| 接口 | 路径 | 作用 |
|---|---|---|
| `CutFacesExact(mesh, curFaces, normals, lines, result)` | 三角形 | 从全局面集构建 ExactMesh（`f:mark`/`f:global`/`v:g`），corefine 逐线切割并重分区，收集缝边；非流形边进 corefine 前被劈开，`dropped_input_face_count` 护栏 |
| `CreateExactMesh(mesh, curFaces, retMesh)` | 多边形 | 区域面集边界环 → 单个 n 边形 ExactMesh |
| `CutMeshExact(exMesh, normals, cutLines, result)` | 多边形 | 逐线以首末点弦向平面切 n 边形，输出 `polyResult`（2D 多边形分片，含 mark） |
| `SubRegionBoundary(mesh, subregion)` | Phase 3 | 子区域边界环提取，第 0 圈外圈、其余为洞 |

cutter 契约要点（2026-08 加固）：corefine 输出面带 `f:source` 父面属性
（visitor 传播）使分片直接归属父面；零面积 sliver 折叠（而非跳过）；
每刀后局部 mesh 保持流形（无重复顶点/非流形边/非流形点）—— CGAL corefine
在非流形输入上不可靠（非流形边被 `add_face` 拒收丢面，非流形点直接崩溃）。

---

## 7. 精确计算与流形保障约定

- **全程精确坐标**：切割线、切点、去重比较都用 EPEC 精确核
  （`jaslmc::ExactPoint` / `Kernel::FT`）；double 仅用于延长量计算与粗筛。
- **切点去重两级**：精确相等（`existingPointToVertex` 索引复用）+ 近重合
  （共形 pass 距离平方 < 1e-12，与 `CutMeshExact` 重合判定同容差）。
- **流形保障**：零面积分片折叠、新顶点按精确坐标合并、每刀后清理孤立顶点；
  seam 传播为纯分割，端点重合跳过。
- **面数守恒护栏**：`mergeLocalCut` 的 dropped_faces 防御 + Phase 3 的
  newMark==0 孤儿面逐面兜底，任何面都不会凭空消失或落入假区域。

---

## 8. 编译与运行

### 依赖

- **VCGlib**：已 vendored 在 `vcglib/`（含 `eigenlib/`）
- **cgalLocalMeshCut**：git submodule，提供 CGAL corefine 切割黑盒
- **CGAL**（如 6.1.1）：配置时 `-DCGAL_DIR` 指定
- **Boost**：CGAL 头文件依赖（配置前设 `BOOST_ROOT`）
- **GMP/MPFR**：CGAL 附带动态库（`${CGAL_DIR}/auxiliary/gmp`，运行时加入 PATH）
- **CMake 3.18+**（submodule 的 CMakeLists 要求）、**OpenMP**、C++17 编译器

### 步骤（Windows / MSVC）

```powershell
# 1. 初始化 submodule
git submodule update --init --recursive

# 2. 配置（BOOST_ROOT 必须在配置前设置）
$env:BOOST_ROOT = 'D:\github\boost_1_91_0'
cmake -S . -B build -G "Visual Studio 17 2022" -DCGAL_DIR=D:/github/CGAL-6.1.1

# 3. 构建 + 运行测试（GMP/MPFR DLL 需在 PATH）
cmake --build build --config Release --target test_mesh_cut
$env:PATH = 'D:\github\CGAL-6.1.1\auxiliary\gmp\bin;' + $env:PATH
.\build\Release\test_mesh_cut.exe
```

> 也可用 Ninja：`-G Ninja -DCMAKE_BUILD_TYPE=Debug`（需 MSVC 环境）。
> MSVC 已设 `/utf-8`（中文注释）；CGAL 模板重代码需要 `/bigobj`（已在 cglmcut 设置）。

### Linux/macOS

```bash
git submodule update --init --recursive
cmake -S . -B build && cmake --build build -j
./build/test_mesh_cut
```

---

## 9. 测试

测试文件 `tests/test_mesh_cut.cpp`：37 个测试，`main()` 顺序执行，
raw `assert` + `REQUIRE` 宏（失败打印表达式与行号后 abort）。

> **注意**：测试文件在 `<cassert>` 前 `#undef NDEBUG` —— Release CMake 会
> void 掉所有 assert，否则套件形同虚设（曾因此漏掉多边形路径回归）。

### tool 层单元测试

| 测试 | 说明 |
|---|---|
| `testEdgeHash` | 边哈希函数对称性 |
| `testBuildEdgeInfo` | 边信息构建（mark 不同、边界边、邻接面查询） |
| `testFindCutEdges` | 切割边查找（边界边、非流形边计数） |
| `testConnectEdgesToPolylines` | 单条折线连接 |
| `testConnectEdgesToPolylinesMultiple` | 多条折线连接 |
| `testConnectEdgesToPolylinesDeduplicate` | 重复边去重与折线防回头 |
| `testConnectEdgesToPolylinesEmpty` | 空输入 |
| `testFloodFill` / `…MarkDiff` / `…Boundary` | flood-fill 连通（同 mark、mark-diff 边、边界边不可跨越） |
| `testExtractSubRegions` / `testMarkSubRegions` / `testInitNewMark` | 子区域提取/标记/初始化 |
| `testExtractBoundaryEdges` / `…TwoTriangles` | 区域边界边提取 |
| `testGrowNewMark` | newMark 容器扩容 |

### 集成与管线回归（真实 cutter）

| 测试 | 说明 |
|---|---|
| `testSplitMeshByMarkAndEdge` / `…SameMark` / `testIntegration` | 全管线端到端（不同/相同 mark、综合场景） |
| `testSplitMeshNormals` | 输出多边形法向与内部三角形一致 |
| `testCutRegionNonManifoldEdgeStability` | 真实非流形边（3 面共边）不崩溃、不丢已存在面 |
| `testPrepareLocalCutMarks` | prepareLocalCut 后局部 ExactMesh 按切割边分区标记 |
| `testCutFacesExact` | CutFacesExact 从全局面集构建/切割/分区/收集缝边 |
| `testCutFacesExactDropRecording` | 非流形边进 corefine 前被劈开，不因 `add_face` 拒收丢面 |
| `testNonManifoldEdgeRegion` | 含非流形边网格整体分割：无 newMark=0 垃圾区、面数守恒、mark 一致 |
| `testSplitRegMarkPreserved` | splitReg.mark 等于原始 IMark，不被局部重标污染 |
| `testStarVertexSkip` | star vertex 区域保守跳过，全管线面数守恒 |
| `testStitchAllSeams` | 统一缝合合并两侧缝边切点，消除双侧裂缝 |
| `testSeamExactDedup` | 精确坐标缝合后无「坐标相等索引不同」的顶点对 |
| `testSeamMultiExternal` | 非流形缝边的两个外部邻接面都被纯分割 |
| `testSeamRedirectAfterSplit` | 同一外部面被两条缝边引用时，拆分后第二条缝边重定向到子面 |
| `testExistingVertexReuse` | 切割精确点已存在全局顶点时复用（不重复追加） |
| `testComplexCutStress` | 压力冒烟：复杂多折线连续切割后保持流形 |

### 多边形切割路径

| 测试 | 说明 |
|---|---|
| `testPolygonPathMergeDirect` | prepareLocalCutPolygon + mergeLocalCutPolygon 直连契约：不切分/不复制面、无孤儿 newMark、切点以孤立顶点追加、边界环引用有效全局下标 |
| `testPolygonPathGridCut` | 全管线：面数不变、区域数等于任务数、inTris 恰好划分全部存活面 |
| `testCutPathModeSelect` | 同一拓扑双路径对比：输出契约相同，网格改写差异符合预期 |
| `testPolygonPathSharedEdgeConformity` | 跨区域切点共形：共享 mark-diff 边上单侧切点（T 形结）与双侧近重合切点，splice 后所有触及该边的环共享同一顶点细分序列 |

---

## 10. 使用示例

```cpp
#include "JasMeshMarkAndCutSplit.h"

CMeshOD mesh;
// ... 加载或创建网格，启用 FF/Mark/顶点 Mark、UpdateTopology/UpdateNormal ...

// 设置 mark（每个三角形的平面 ID）
mesh.face[0].IMark() = 1;
mesh.face[1].IMark() = 1;
mesh.face[2].IMark() = 2;

JasMeshMarkAndCutSplit splitter;
splitter.SetMainMesh(&mesh);
// 可选：选择切割路径（默认 CUT_PATH_POLYGON，不切分原始三角形）
splitter.SetCutPathMode(JasMeshMarkAndCutSplit::CUT_PATH_POLYGON);

std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
splitter.SplitMeshByMarkAndEdge(regions);

for (const auto& region : regions) {
    std::cout << "mark=" << region.mark << " newMark=" << region.newMark
              << " tris=" << region.inTris.size()
              << " loops=" << region.boundaries.size() << std::endl;
    // region.boundaries[0] 为外圈环（= region.boundlines），其余为洞
}
```

---

## 11. 调试输出

```cpp
splitter.SetDebug(true);                        // 开启
splitter.SetDebugOutputDir("my_debug_dir/");    // 可选，默认 debug_output/
```

| 文件 | 格式 | 内容 | 输出时机 |
|---|---|---|---|
| `iter_N_cur_faces.off` | OFF | flood-fill 得到的连通区域面集 | 阶段 1 每个区域 |
| `iter_N_polylines.obj` | OBJ（`l` 元素） | 切割边连接成的折线 | 阶段 1 每个区域 |
| `final_polygons.obj` | OBJ（`f` 元素） | 各区域边界多边形 | Phase 3 后 |
| `colored_mesh.obj` + `.mtl` | OBJ+MTL | 按 newMark 面着色的完整网格（面级 `usemtl`） | Phase 3 后 |

`N` 为区域任务编号。颜色由 `std::map<int, vcg::Color4b>` 管理，
key 为 newMark，每个区域随机配色。

---

## 12. 已知限制

1. **外部 cutter 为保守黑盒**：退化切割线（沿边/过顶点/中点落边上）可能 no-op，
   可接受（Phase 2 设计允许）；
2. **corefine 输入必须流形**：CGAL `Surface_mesh` 拒绝非流形边（`add_face`
   失败丢面）；非流形点（star vertex）会崩溃（Debug 断言 / Release 访问冲突）。
   管线相应做了非流形边预劈开、零面积分片折叠、按精确坐标合并新顶点、
   每刀后清理孤立顶点；星形顶点区域保守回退；
3. **缝边传播为纯分割**：同缝边多顶点一次分割并更新「缝边→邻居面」映射，
   端点重合跳过；只分割、不新增边界；
4. **单边界环兼容字段**：`boundlines` 只存第 0 圈，完整环在 `boundaries`；
5. **流形假设**：边界遍历假设网格流形；
6. **多边形路径为近似划分**：不切分原始三角形，跨越三角形按质心归属；
   无完整三角形质心落入的窄片不输出。`CutMeshExact` 只用切割线与多边形的
   **前两个交点**：非凸区域被一条线穿越 4+ 次时分割不正确（submodule 待办），
   丢弃的子多边形（size<3 / `add_face` 拒收）会留下几何空洞（质心回退掩盖）；
7. **多边形路径切点为孤立顶点**：切点不被任何面引用地追加进全局网格，
   网格面数不变；共享 mark-diff 边上的切点由 `conformSharedEdgeCutPoints`
   统一 splice 进所有经过该边的输出环（含旧路径缝点，兼容混合模式），
   近重合切点（距离平方 < 1e-12）规范合并，邻接区域共享同一细分序列；
8. **多边形路径安全回退**：全局星形顶点或区域带洞时该区域走三角形路径；
9. **性能**：折线前插入为 O(n²)（待优化）。

---

## 13. 历史文档

`docs/` 下的设计与排错记录反映当时的实现，代码此后继续演进（以本文档与
git log 为准）：

- `docs/superpowers/specs/2026-07-21-mesh-cut-by-mark-design.md`、
  `docs/superpowers/plans/2026-07-21-mesh-cut-by-mark.md` —— 初版设计/计划；
- `docs/superpowers/specs/2026-07-22-phase2-local-mesh-cut-design.md`、
  `docs/superpowers/plans/2026-07-22-phase2-local-mesh-cut.md` —— Phase 2 设计/计划；
- `docs/local-parallel-cut-merge-design.md` —— 局部并行切割 + 统一缝合设计；
- `docs/problem-analysis.md`、`docs/fix-plan.md`、`docs/experience-debug-output.md`
  —— 排错与修复记录。

---

## 14. 许可证

本项目使用 VCGlib 的 GPL 许可证。

---

## 维护记录

- 2026-07-21：初版（按 mark 分割 + 延长线切割）。
- 2026-08-13：cutRegion 一体化、f:source 分片父面、流形保障、缝边纯分割传播。
- 2026-08-13（codex/local-parallel-cut-merge）：局部独立切割 + 统一缝合两阶段
  （ExactMesh 会话、精确切割线、并行 prepareLocalCut、stitchAllSeams）。
- 2026-08：多边形切割路径（prepareLocalCutPolygon + mergeLocalCutPolygon）、
  安全回退（星形顶点/带洞）、面数守恒护栏。
- 2026-08-19：新增跨区域切点共形 `conformSharedEdgeCutPoints`
  （共享边切点 splice，消除 T 形结）；README 按当前代码全量重写。
