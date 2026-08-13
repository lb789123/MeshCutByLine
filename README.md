# MeshCutByMark

将三角形网格按 mark 标记分割成简单多边形区域的 C++ 库。

## 项目背景

### 问题描述

在网格处理中，三角形通常按所属平面进行 mark 标记。我们需要从这些标记的三角形中提取出**简单多边形**（不自交、闭合的多边形边界）。

### 遇到的挑战

由于网格存在以下问题，直接按 mark 分组提取边界会导致多边形异常：

- **非流形边**：被 3 个或更多三角形共享的边
- **孔洞**：边界不闭合

### 解决方案

采用**延长线切割**算法：

1. 按 mark 收集连通区域的三角形
2. 找到区域内的切割边（mark 不同的边、非流形边、边界边）
3. 将切割边连接成连续折线
4. 从折线端点构造切割平面，切割区域
5. 通过新标记机制得到简单多边形

---

## 核心功能

| 功能 | 说明 |
|------|------|
| 边信息管理 | 构建边→面映射，分类边类型（mark 不同、非流形、边界） |
| 切割边查找 | 从连通区域中找到所有切割边 |
| 折线连接 | 将零散的切割边连接成连续折线 |
| 切割平面构造 | 从折线端点构造切割平面 |
| 区域标记 | flood-fill 找连通区域；AddCutLines 按“切割边不可跨越”重标 local 区域，cutRegion 同步为全局 new-mark |
| 边界边提取 | 从区域中提取边界边序列 |

---

## 文件结构

```
MeshCutByLine/
├── tool/
│   ├── edge_info.h          # 边信息数据结构
│   ├── polyline.h           # 折线连接
│   ├── cut_plane.h          # 切割平面构造
│   ├── region_marker.h      # 区域标记
│   ├── local_mesh_cut.h     # 局部 mesh 切割管线（LocalMeshCutManager）
│   ├── cut_mesh.h           # 外部 cutter 契约（引用 cgalLocalMeshCut）
│   └── cmesh.h              # VCGlib 网格类型定义
├── JasMeshMarkAndCutSplit.h    # 主头文件
├── JasMeshMarkAndCutSplit.cpp  # 主实现
├── external/
│   └── cgalLocalMeshCut/    # 外部局部切割库（git submodule，CGAL corefine）
├── tests/
│   └── test_mesh_cut.cpp    # 测试文件
├── vcglib/                  # VCGlib 依赖库
├── CMakeLists.txt           # CMake 构建文件
└── README.md                # 本文件
```

---

## 算法流程

```
Phase 1: 构建边信息
    遍历所有三角形，建立边→面的映射
    分类每条边：mark不同 / 非流形 / 边界 / 普通

Phase 2: 延长线切割并标记新区域
    for each 未标记新 mark 的面 i:
        curFaces = floodFill(i, sameMark, notCrossCutEdge)
        cutEdges = findCutEdges(curFaces)
        
        // 将切割边连接成连续折线
        polylines = connectEdgesToPolylines(cutEdges)
        
        // cutRegion：提取局部 mesh → 逐条 NON_MANIFOLD 折线延长切割
        // （外部 cutter：JasMeshAddCutLines::AddCutLines，cgalLocalMeshCut
        // submodule，CGAL corefine）→ 合并回主网格 → 同步区域标记
        LocalMeshCutManager::cutRegion(curFaces, polylines, targetMark)

    cutRegion 内部：
        1) extractLocalMesh       提取区域局部 mesh 并记录缝边 seamExternal
        2) 逐条 NON_MANIFOLD 折线：悬空端点两端延长 → buildCutInput →
           AddCutLines（corefine 切局部 mesh，按“切割边不可跨越”直接重标
           区域；分片按 f:source 归属父面，零面积分片折叠，清理孤立顶点）
        3) mergeBack              原位改写被切原面 + append 新分片（不 SetD）
        4) propagateExternal      缝边新顶点传播到外部邻接面（纯分割：
           同缝边多顶点一次分割，分割后更新缝边→邻居面映射，端点重合跳过）
        5) finalizeGrow           扩容 newMark 存储 + 重算 FF/法向
        6) propagateLocalRegionMarks  把 local 区域标记同步为全局 new-mark
        7) rebuildCurFaces        重建 curFaces

Phase 3: 根据新标记提取多边形
    按新标记分组所有三角形
    对每组提取边界边 → 形成简单闭合多边形
```

---

## 数据结构

### 切割边类型

```cpp
enum CutEdgeType {
    CUT_EDGE_NONE,           // 普通边（非切割边）
    CUT_EDGE_MARK_DIFF,      // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD,   // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY        // 只被 1 个三角形使用（孔洞边缘）
};
```

### 输出结构

```cpp
struct splitReg {
    int mark;                    // 原始平面标记
    int newMark;                 // 新标记（简单多边形 ID）
    std::vector<int> inTris;     // 包含的三角形索引
    vcg::Point3d normal;         // 法向量
    std::vector<int> boundlines; // 边界边的顶点索引序列
    std::vector<std::vector<int>> boundaries; // 全部边界环（第 0 圈外圈，其余为洞）
};
```

---

## 编译环境

### 依赖

- **VCGlib**：已包含在 `vcglib/` 目录
- **Eigen**：已包含在 `vcglib/eigenlib/` 目录
- **cgalLocalMeshCut**：`external/` 下的 git submodule（GitHub），提供局部切割黑盒
  `JasMeshAddCutLines::AddCutLines`（基于 CGAL corefine）
- **CGAL 6.1.1**：`D:\github\CGAL-6.1.1`（配置时 `-DCGAL_DIR` 指定）
- **Boost**：CGAL 头文件依赖（如 `D:\github\boost_1_91_0`，配置前设置 `BOOST_ROOT`）
- **GMP/MPFR**：CGAL 附带（`${CGAL_DIR}/auxiliary/gmp`，运行测试需把 `bin` 加入 PATH）
- **CMake**：版本 3.15+
- **C++ 编译器**：支持 C++17（MSVC 2022、GCC 9+、Clang 10+）

### 初始化 submodule

```bash
git submodule update --init --recursive
```

### Windows（MSVC）

```bash
$env:BOOST_ROOT = 'D:\github\boost_1_91_0'   # 必须在配置前设置
cmake -S . -B build -G "Visual Studio 17 2022" -DCGAL_DIR=D:/github/CGAL-6.1.1
cmake --build build --config Release --target test_mesh_cut

# 运行测试（GMP/MPFR 为动态库，需加入 PATH）
$env:PATH = 'D:\github\CGAL-6.1.1\auxiliary\gmp\bin;' + $env:PATH
.\build\Release\test_mesh_cut.exe
```

> 也可使用 Ninja 生成器：`-G Ninja -DCMAKE_BUILD_TYPE=Debug`（需 MSVC 环境，且
> 用 Ninja 的 `-DCMAKE_MAKE_PROGRAM` 指向 VS 自带的 ninja.exe）。

### Linux/macOS

```bash
# 创建构建目录
mkdir build
cd build

# 生成 Makefile
cmake ..

# 编译
make -j$(nproc)

# 运行测试
./test_mesh_cut
```

---

## 测试用例

测试文件：`tests/test_mesh_cut.cpp`

### 单元测试

| 测试名称 | 说明 |
|----------|------|
| `testEdgeHash` | 验证边哈希函数的对称性 |
| `testBuildEdgeInfo` | 验证边信息构建（mark 不同、边界边） |
| `testFindCutEdges` | 验证切割边查找（边界、非流形） |
| `testConnectEdgesToPolylines` | 验证单条折线连接 |
| `testConnectEdgesToPolylinesMultiple` | 验证多条折线连接 |
| `testConnectEdgesToPolylinesDeduplicate` | 验证重复边去重与折线防回头 |
| `testConnectEdgesToPolylinesEmpty` | 验证空输入 |
| `testMakeCutPlane` | 验证切割平面构造 |
| `testMakeCutPlaneLongPolyline` | 验证长折线的平面构造 |
| `testIsOnMarkDiffEdge` | 验证 mark 不同边检测 |
| `testSignedDistanceAndIntersection` | 验证有符号距离和交点 |
| `testFloodFill` | 验证 flood-fill 连通区域查找 |
| `testFloodFillMarkDiff` | 验证不同 mark 的 flood-fill |
| `testFloodFillBoundary` | 验证边界边的 flood-fill |
| `testExtractSubRegions` | 验证子区域提取 |
| `testMarkSubRegions` | 验证子区域标记 |
| `testInitNewMark` | 验证新标记初始化 |
| `testExtractBoundaryEdges` | 验证单三角形边界提取 |
| `testExtractBoundaryEdgesTwoTriangles` | 验证两三角形边界提取 |
| `testExtractLocalMesh` | 验证局部 mesh 提取（顶点/面映射、Nv0） |
| `testBuildCutInput` | 验证延长段 line + normal 构造 |
| `testMergeBack` | 验证合并回主网格（原位改写、不 SetD） |
| `testMergeBackSharedEdge` | 验证共享边场景的合并 |
| `testMarkCutEdges` | 验证切割边标为 FFp 自指屏障 |
| `testPropagateExternal` | 验证缝边新顶点传播到外部邻接面 |
| `testGrowNewMark` | 验证 newMark 容器扩容 |
| `testRebuildCurFaces` | 验证 curFaces 重建 |
| `testCutRegionPlumbing` | Phase 2.4 管线冒烟（真实 cutter） |
| `testCutRegionManyComplexPolylines` | 回归：多条复杂折线连续切割后网格保持流形（无重复/退化面、无非流形边/点、无重合顶点） |
| `testCutRegionNonManifoldEdgeStability` | 回归：真实非流形边下 f:source 路径不崩溃、不丢已存在面 |
| `testPropagateExternalCoincident` | 回归：缝边新顶点与端点重合时跳过分割（不产生退化面/重合顶点） |
| `testSeamPropagation` | 回归：缝边切割新顶点在外部邻接面上被加点细分，同缝边多顶点与角场景均无裂缝 |

### 集成测试

| 测试名称 | 说明 |
|----------|------|
| `testSplitMeshByMarkAndEdge` | 验证不同 mark 的分割 |
| `testSplitMeshByMarkAndEdgeSameMark` | 验证相同 mark 的分割 |
| `testIntegration` | 端到端集成测试 |

### 测试输出示例

```
testEdgeHash passed
testBuildEdgeInfo passed
testFindCutEdges passed
testConnectEdgesToPolylines passed
testConnectEdgesToPolylinesMultiple passed
testConnectEdgesToPolylinesEmpty passed
testMakeCutPlane passed
testMakeCutPlaneLongPolyline passed
testIsOnMarkDiffEdge passed
testSignedDistanceAndIntersection passed
testFloodFill passed
testFloodFillMarkDiff passed
testFloodFillBoundary passed
testExtractSubRegions passed
testMarkSubRegions passed
testInitNewMark passed
testExtractBoundaryEdges passed
testExtractBoundaryEdgesTwoTriangles passed
testExtractLocalMesh passed
testBuildCutInput passed
testMergeBack passed
testMergeBackSharedEdge passed
testMarkCutEdges passed
testPropagateExternal passed
testGrowNewMark passed
testRebuildCurFaces passed
testCutRegionPlumbing passed
testSplitMeshByMarkAndEdge passed
testSplitMeshByMarkAndEdgeSameMark passed
testIntegration passed

All 35 tests passed!
```

---

## 使用示例

```cpp
#include "JasMeshMarkAndCutSplit.h"

// 创建网格
CMeshOD mesh;
// ... 加载或创建网格 ...

// 设置 mark（每个三角形的平面 ID）
mesh.face[0].IMark() = 1;
mesh.face[1].IMark() = 1;
mesh.face[2].IMark() = 2;

// 创建分割器
JasMeshMarkAndCutSplit splitter;
splitter.SetMainMesh(&mesh);

// 执行分割
std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
splitter.SplitMeshByMarkAndEdge(regions);

// 使用结果
for (const auto& region : regions) {
    std::cout << "Mark: " << region.mark << std::endl;
    std::cout << "New Mark: " << region.newMark << std::endl;
    std::cout << "Triangles: " << region.inTris.size() << std::endl;
    std::cout << "Boundary vertices: " << region.boundlines.size() << std::endl;
}
```

---

## 调试输出

当需要可视化算法中间结果时，可开启调试模式。调试模式会在算法各阶段输出中间数据到文件。

### 开启调试

```cpp
JasMeshMarkAndCutSplit splitter;
splitter.SetMainMesh(&mesh);
splitter.SetDebug(true);                          // 开启调试输出
splitter.SetDebugOutputDir("my_debug_dir/");      // 可选：自定义输出目录（默认 debug_output/）
```

### 输出文件

调试模式会在 `debug_output/` 目录下生成以下文件：

| 文件 | 格式 | 内容 | 输出时机 |
|------|------|------|----------|
| `iter_N_cur_faces.off` | OFF | flood-fill 得到的连通区域 | Phase 2 每次迭代，步骤 2.1 后 |
| `iter_N_polylines.obj` | OBJ | 折线（cut edges 连接成的连续折线） | Phase 2 每次迭代，步骤 2.3 后 |
| `final_polygons.obj` | OBJ | 各区域的边界多边形 | Phase 3 结束后 |
| `colored_mesh.obj` + `colored_mesh.mtl` | OBJ+MTL | 按 NewMark 面颜色的完整网格（面级 usemtl 材质） | Phase 3 结束后 |

其中 `N` 是主循环迭代编号，`J` 是子区域编号。

### 文件格式说明

**OFF 格式**（三角形网格）：
```
OFF
顶点数 面数 0
x0 y0 z0
x1 y1 z1
...
3 v0 v1 v2
3 v3 v4 v5
...
```

**OBJ 格式 - 折线**（`l` 元素）：
```
v x0 y0 z0
v x1 y1 z1
...
l 1 2 3 4
```

**OBJ 格式 - 多边形**（`f` 元素）：
```
# newMark = 1
v x0 y0 z0
v x1 y1 z1
...
f 1 2 3 4
```

**OBJ 格式 - 带颜色网格**（面颜色，OBJ + MTL）：

`colored_mesh.obj`：
```
mtllib colored_mesh.mtl
v x y z
v x y z
...
usemtl newmark_1
f v1 v2 v3
```

`colored_mesh.mtl`：
```
newmtl newmark_1
Kd 0.29 0.33 0.32
```

颜色通过 `std::map<int, vcg::Color4b>` 管理，key 为 NewMark，每个区域分配随机颜色；
面通过 `usemtl` 引用材质着色，不再输出顶点颜色。

---

## 已知限制

1. **Phase 2.4 延长切割依赖外部 cutter**：真实 cutter（cgalLocalMeshCut submodule，CGAL corefine）为保守黑盒，切割线退化（沿边/过顶点/中点落边上）时可能 no-op，可接受
2. **corefine 输入必须流形**：CGAL `Surface_mesh` 拒绝非流形边（`add_face` 失败丢面，`Cut3D` 计数）；非流形点（star vertex）会导致 corefine 断言崩溃（Debug）或访问冲突（Release）。因此 `cutRegion` 每刀后折叠零面积分片、按精确坐标合并新顶点、清理孤立顶点，保障局部 mesh 流形
3. **缝边传播为纯分割**：`propagateExternal` 把缝边上的新顶点传播到外部邻接面，同缝边多顶点一次分割并更新「缝边→邻居面」映射，端点重合跳过；只分割、不新增边界
4. **单边界环**：`boundlines` 只存储第一个边界环，多孔区域会丢失信息（完整环在 `splitReg::boundaries`）
5. **流形假设**：边界遍历假设网格是流形的

---

## 未来改进

- [x] Phase 2.4 延长切割：局部 mesh + 外部 cutter 管线，并接入真实 CGAL corefine
- [x] 多折线连续切割流形保障：f:source 分片父面归属、零面积分片折叠、孤立顶点清理
- [x] 缝边传播：propagateExternal 纯分割 + 邻居映射更新，接缝无裂缝
- [ ] 支持多孔区域的边界提取
- [ ] 非流形输入（非流形边/点）的预处理：劈开非流形边后再进 corefine
- [ ] 优化性能（O(n²) 前插入 → O(n)）
- [ ] 添加更多边界测试用例
- [ ] 使用测试框架替代 raw assert

---

## 许可证

本项目使用 VCGlib 的 GPL 许可证。

---

## 作者与维护

Claude (Anthropic) - 2026-07-21

2026-08-13：按当前代码同步更新（cutRegion 一体化流程、f:source 分片父面、
流形保障、缝边纯分割传播、面颜色调试输出、测试清单与构建命令）。
