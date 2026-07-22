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
| 区域标记 | flood-fill 找连通区域，标记新区域 |
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
│   └── cmesh.h              # VCGlib 网格类型定义
├── JasMeshMarkAndSplit.h    # 主头文件
├── JasMeshMarkAndSplit.cpp  # 主实现
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
        
        // 从端点延长切割
        for each polyline in polylines:
            if polyline.start 不在 mark 不同的边上:
                从 polyline.start 延长切割
            if polyline.end 不在 mark 不同的边上:
                从 polyline.end 延长切割
        
        // 拣选子区域并标记新 mark
        subRegions = extractSubRegions(curFaces)
        每个子区域标记新 mark（新1, 新2, 新3...）

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
};
```

---

## 编译环境

### 依赖

- **VCGlib**：已包含在 `vcglib/` 目录
- **Eigen**：已包含在 `vcglib/eigenlib/` 目录
- **CMake**：版本 3.15+
- **C++ 编译器**：支持 C++17（MSVC 2022、GCC 9+、Clang 10+）

### Windows (Visual Studio 2022)

```bash
# 创建构建目录
mkdir build
cd build

# 生成 Visual Studio 项目
cmake -G "Visual Studio 17 2022" -A x64 ..

# 编译
cmake --build . --config Release

# 运行测试
./Release/test_mesh_cut.exe
```

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
testSplitMeshByMarkAndEdge passed
testSplitMeshByMarkAndEdgeSameMark passed
testIntegration passed

All 21 tests passed!
```

---

## 使用示例

```cpp
#include "JasMeshMarkAndSplit.h"

// 创建网格
CMeshOD mesh;
// ... 加载或创建网格 ...

// 设置 mark（每个三角形的平面 ID）
mesh.face[0].IMark() = 1;
mesh.face[1].IMark() = 1;
mesh.face[2].IMark() = 2;

// 创建分割器
JasMeshMarkAndSplit splitter;
splitter.SetMainMesh(&mesh);

// 执行分割
std::vector<JasMeshMarkAndSplit::splitReg> regions;
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
JasMeshMarkAndSplit splitter;
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
| `iter_N_sub_region_J.off` | OFF | 切割后的子区域 | Phase 2 每次迭代，步骤 2.5 后 |
| `final_polygons.obj` | OBJ | 各区域的边界多边形 | Phase 3 结束后 |
| `colored_mesh.obj` | OBJ | 带区域颜色的完整网格 | Phase 3 结束后 |

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

**OBJ 格式 - 带颜色网格**（顶点颜色）：
```
v x y z r g b
v x y z r g b
...
f v1 v2 v3
```

颜色通过 `std::map<int, vcg::Color4b>` 管理，key 为区域索引，每个区域分配随机颜色。

---

## 已知限制

1. **cutTriangleByPlane 未实现**：切割平面的三角形切割逻辑是存根，边缘切割功能尚未完全实现
2. **单边界环**：只存储第一个边界环，多孔区域会丢失信息
3. **流形假设**：边界遍历假设网格是流形的

---

## 未来改进

- [ ] 实现 `cutTriangleByPlane` 的实际切割逻辑
- [ ] 支持多孔区域的边界提取
- [ ] 添加非流形网格的处理
- [ ] 优化性能（O(n²) 前插入 → O(n)）
- [ ] 添加更多边界测试用例
- [ ] 使用测试框架替代 raw assert

---

## 许可证

本项目使用 VCGlib 的 GPL 许可证。

---

## 作者

Claude (Anthropic) - 2026-07-21
