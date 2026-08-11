# MeshCutByMark 设计文档

## 1. 问题描述

### 1.1 核心需求

网格中的三角形按平面标记（mark）分组，需要提取每组的简单多边形边界。

### 1.2 遇到的问题

- 网格存在**非流形边**（被 3 个或更多三角形共享的边）
- 网格存在**孔洞**（边界不闭合）
- 直接从三角形边界提取的多边形存在异常（自交、不闭合等）

### 1.3 图示说明

```
假设 curFaces 是一个 mark=1 的区域，其中有 3 条切割边（粗线）：

    +-------------------+
    |                   |
    |   A ---- B        |
    |   |      |        |
    |   |      |        |
    |   C ---- D        |
    |                   |
    +-------------------+

这 3 条切割边连接成一条折线：A → B → D → C

检查端点：
- A 在 mark 不同的边上 → 不需要延长
- C 不在 mark 不同的边上 → 需要延长

从 C 向下延长切割：

    +-------------------+
    |                   |
    |   A ---- B        |
    |   |      |        |
    |   |      |        |
    |   C ---- D        |
    |   |                |
    |   |                |
    +-------------------+

切割后，区域被分离成两个简单多边形
```

### 1.4 解决思路

1. 按 mark 收集连通区域的三角形（curFaces）
2. 找到区域内的切割边（mark 不同的边、非流形边、边界边）
3. **将切割边连接成连续折线**（按端点首尾相连）
4. 对每条折线，检查首尾端点：
   - 如果端点在 **mark 不同的边上** → 已在边界，不需要延长
   - 如果端点**不在** mark 不同的边上 → 从该端点**延长切割**
5. 延长切割：构造切割平面（垂直于三角形平面，包含折线方向），用平面切割 curFaces 中的所有三角形
6. 同时更新邻域三角形（共享边有交点就加点）
7. 切割后，区域被彻底分离成简单多边形
8. 从分离后的区域提取边界边

**为什么需要折线连接和端点延长？**
- 零散的切割边不能完全分离区域
- 连接成折线后，从"自由端点"（不在 mark 不同边上的端点）延长，像"刀"一样切穿区域
- 这样才能确保区域被彻底分离成简单多边形

---

## 2. 术语定义

| 术语 | 说明 |
|------|------|
| **mark** | 三角形的平面 ID，表示该三角形属于哪个平面 |
| **curFaces** | 从一个面出发，通过 flood-fill 找到的同 mark 连通区域 |
| **切割边** | 需要切开的边，包括三种类型（见 3.2） |
| **简单多边形** | 不自交、闭合的多边形边界 |

---

## 3. 数据结构

### 3.1 切割边类型

```cpp
enum CutEdgeType {
    CUT_EDGE_NONE,         // 普通边（2 个面共享且 mark 相同，非切割边）
    CUT_EDGE_MARK_DIFF,    // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD, // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY      // 只被 1 个三角形使用（孔洞边缘）
};
```

### 3.2 切割边

```cpp
struct CutEdge {
    int v0, v1;       // 端点顶点索引 (v0 < v1)
    int faceIdx;      // 所属面索引
    int edgeIdx;      // 边在面中的索引 (0, 1, 2)
    CutEdgeType type; // 切割边类型
};
```

### 3.3 新区域标记 newMark

实现上**不修改 `CFaceOD`**，而是在 `RegionMarker`（`tool/region_marker.h`）内用外部 vector 并行存储：

```cpp
// RegionMarker 内部，与 mesh->face 平行
std::vector<int> m_newMark;   // 每个 face 的简单多边形 ID，0 表示未处理
```

**初始化**：`initNewMark(mesh)` 将 `m_newMark` 全部置 0。

**处理流程**：
- 读取：`getNewMark(faceIdx)`；写入：`setNewMark(faceIdx, value)`；
- 每次 `extractSubRegions` 后，`markSubRegions(subRegions, mesh, newMarkCounter)` 把每个子区域的 face 标记为自增的新 mark；
- 最终所有 face 的 `newMark > 0`，表示已分配到某个简单多边形。

### 3.4 分割区域

```cpp
// 实际输出结构（JasMeshMarkAndCutSplit::splitReg）
struct splitReg {
    int mark;                    // 原始归属平面标记
    int newMark;                 // 新标记（切割后的简单多边形 ID）
    std::vector<int> inTris;     // 包含的三角形索引
    vcg::Point3d normal;         // 法向量（取组内第一个 face 的 N()，与原始 mesh 一致）
    std::vector<int> boundlines; // 主边界环的顶点索引序列（只存第一个环，孔洞丢失——已知限制）
};
```

### 3.5 JasMeshMarkAndCutSplit 类

```cpp
class JasMeshMarkAndCutSplit
{
public:
    struct splitReg
    {
        int mark;                     // 原始归属平面标记
        int newMark;                  // 新标记（切割后的简单多边形 ID）
        std::vector<int> inTris;      // 包含的三角形索引
        vcg::Point3d normal;          // 法向量（与原始 mesh face 一致）
        std::vector<int> boundlines;  // 主边界环顶点索引序列（只存第一个环）
    };

    JasMeshMarkAndCutSplit();
    ~JasMeshMarkAndCutSplit();

    void SetMainMesh(CMeshOD* pMesh) { m_pMesh = pMesh; }
    void BuildEdgeInfo() { m_edgeInfoManager.buildEdgeInfo(m_pMesh); }
    void SetDebug(bool enable);
    void SetDebugOutputDir(const std::string& dir);

    void SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs);
    std::vector<MeshCutByMark::CutEdge> findCutEdges(const std::vector<int>& curFaces);
    std::vector<std::vector<int>> extractBoundaryEdges(const std::vector<int>& regionFaces);

private:
    CMeshOD* m_pMesh = nullptr;
    MeshCutByMark::EdgeInfoManager m_edgeInfoManager;
    MeshCutByMark::PolylineManager m_polylineManager;
    MeshCutByMark::CutPlaneManager m_cutPlaneManager;
    MeshCutByMark::RegionMarker m_regionMarker;
    int m_newMarkCounter = 0;                       // 新标记计数器
    std::vector<vcg::Point3i> m_edgeMarks;          // 是否为分割边（当前未使用）
    bool m_debug = false;
    std::string m_debugOutputDir = "debug_output/";
    int m_debugIterCounter = 0;
};
```

---

## 4. 算法流程

### 4.1 总体流程

```
Phase 1: 构建边信息
    遍历所有三角形，建立边 → 面的映射
    分类每条边：mark不同 / 非流形 / 边界 / 普通

Phase 2: 延长线切割并标记新区域
    for each 未标记新 mark 的面 i:
        curFaces = regionMarker.floodFill(i, targetMark, mesh, edgeInfo)
        cutEdges = findCutEdges(curFaces)
        
        // 将切割边连接成连续折线
        polylines = polylineManager.connectEdgesToPolylines(cutEdges, mesh)
        
        // 从端点延长切割（仅 NON_MANIFOLD 折线的悬空端点）
        for each polyline in polylines:
            if polyline.type != NON_MANIFOLD: continue
            if polyline.start 不在 mark 不同的边上:
                从 polyline.start 延长切割
            if polyline.end 不在 mark 不同的边上:
                从 polyline.end 延长切割
        
        // 拣选子区域并标记新 mark
        subRegions = regionMarker.extractSubRegions(curFaces, mesh)
        regionMarker.markSubRegions(subRegions, mesh, newMarkCounter)

Phase 3: 根据新标记提取多边形
    按新标记分组所有三角形
    对每组提取边界边 → 形成简单闭合多边形
```

### 4.2 Phase 1: 构建边信息

**目标**：建立边的索引，分类每条边的类型。

**步骤**：

1. 遍历所有三角形，对每条边：
   - 记录边的两个端点顶点索引 `(v0, v1)`，其中 `v0 < v1`
   - 记录该边所属的面

2. 对每条边，统计共享该边的面数：
   - 面数 = 1 → 边界边（`CUT_EDGE_BOUNDARY`）
   - 面数 = 2 → 普通边或 mark 不同边
   - 面数 ≥ 3 → 非流形边（`CUT_EDGE_NON_MANIFOLD`）

3. 对面数 = 2 的边，检查两侧面的 mark：
   - mark 不同 → `CUT_EDGE_MARK_DIFF`
   - mark 相同 → 普通边（非切割边）

**数据结构**：

```cpp
// 边的哈希函数（用于快速查找）
struct EdgeHash {
    size_t operator()(const std::pair<int,int>& e) const {
        int lo = std::min(e.first, e.second);
        int hi = std::max(e.first, e.second);
        return std::hash<int>()(lo) ^ (std::hash<int>()(hi) << 1);
    }
};

// 边的相等比较（规范化后比较，保证 {a,b} == {b,a}）
struct EdgeEqual {
    bool operator()(const std::pair<int,int>& a, const std::pair<int,int>& b) const {
        return std::minmax(a.first, a.second) == std::minmax(b.first, b.second);
    }
};

// EdgeInfoManager（tool/edge_info.h）内部实际存储：
//   m_edgeToFaces: unordered_map<pair<int,int>, vector<int>, EdgeHash, EdgeEqual>
//   m_edgeTypes:   unordered_map<pair<int,int>, CutEdgeType, EdgeHash, EdgeEqual>
// 接口：buildEdgeInfo(mesh) / getEdgeType(v0,v1) / getAdjacentFaces(v0,v1) / isCutEdge(v0,v1)
```

### 4.3 Phase 2: 延长线切割并标记新区域

**目标**：对每个连通区域，将切割边连接成折线，从端点延长切割，将切割后的子区域标记新的 mark。

**步骤**：

1. 维护全局的新标记计数器 `newMarkCounter = 1`

2. 对每个未标记新 mark 的面 `i`：
   ```
   if regionMarker.getNewMark(i) > 0:
       continue  // 已处理

   // 2.1 flood-fill 找连通区域
   targetMark = mesh.face[i].IMark()
   curFaces = regionMarker.floodFill(i, targetMark, mesh, edgeInfo)
   
   // 2.2 找 curFaces 的切割边
   cutEdges = findCutEdges(curFaces)
   
   // 2.3 将切割边连接成连续折线
   polylines = polylineManager.connectEdgesToPolylines(cutEdges, mesh)
   
   // 2.4 从端点延长切割（仅 NON_MANIFOLD 折线）
   for each polyline in polylines:
       if polyline.type != CUT_EDGE_NON_MANIFOLD: continue
       // 检查首端点
       if polyline.start 不在 mark 不同的边上:
           从 polyline.start 延长切割
      
       // 检查尾端点
       if polyline.end 不在 mark 不同的边上:
           从 polyline.end 延长切割
   
   // 2.5 通过拣选得到切割后的子区域
   subRegions = regionMarker.extractSubRegions(curFaces, mesh)
   
   // 2.6 每个子区域标记新 mark
   regionMarker.markSubRegions(subRegions, mesh, newMarkCounter)
   ```

**连接切割边为折线的逻辑**：

`tool/polyline.h` 中 `Polyline` 的实际字段：

```cpp
struct Polyline {
    std::vector<int> vertexIndices;   // 折线顶点序列
    int startFaceIdx;                 // 首端点所在的面
    int startEdgeIdx;                 // 首端点所在的边索引
    int endFaceIdx;                   // 尾端点所在的面
    int endEdgeIdx;                   // 尾端点所在的边索引
    bool isClosed = false;            // 是否闭合环
    CutEdgeType type = CUT_EDGE_NONE; // 折线主导边类型
};
```

`connectEdgesToPolylines(cutEdges, mesh)` 的实际行为：

1. 按类型把切割边分成三组：`CUT_EDGE_MARK_DIFF`、`CUT_EDGE_BOUNDARY`、`CUT_EDGE_NON_MANIFOLD`；
2. 各组内部 `connectByType`：从一条未用边出发，沿端点向两端扩展成折线；`NON_MANIFOLD` 组用 canonical key（`EdgeHash`/`EdgeEqual`）记录已用边并跳过反向边；
3. `MARK_DIFF` 与 `BOUNDARY` 的折线再用 `tryMergePolylines` 按端点（正向/反向）合并；
4. 若折线首尾顶点相同，`isClosed = true`。

`findCutEdges` 见下方；`PolylineManager` 接口为
`std::vector<Polyline> connectEdgesToPolylines(const std::vector<CutEdge>&, CMeshOD*)`。

**从端点延长切割的逻辑**：

仅对 `type == CUT_EDGE_NON_MANIFOLD` 的折线处理悬空端点；端点位于 mark 不同边上（`isOnMarkDiffEdge`）则不延长。实际代码（`JasMeshMarkAndCutSplit.cpp` Phase 2.4）：

```cpp
for (const auto& polyline : polylines) {
    if (polyline.type != CUT_EDGE_NON_MANIFOLD) continue;
    // 首端点
    if (!m_cutPlaneManager.isOnMarkDiffEdge(polyline.startFaceIdx, polyline.startEdgeIdx, m_pMesh)) {
        vcg::Plane3d plane = m_cutPlaneManager.makeCutPlane(polyline, true, m_pMesh);
        for (int faceIdx : curFaces)
            if (!m_pMesh->face[faceIdx].IsD())
                m_cutPlaneManager.cutTriangleByPlane(faceIdx, plane, m_pMesh);
    }
    // 尾端点同理（makeCutPlane(polyline, false, m_pMesh)）
}
```

`makeCutPlane(polyline, isStart, mesh)`：方向 `D`（start 取 `v[0]-v[1]`、end 取 `v[last]-v[last-1]`，归一化）、面法向 `N`、切割平面法向 `C = D × N`，平面过端点。

> **现状（以代码为准）**：`cutTriangleByPlane` 是**存根**——只计算三个顶点到平面的有符号距离 `d0/d1/d2` 后 `(void)` 丢弃，不真正切分三角形。因此当前版本"延长切割"实际切不开区域（"切不断"问题）；邻域三角形加点（`updateAdjacentTriangles`）也未实现。

**拣选子区域的逻辑**：

`tool/region_marker.h` 的 `RegionMarker::extractSubRegions(curFaces, mesh)`：基于内部 `m_newMark` 与 `isCutEdge`（FF 邻接为空或自指即视为切割边），对 `curFaces` 做 flood-fill，返回若干子区域（每组为全局 face 索引）。

```cpp
// 实际签名
std::vector<std::vector<int>> extractSubRegions(
    const std::vector<int>& curFaces, CMeshOD* mesh);
```

> 说明：当前 `isCutEdge` 只把"无 FF 邻接"的边视为切割边；切割路径上的边需在 phase-2 改造中把 FF 置为自指（或由外部 cutter 产生真实切割边）后才会成为子区域屏障。

**floodFill 算法**：

`RegionMarker::floodFill(startFaceIdx, targetMark, mesh, edgeInfo)`：从起点 BFS；跳过 `isCutEdge`（无 FF 邻接）的边；邻接面必须与 `targetMark` 相同且未访问。

```cpp
// 实际签名
std::vector<int> floodFill(int startFaceIdx, int targetMark,
                           CMeshOD* mesh, const EdgeInfoManager& edgeInfo);
```

**findCutEdges 算法**：

类方法 `JasMeshMarkAndCutSplit::findCutEdges(curFaces)`：遍历 `curFaces` 的每条边，查 `m_edgeInfoManager.getEdgeType(v0, v1)`，非 `CUT_EDGE_NONE` 即记录 `{v0, v1, faceIdx, edgeIdx, type}`。

```cpp
std::vector<MeshCutByMark::CutEdge> findCutEdges(const std::vector<int>& curFaces) {
    std::vector<MeshCutByMark::CutEdge> cutEdges;
    for (int faceIdx : curFaces) {
        for (int j = 0; j < 3; j++) {
            int v0 = m_pMesh->face[faceIdx].V(j)->Index();
            int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();
            if (v0 > v1) std::swap(v0, v1);
            MeshCutByMark::CutEdgeType type = m_edgeInfoManager.getEdgeType(v0, v1);
            if (type != MeshCutByMark::CUT_EDGE_NONE) {
                cutEdges.push_back({v0, v1, faceIdx, j, type});
            }
        }
    }
    return cutEdges;
}
```

### 4.4 Phase 2.4: 延长线切割（当前为存根）

**现状（以代码为准）**：

- `CutPlaneManager::makeCutPlane(polyline, isStart, mesh)` 已实现：取端点世界坐标 `P`、外延方向 `D`（start 用 `normalize(v[0]-v[1])`，end 用 `normalize(v[last]-v[last-1])`）、面法向 `N`，切割平面法向 `C = D × N`，`plane.Init(P, C)`。
- `CutPlaneManager::cutTriangleByPlane(faceIdx, plane, mesh)` 是 **TODO 存根**：只计算 `d0/d1/d2` 有符号距离后 `(void)` 丢弃，**不切分三角形**。
- 邻域三角形加点（`updateAdjacentTriangles`）：设计中有，代码**未实现**。

> 因此当前版本无法真正把区域切开——非流形边区域无法分离（"切不断"问题）。
> 后续阶段（见 `docs/superpowers/plans/2026-07-22-phase2-local-mesh-cut.md`）将 Phase 2.4 改为
> 「提取局部 mesh → 调用外部稳定 cutter（cgalLocalMeshCut 的 `AddCutLines`）→ merge 回主网格」管线，
> 替换该存根路径。

### 4.5 Phase 3: 根据新标记提取多边形

**目标**：根据新标记分组三角形，提取每组的边界边序列，形成简单闭合多边形，输出最终结果。

**步骤**：

1. 按新标记分组所有三角形：
   ```cpp
   std::map<int, std::vector<int>> markToFaces;
   for (int i = 0; i < m_pMesh->face.size(); i++) {
       if (!m_pMesh->face[i].IsD()) {
           markToFaces[m_regionMarker.getNewMark(i)].push_back(i);
       }
   }
   ```

2. 对每个新标记的三角形组，提取边界边：
   ```cpp
   for (const auto& [newMark, faces] : markToFaces) {
       // 提取边界边
       std::vector<std::vector<int>> boundaries = extractBoundaryEdges(faces);
       
       // 构造 splitReg
       splitReg reg;
       reg.mark = m_pMesh->face[faces[0]].IMark();  // 原始 mark
       reg.newMark = newMark;
       reg.inTris = faces;
       reg.normal = m_pMesh->face[faces[0]].N();
       reg.boundlines = boundaries.empty() ? std::vector<int>() : boundaries[0];  // 主边界（只存第一个环）
       
       retRegs.push_back(reg);
   }
   ```

3. 处理多个边界的情况（**现状**）：
   - `extractBoundaryEdges` 会返回所有边界环（含孔洞环），但 `splitReg::boundlines` **只保存第一个环**，带孔区域会丢失洞信息（已知限制）；
   - 后续阶段用外部库（cgalLocalMeshCut 的 `SubRegionBoundary`）替换该提取，返回外圈 + 内部洞。

**提取边界边算法**：

```cpp
// 返回边界边的顶点索引序列，方向与原始 mesh face 一致
std::vector<std::vector<int>> extractBoundaryEdges(const std::vector<int>& regionFaces) {
    std::vector<std::vector<int>> boundaries;
    
    // 找到所有边界边（只被一个面使用的边）
    std::vector<std::pair<int,int>> boundaryEdges;
    for (int faceIdx : regionFaces) {
        for (int j = 0; j < 3; j++) {
            if (isBoundaryEdge(faceIdx, j)) {
                // 保持原始 mesh face 的边方向
                int v0 = m_pMesh->face[faceIdx].V(j)->Index();
                int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();
                boundaryEdges.push_back({v0, v1});
            }
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

---

## 5. VCGlib 依赖

### 5.1 使用的 VCGlib 函数

| 函数 | 文件 | 用途 |
|------|------|------|
| `SignedDistancePlanePoint` | `vcg/space/plane3.h` | 计算点到平面的有符号距离 |
| `IntersectionPlaneSegment` | `vcg/space/intersection3.h` | 平面与线段求交 |
| `IntersectionPlaneTriangle` | `vcg/space/intersection3.h` | 平面与三角形求交 |
| `Plane3::Init` | `vcg/space/plane3.h` | 从点+法向量构造平面 |

### 5.2 使用的 VCGlib 数据结构

| 结构 | 用途 |
|------|------|
| `CMeshOD` | 网格数据结构 |
| `FFAdjOcf` | 面-面邻接关系（用于遍历邻接面） |
| `BitFlags` | 面的边选中标记 |

---

## 6. 边界情况处理

### 6.1 退化三角形

- 面积为零的三角形：跳过，不参与处理
- 三点共线的三角形：视为退化，跳过

### 6.2 孤立面

- 没有邻接面的面：直接作为独立区域处理

### 6.3 共面但 mark 不同

- 两个相邻三角形共面但 mark 不同：视为切割边

### 6.4 非流形顶点

- 多个面共享一个顶点但拓扑不连续：通过 flood-fill 自然处理

---

## 7. 输出格式

### 7.1 splitReg

```cpp
struct splitReg {
    int mark;                    // 原始归属平面标记
    int newMark;                 // 切割后的简单多边形 ID
    std::vector<int> inTris;     // 包含的三角形索引
    vcg::Point3d normal;         // 法向量（组内第一个 face 的 N()，与原始 mesh 一致）
    std::vector<int> boundlines; // 主边界环顶点索引序列（只存第一个环）
};
```

### 7.2 使用示例

```cpp
CMeshOD mesh;
// ... 加载 mesh ...

JasMeshMarkAndCutSplit splitter;
splitter.SetMainMesh(&mesh);

std::vector<JasMeshMarkAndCutSplit::splitReg> regions;
splitter.SplitMeshByMarkAndEdge(regions);

// 使用结果
for (const auto& region : regions) {
    std::cout << "Original Mark: " << region.mark << std::endl;
    std::cout << "New Mark: " << region.newMark << std::endl;
    std::cout << "Triangles: " << region.inTris.size() << std::endl;
    std::cout << "Boundary edges: " << region.boundlines.size() << std::endl;
    
    // boundlines 存储的是边界边的顶点索引序列
    // 例如：[v0, v1, v2, v3, ..., v0] 表示一个闭合多边形
    // 方向与原始 mesh face 一致
}

// newMark 由 RegionMarker 内部维护（m_newMark vector，无公开访问），
// 外部按 retRegs 使用结果即可。
```

---

## 8. 性能考虑

### 8.1 时间复杂度

- Phase 1（构建边信息）：O(F)，F 为三角形数
- Phase 2（延长线切割并标记新区域）：O(F * C)，C 为切割边数量
  - 每个 curFaces 的 flood-fill：O(F)
  - 连接切割边为折线：O(C)，C 为切割边数量
  - 从端点延长切割：O(F)（当前 `cutTriangleByPlane` 为存根，只遍历算距离、不产生新面）
  - 拣选子区域：O(F)
- Phase 3（根据新标记提取多边形）：O(F)

总体：O(F * C)，C 通常很小

### 8.2 空间复杂度

- 边信息存储：O(E)，E 为边数
- 折线存储：O(C)
- flood-fill 队列：O(F)
- 新区域标记：O(F)（`RegionMarker::m_newMark` 与 face 平行的 int vector）
- 输出结果：O(F)

---

## 9. 调试输出

### 9.1 设计目标

算法执行过程中产生大量中间数据（连通区域、折线、子区域、边界多边形），需要可视化这些中间结果来验证算法正确性。

### 9.2 接口设计

```cpp
class JasMeshMarkAndCutSplit {
public:
    void SetDebug(bool enable);                      // 开关调试输出
    void SetDebugOutputDir(const std::string& dir);  // 设置输出目录
private:
    bool m_debug = false;
    std::string m_debugOutputDir = "debug_output/";
    int m_debugIterCounter = 0;
};
```

### 9.3 输出文件

| 文件 | 格式 | 内容 | 输出位置 |
|------|------|------|----------|
| `iter_N_cur_faces.off` | OFF | flood-fill 连通区域 | Phase 2 步骤 2.1 后 |
| `iter_N_polylines.obj` | OBJ (l) | 折线 | Phase 2 步骤 2.3 后 |
| `iter_N_sub_region_J.off` | OFF | 子区域 | Phase 2 步骤 2.5 后 |
| `final_polygons.obj` | OBJ (f) | 边界多边形 | Phase 3 |
| `colored_mesh.obj` | OBJ | 带颜色网格 | Phase 3 |

### 9.4 颜色管理

使用 `std::map<int, vcg::Color4b>` 管理区域颜色映射：
- key：retRegs 数组索引
- value：随机生成的 RGBA 颜色

面颜色通过 VCGlib 的 `EnableColor()` 启用，输出时转换为顶点颜色（取相邻面颜色平均值）。

### 9.5 辅助方法

```cpp
void debugEnsureDir();                                              // 创建输出目录
void debugWritePolylines(int iterIdx, const vector<Polyline>&);    // 输出折线 OBJ
void debugWriteFacesOFF(int iterIdx, const char* suffix, const vector<int>&);  // 输出三角形 OFF
void debugWriteSubRegionsOFF(int iterIdx, const vector<vector<int>>&);         // 输出子区域 OFF
void debugWritePolygonsOBJ(const map<int, vector<int>>&);          // 输出多边形 OBJ
void debugSaveColoredMesh(const vector<splitReg>&);                // 输出带颜色网格
```

---

## 10. 测试计划

### 10.1 单元测试（现状：`tests/test_mesh_cut.cpp`，raw assert，共 21 个，`main()` 按序调用）

- 边信息：`testEdgeHash`、`testBuildEdgeInfo`、`testFindCutEdges`
- 折线：`testConnectEdgesToPolylines` / `Multiple` / `Empty`
- 切割平面：`testMakeCutPlane`、`testMakeCutPlaneLongPolyline`、`testIsOnMarkDiffEdge`、`testSignedDistanceAndIntersection`
- 区域：`testFloodFill`、`testFloodFillMarkDiff`、`testFloodFillBoundary`、`testExtractSubRegions`、`testMarkSubRegions`、`testInitNewMark`
- 边界：`testExtractBoundaryEdges`、`testExtractBoundaryEdgesTwoTriangles`

### 10.2 集成测试

- `testSplitMeshByMarkAndEdge`、`testSplitMeshByMarkAndEdgeSameMark`、`testIntegration`

---

## 11. 已确认事项

1. **mark 的来源**：mark 由外部定义，本模块只读取，不负责计算。
2. **多边形方向**：输出多边形的边以 mesh 中提取的边的方向为准。`std::vector<int>` 存储的是 curFaces 切割后得到的边的顶点索引。最终多边形方向与原始输入 mesh face 的方向一致。
3. **法向量计算**：`splitReg::normal` 与原始 mesh face 的法向量一致，不做额外计算。
4. **boundlines 格式**：`splitReg::boundlines` 存储的是边的顶点索引序列，与原始 mesh 的边方向一致；**只保存第一个边界环**，多孔区域会丢失洞信息（已知限制，后续用 cgalLocalMeshCut 的 `SubRegionBoundary` 修复）。
