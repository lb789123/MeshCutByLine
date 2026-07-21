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
    CUT_EDGE_MARK_DIFF,    // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD, // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY      // 只被 1 个三角形使用（孔洞边缘）
};
```

### 3.2 切割边

```cpp
struct CutEdge {
    int faceIdx;      // 所属面索引
    int edgeIdx;      // 边在面中的索引 (0, 1, 2)
    CutEdgeType type; // 切割边类型
};
```

### 3.3 Face 新增属性

每个 face 需要新增一个 `newMark` 属性，用于记录切割后所属的简单多边形 ID：

```cpp
// 在 CFaceO 中添加
int newMark;  // 切割后的简单多边形 ID，0 表示未处理
```

**初始化**：所有 face 的 `newMark = 0`

**处理流程**：
- 每次延长线切割后，子区域中的 face 被标记为 `newMark = ++newMarkCounter`
- 最终所有 face 的 `newMark > 0`，表示已分配到某个简单多边形

### 3.4 分割区域

```cpp
struct SplitRegion {
    int mark;                                    // 原始平面 ID
    int newMark;                                 // 新标记（简单多边形 ID）
    std::vector<int> faceIndices;                // 包含的三角形索引
    std::vector<std::vector<int>> boundaries;    // 边界边的顶点索引序列（方向与原始 mesh face 一致）
};
```

### 3.4 JasMeshMarkAndSplit 类

```cpp
class JasMeshMarkAndSplit
{
public:
    struct splitReg
    {
        int mark;                             // 原始归属平面标记
        int newMark;                          // 新标记（切割后的简单多边形 ID）
        int polyInd;                          // 平面索引
        std::vector<int> inTris;              // 包含的三角形索引
        vcg::Point3d normal;                  // 法向量（与原始 mesh face 一致）
        std::vector<int> boundlines;          // 边界边的顶点索引序列（与原始 mesh 边方向一致）
    };

    JasMeshMarkAndSplit();
    ~JasMeshMarkAndSplit();

    void SetMainMesh(CMeshO* pMesh) { m_pMesh = pMesh; }

    // 返回分割后的多个三维多边形
    void SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs);

private:
    CMeshO* m_pMesh;
    int m_newMarkCounter; // 新标记计数器
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
// 边信息
struct EdgeInfo {
    int v0, v1;                    // 端点顶点索引 (v0 < v1)
    std::vector<int> adjFaces;     // 邻接面索引
    CutEdgeType type;              // 边类型
};

// 边的哈希函数（用于快速查找）
struct EdgeHash {
    size_t operator()(const std::pair<int,int>& e) const {
        return std::hash<int>()(e.first) ^ std::hash<int>()(e.second);
    }
};

// 边 → 面的映射
std::unordered_map<std::pair<int,int>, EdgeInfo, EdgeHash> edgeMap;
```

### 4.3 Phase 2: 延长线切割并标记新区域

**目标**：对每个连通区域，将切割边连接成折线，从端点延长切割，将切割后的子区域标记新的 mark。

**步骤**：

1. 维护全局的新标记计数器 `newMarkCounter = 1`

2. 对每个未标记新 mark 的面 `i`：
   ```
   if face[i].newMark > 0:
       continue  // 已处理

   // 2.1 flood-fill 找连通区域
   curFaces = floodFill(i)
   
   // 2.2 找 curFaces 的切割边
   cutEdges = findCutEdges(curFaces)
   
   // 2.3 将切割边连接成连续折线
   polylines = connectEdgesToPolylines(cutEdges)
   
   // 2.4 从端点延长切割
   for each polyline in polylines:
       // 检查首端点
       if polyline.start 不在 mark 不同的边上:
           从 polyline.start 延长切割
       
       // 检查尾端点
       if polyline.end 不在 mark 不同的边上:
           从 polyline.end 延长切割
   
   // 2.5 通过拣选得到切割后的子区域
   subRegions = extractSubRegions(curFaces)
   
   // 2.6 每个子区域标记新 mark
   for each region in subRegions:
       for each face in region:
           face.newMark = newMarkCounter
       newMarkCounter++
   ```

**连接切割边为折线的逻辑**：

将零散的切割边按端点连接成连续的折线：

```cpp
struct Polyline {
    std::vector<int> vertexIndices;  // 折线的顶点序列
    int startFaceIdx;                // 首端点所在的面
    int startEdgeIdx;                // 首端点所在的边索引
    int endFaceIdx;                  // 尾端点所在的面
    int endEdgeIdx;                  // 尾端点所在的边索引
};

std::vector<Polyline> connectEdgesToPolylines(const std::vector<CutEdge>& cutEdges) {
    std::vector<Polyline> polylines;
    
    // 构建端点 → 边的映射
    std::unordered_map<int, std::vector<int>> vertexToEdges;
    for (int i = 0; i < cutEdges.size(); i++) {
        int v0 = getEdgeVertex0(cutEdges[i]);
        int v1 = getEdgeVertex1(cutEdges[i]);
        vertexToEdges[v0].push_back(i);
        vertexToEdges[v1].push_back(i);
    }
    
    // 连接边形成折线
    std::vector<bool> used(cutEdges.size(), false);
    for (int i = 0; i < cutEdges.size(); i++) {
        if (used[i]) continue;
        
        Polyline polyline;
        used[i] = true;
        
        // 从这条边开始，向两端扩展
        int startV = getEdgeVertex0(cutEdges[i]);
        int endV = getEdgeVertex1(cutEdges[i]);
        polyline.vertexIndices.push_back(startV);
        polyline.vertexIndices.push_back(endV);
        
        // 向 startV 方向扩展
        while (true) {
            bool found = false;
            for (int edgeIdx : vertexToEdges[startV]) {
                if (!used[edgeIdx]) {
                    used[edgeIdx] = true;
                    int otherV = (getEdgeVertex0(cutEdges[edgeIdx]) == startV) ? 
                                 getEdgeVertex1(cutEdges[edgeIdx]) : 
                                 getEdgeVertex0(cutEdges[edgeIdx]);
                    polyline.vertexIndices.insert(polyline.vertexIndices.begin(), otherV);
                    startV = otherV;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }
        
        // 向 endV 方向扩展
        while (true) {
            bool found = false;
            for (int edgeIdx : vertexToEdges[endV]) {
                if (!used[edgeIdx]) {
                    used[edgeIdx] = true;
                    int otherV = (getEdgeVertex0(cutEdges[edgeIdx]) == endV) ? 
                                 getEdgeVertex1(cutEdges[edgeIdx]) : 
                                 getEdgeVertex0(cutEdges[edgeIdx]);
                    polyline.vertexIndices.push_back(otherV);
                    endV = otherV;
                    found = true;
                    break;
                }
            }
            if (!found) break;
        }
        
        // 记录首尾端点信息
        polyline.startFaceIdx = getFaceAtVertex(startV);
        polyline.startEdgeIdx = getEdgeAtVertex(startV);
        polyline.endFaceIdx = getFaceAtVertex(endV);
        polyline.endEdgeIdx = getEdgeAtVertex(endV);
        
        polylines.push_back(polyline);
    }
    
    return polylines;
}
```

**从端点延长切割的逻辑**：

```cpp
void extendCutFromEndpoint(const Polyline& polyline, bool isStart) {
    // 获取端点信息
    int faceIdx = isStart ? polyline.startFaceIdx : polyline.endFaceIdx;
    int edgeIdx = isStart ? polyline.startEdgeIdx : polyline.endEdgeIdx;
    int vertexIdx = isStart ? polyline.vertexIndices[0] : polyline.vertexIndices.back();
    
    // 检查端点是否在 mark 不同的边上
    if (isOnMarkDiffEdge(faceIdx, edgeIdx)) {
        return;  // 不需要延长
    }
    
    // 获取折线方向
    vcg::Point3d dir;
    if (isStart) {
        vcg::Point3d v0 = m_pMesh->vert[polyline.vertexIndices[0]].P();
        vcg::Point3d v1 = m_pMesh->vert[polyline.vertexIndices[1]].P();
        dir = v0 - v1;  // 从 v1 指向 v0（向外延伸）
    } else {
        vcg::Point3d v0 = m_pMesh->vert[polyline.vertexIndices[polyline.vertexIndices.size()-2]].P();
        vcg::Point3d v1 = m_pMesh->vert[polyline.vertexIndices.back()].P();
        dir = v1 - v0;  // 从 v0 指向 v1（向外延伸）
    }
    dir.Normalize();
    
    // 构造切割平面（过端点，方向为折线方向，垂直于三角形平面）
    vcg::Point3d N = m_pMesh->face[faceIdx].N();
    vcg::Point3d C = dir ^ N;  // 切割平面法向量
    C.Normalize();
    
    vcg::Plane3d plane;
    plane.Init(m_pMesh->vert[vertexIdx].P(), C);
    
    // 用这个平面切割 curFaces 中的所有三角形
    cutCurFacesByPlane(plane);
}
```

**拣选子区域的逻辑**：

切割后，curFaces 被分离成多个不连通的子区域。通过 flood-fill 从任意未标记的面开始，找到所有连通的面，形成一个子区域。

```cpp
std::vector<std::vector<int>> extractSubRegions(const std::vector<int>& curFaces) {
    std::vector<std::vector<int>> subRegions;
    
    for (int faceIdx : curFaces) {
        if (m_pMesh->face[faceIdx].newMark > 0)
            continue;  // 已标记
        
        // 从这个面开始 flood-fill
        std::vector<int> region;
        std::queue<int> queue;
        queue.push(faceIdx);
        m_pMesh->face[faceIdx].newMark = -1;  // 临时标记为正在处理
        
        while (!queue.empty()) {
            int curFace = queue.front();
            queue.pop();
            region.push_back(curFace);
            
            // 遍历三条边
            for (int j = 0; j < 3; j++) {
                // 跳过切割边（已被切开，不再是邻接关系）
                if (isCutEdge(curFace, j))
                    continue;
                
                int adjFace = getAdjacentFace(curFace, j);
                if (adjFace < 0 || m_pMesh->face[adjFace].newMark != 0)
                    continue;
                
                // 检查是否还在 curFaces 中
                if (!isInCurFaces(adjFace))
                    continue;
                
                m_pMesh->face[adjFace].newMark = -1;
                queue.push(adjFace);
            }
        }
        
        subRegions.push_back(region);
    }
    
    return subRegions;
}
```

**floodFill 算法**：

```cpp
std::vector<int> floodFill(int startFaceIdx) {
    std::vector<int> result;
    std::queue<int> queue;
    queue.push(startFaceIdx);
    visited[startFaceIdx] = true;
    
    int targetMark = m_pMesh->face[startFaceIdx].mark;
    
    while (!queue.empty()) {
        int faceIdx = queue.front();
        queue.pop();
        result.push_back(faceIdx);
        
        // 遍历三条边
        for (int j = 0; j < 3; j++) {
            // 跳过切割边
            if (isCutEdge(faceIdx, j))
                continue;
            
            // 获取邻接面
            int adjFaceIdx = getAdjacentFace(faceIdx, j);
            
            // 检查邻接面是否有效
            if (adjFaceIdx < 0 || visited[adjFaceIdx])
                continue;
            
            // 检查 mark 是否相同
            if (m_pMesh->face[adjFaceIdx].mark != targetMark)
                continue;
            
            visited[adjFaceIdx] = true;
            queue.push(adjFaceIdx);
        }
    }
    
    return result;
}
```

**findCutEdges 算法**：

```cpp
std::vector<CutEdge> findCutEdges(const std::vector<int>& curFaces) {
    std::vector<CutEdge> cutEdges;
    
    for (int faceIdx : curFaces) {
        for (int j = 0; j < 3; j++) {
            EdgeInfo& edge = getEdge(faceIdx, j);
            
            if (edge.type == CUT_EDGE_MARK_DIFF ||
                edge.type == CUT_EDGE_NON_MANIFOLD ||
                edge.type == CUT_EDGE_BOUNDARY) {
                cutEdges.push_back({faceIdx, j, edge.type});
            }
        }
    }
    
    return cutEdges;
}
```

### 4.4 Phase 3: 延长线切割

**目标**：将切割边延长为直线，用切割平面切割 curFaces 中的所有三角形，彻底分离区域。

**切割平面构造**：

对于每条切割边 `(v0, v1)`：
- **边方向**：`E = v1 - v0`
- **三角形法向量**：`N`（该三角形所在平面的法向量）
- **切割平面法向量**：`C = E × N`（叉积，垂直于边和法向量）
- **切割平面**：过边 `(v0, v1)`，法向量为 `C`

```cpp
// 构造切割平面
vcg::Plane3d makeCutPlane(int faceIdx, int edgeIdx) {
    // 获取边的两个端点
    vcg::Point3d v0 = m_pMesh->face[faceIdx].V(edgeIdx)->P();
    vcg::Point3d v1 = m_pMesh->face[faceIdx].V((edgeIdx+1)%3)->P();
    
    // 边方向
    vcg::Point3d E = v1 - v0;
    
    // 三角形法向量
    vcg::Point3d N = m_pMesh->face[faceIdx].N();
    
    // 切割平面法向量（垂直于边和法向量）
    vcg::Point3d C = E ^ N;  // 叉积
    C.Normalize();
    
    // 构造平面（过 v0，法向量为 C）
    vcg::Plane3d plane;
    plane.Init(v0, C);
    
    return plane;
}
```

**三角形切割**：

使用 VCGlib 的有符号距离法切割三角形：

```cpp
// 用切割平面切割三角形
void cutTriangleByPlane(int faceIdx, const vcg::Plane3d& plane) {
    // 计算三个顶点到平面的有符号距离
    double d0 = vcg::SignedDistancePlanePoint(plane, m_pMesh->face[faceIdx].V(0)->P());
    double d1 = vcg::SignedDistancePlanePoint(plane, m_pMesh->face[faceIdx].V(1)->P());
    double d2 = vcg::SignedDistancePlanePoint(plane, m_pMesh->face[faceIdx].V(2)->P());
    
    // 根据距离的正负号判断切割方式
    // ... 三角形切割逻辑 ...
}
```

**邻域处理**：

当切割边被延长切割时，邻域三角形（curFaces 外部）如果共享被切割的边，也需要在交点处加点：

```cpp
// 更新邻域三角形
void updateAdjacentTriangles(const vcg::Plane3d& plane, int cutEdgeFaceIdx, int cutEdgeIdx) {
    // 获取共享该边的邻接面
    int adjFaceIdx = getAdjacentFace(cutEdgeFaceIdx, cutEdgeIdx);
    
    if (adjFaceIdx >= 0 && !isInCurFaces(adjFaceIdx)) {
        // 邻接面不在 curFaces 中，需要在交点处加点
        // 计算边与平面的交点
        // 在交点处分裂邻接面
    }
}
```

**完整切割流程**：

```cpp
void cutRegionByExtendedLines(std::vector<int>& curFaces, const std::vector<CutEdge>& cutEdges) {
    for (const auto& cutEdge : cutEdges) {
        // 1. 构造切割平面
        vcg::Plane3d plane = makeCutPlane(cutEdge.faceIdx, cutEdge.edgeIdx);
        
        // 2. 切割 curFaces 中的所有三角形
        for (int faceIdx : curFaces) {
            if (!isDeleted(faceIdx)) {
                cutTriangleByPlane(faceIdx, plane);
            }
        }
        
        // 3. 更新邻域三角形（共享边有交点就加点）
        updateAdjacentTriangles(plane, cutEdge.faceIdx, cutEdge.edgeIdx);
    }
}
```

### 4.5 Phase 4: 根据新标记提取多边形

**目标**：根据新标记分组三角形，提取每组的边界边序列，形成简单闭合多边形，输出最终结果。

**步骤**：

1. 按新标记分组所有三角形：
   ```cpp
   std::map<int, std::vector<int>> markToFaces;
   for (int i = 0; i < m_pMesh->face.size(); i++) {
       if (!m_pMesh->face[i].IsD()) {
           markToFaces[m_pMesh->face[i].newMark].push_back(i);
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
       reg.mark = m_pMesh->face[faces[0]].mark;  // 原始 mark
       reg.newMark = newMark;
       reg.inTris = faces;
       reg.boundlines = boundaries[0];  // 主边界
       // ... 处理内边界（孔）...
       
       retRegs.push_back(reg);
   }
   ```

3. 处理多个边界的情况：
   - 一个区域可能有多个边界（如带孔的区域）
   - 外边界和内边界（孔）分别提取
   - `boundlines` 存储主边界，内边界可以存储在额外字段中

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
| `CMeshO` | 网格数据结构 |
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

### 7.1 SplitRegion

```cpp
struct SplitRegion {
    int mark;                                         // 平面 ID
    std::vector<int> faceIndices;                     // 包含的三角形索引
    std::vector<std::vector<vcg::Point3d>> polygons;  // 简单多边形
};
```

### 7.2 使用示例

```cpp
CMeshO mesh;
// ... 加载 mesh ...

JasMeshMarkAndSplit splitter;
splitter.SetMainMesh(&mesh);

std::vector<JasMeshMarkAndSplit::splitReg> regions;
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

// 也可以直接遍历 mesh，按 newMark 分组
std::map<int, std::vector<int>> markToFaces;
for (int i = 0; i < mesh.face.size(); i++) {
    if (!mesh.face[i].IsD()) {
        markToFaces[mesh.face[i].newMark].push_back(i);
    }
}
```

---

## 8. 性能考虑

### 8.1 时间复杂度

- Phase 1（构建边信息）：O(F)，F 为三角形数
- Phase 2（延长线切割并标记新区域）：O(F * C)，C 为切割边数量
  - 每个 curFaces 的 flood-fill：O(F)
  - 连接切割边为折线：O(C)，C 为切割边数量
  - 从端点延长切割：O(F)（每次延长切割 curFaces 中所有三角形）
  - 拣选子区域：O(F)
- Phase 3（根据新标记提取多边形）：O(F)

总体：O(F * C)，C 通常很小

### 8.2 空间复杂度

- 边信息存储：O(E)，E 为边数
- 折线存储：O(C)
- flood-fill 队列：O(F)
- Face 新增属性：O(F)（每个 face 一个 int）
- 输出结果：O(F)

---

## 9. 测试计划

### 9.1 单元测试

1. 简单平面 mesh（无切割边）
2. 两个平面相交（有 mark 不同边）
3. 带孔洞的 mesh（有边界边）
4. 非流形 mesh（有非流形边）

### 9.2 集成测试

1. 从实际 mesh 文件加载测试
2. 验证输出多边形的正确性
3. 验证拓扑一致性

---

## 10. 已确认事项

1. **mark 的来源**：mark 由外部定义，本模块只读取，不负责计算。
2. **多边形方向**：输出多边形的边以 mesh 中提取的边的方向为准。`std::vector<int>` 存储的是 curFaces 切割后得到的边的顶点索引。最终多边形方向与原始输入 mesh face 的方向一致。
3. **法向量计算**：`splitReg::normal` 与原始 mesh face 的法向量一致，不做额外计算。
4. **boundlines 格式**：`splitReg::boundlines` 存储的是边的顶点索引序列，与原始 mesh 的边方向一致。
