# 局部独立切割 + 统一拼接缝合 设计方案

> 状态：设计草案（未开始实现）
> 范围：`MeshCutByLine` 主仓库，涉及 `external/cgalLocalMeshCut` 子模块
> 目标：把当前“串行逐区域切割、切一个区域立即改邻居”的流程，改为“局部单元独立切割、并行计算、最后统一缝合区域拼接边”。

## 1. 背景与动机

当前 `JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge` 的主循环是：

1. `floodFill` 得到一个连通区域 `curFaces`；
2. `findCutEdges` + `connectEdgesToPolylines` 得到切割折线；
3. `cutRegion` 对该区域切割，并在 `propagateExternal` 中**立即**把拼接边上的新顶点加到外部邻接面；
4. 进入下一个区域。

存在三个问题：

- **串行依赖**：一个区域切割后要立刻改它的邻居面，区域处理顺序会影响结果，也阻止了并行。
- **拼接边单侧传播且容差脆弱**：`propagateExternal` 用 double 坐标的 `pointOnSegment` 判断新顶点是否落在拼接边上，并依赖“分割后更新缝边→邻居面映射”；同缝边多顶点、同一外部面多条缝边等情形已经证明很容易漏缝。
- **重复转换**：每条折线都做一次 VCG(double) ↔ CGAL(exact) 全量往返并重切整个局部网格。

本方案把 `curFaces` 视为**局部单元**，局部切割完全独立；切割过程中只记录“拼接边上的切点”，最后统一按精确坐标缝合。

## 2. 核心概念

### 2.1 局部单元

局部单元就是现有 `floodFill` 得到的 `curFaces`，不重新设计区域划分。一个局部单元包含：

- 一个连通的面集合；
- 一组切割折线（由 `findCutEdges` + `connectEdgesToPolylines` 得到）；
- 一组拼接边（该局部单元与其他局部单元共享的边）。

这里的“拼接边”指**区域与区域之间的共享边**，不是整个网格的孔洞外边界。

### 2.2 拼接边切点

局部单元被切割后，某条拼接边可能被切割线分成若干段。每个分段的端点称为拼接边上的切点。切点用精确坐标表示，并按该拼接边的方向排序。

两个相邻局部单元在共享边上由同一几何切割，切点的精确坐标必然一致，这是合并阶段可以对齐的前提。

## 3. 目标流程

```mermaid
flowchart LR
    A["flood-fill 得 curFaces<br/>（局部单元）"] --> B["劈开非流形边/点"]
    B --> C["转 CGAL 精确网格<br/>记录拼接边"]
    C --> D["局部内多刀切割<br/>按切割边分区"]
    D --> E["输出局部面集<br/>+ 拼接边切点表<br/>+ 局部 newMark"]
    E --> F["收集所有拼接边切点<br/>按精确坐标去重"]
    F --> G["两侧邻接三角形<br/>按切点细分"]
    G --> H["写回全局网格<br/>局部 newMark 映射为全局 newMark"]
```

## 4. 数据结构

### 4.1 局部计算结果

每个局部单元计算完成后输出一个独立结构：

```cpp
struct LocalCutResult
{
    std::vector<int> faceGlobals;          // 该局部单元涉及的全局面下标
    // 切好的局部网格（CGAL 精确网格 + 全局顶点/面映射）
    jaslmc::ExactMesh cutMesh;
    std::vector<int> vertexToGlobal;       // 局部顶点 -> 全局顶点下标
    // 局部区域标记（后续映射为全局 newMark）
    std::vector<int> faceLocalMark;

    // 拼接边切点表：全局拼接边 -> 沿边有序的精确切点
    std::map<std::pair<int, int>, std::vector<ExactPoint>> seamCutPoints;
};
```

### 4.2 拼接边切点表

- key：拼接边两个全局端点，规范化后排序；
- value：该边上被切割产生的新顶点，按沿边方向排序，只存**内部切点**，不含端点；
- 同一局部单元内同一条拼接边只产生一张有序表。

### 4.3 局部 newMark 与全局 newMark

局部单元并行计算时各自从局部号开始标，可能冲突。合并阶段维护：

```cpp
std::map<std::pair<int, int>, int> localMarkToGlobalMark;
```

其中第一个 int 是局部单元 id，第二个 int 是局部 mark；合并时统一分配全局 newMark。

## 5. 模块划分与现有代码映射

| 现有位置 | 角色 | 变化 |
|---|---|---|
| `SplitMeshByMarkAndEdge` 主循环 | 生成局部单元 | 保留 flood-fill / 找切割边 / 连折线 |
| `extractLocalMesh` | 提取局部 | 保留；增加劈开非流形边/点、记录拼接边切点容器 |
| `cutRegion` 逐折线循环 | 局部切割 | 改为在 CGAL 精确网格上一次完成多刀，不再每刀 VCG↔CGAL 往返 |
| `propagateExternal` | 邻居加点 | 删除；替换为“全局拼接边切点合并 + 双侧细分” |
| `mergeBack` | 局部写回全局 | 上移并改造为“多局部统一写回” |
| `propagateLocalRegionMarks` | 局部标记同步 | 改为合并阶段的“局部 newMark → 全局 newMark”映射 |
| `AddCutLines` / `Cut3D`（子模块） | 核心切割 | 保留 f:source 来源标记、零面积折叠、孤立顶点清理思想，作用域改为一个局部单元 |

## 6. 并行与串行边界

可并行：

- 每个局部单元的劈边、精确网格构建、多刀切割、局部分区、生成 `LocalCutResult`。

必须串行：

- 收集所有局部单元的拼接边切点；
- 建立“精确坐标 → 全局顶点”的唯一映射；
- 把局部结果写回全局 VCG 网格；
- 分配全局 newMark。

原因：最终只有一个全局网格，写回与改标记不能并发。

## 7. 前置条件与风险

### 7.1 非流形边/点必须先在局部内劈开

CGAL `Surface_mesh` 不接受非流形边（`add_face` 失败），也不稳定支持 star vertex。局部单元进精确网格前必须：

- 非流形边：复制端点，拆成独立边；
- star vertex：复制顶点，让每个扇面各用各的副本。

否则会出现丢面（`dropped_input_face_count` 增长）或 corefine 断言/访问冲突。

### 7.2 拼接边两侧切点的一致性

拼接边两侧的局部单元独立切割时，切点来自同一几何。为保证一致：

- 局部内切割必须用同一精确核；
- 拼接边切点必须以精确坐标比较和排序，不得转回 double 再比较。

### 7.3 区域角点

同一个全局顶点可能同时是多条拼接边的端点。拼接阶段应先按顶点统一去重，再按边细分；否则角点会被复制出多个重合顶点，重新引入非流形点。

### 7.4 局部 newMark 冲突

多个局部并行会各自产生局部 mark。合并阶段必须统一映射为全局 newMark，不能直接沿用局部号。

## 8. 迁移步骤

建议分四步，每步保持可编译、测试通过：

1. 定义 `LocalCutResult` 与拼接边切点表，先只收集数据、不改变现有切割行为；
2. 把 `cutRegion` 内部改为 CGAL 精确网格一次多刀，局部结果独立，暂不并行；
3. 把 `propagateExternal` 替换为“收集切点 + 双侧细分”，验证无裂缝；
4. 引入局部并行，并把合并/写回/标记段保持串行。

## 9. 待确认问题

- 局部单元是否严格按现有 `floodFill` 的 mark 连通分量，还是允许更大的区域合并以降低边界数量；
- 拼接边切点由哪一侧负责创建、是否允许两侧各创建一份后在合并阶段合并；
- 是否保留现有 `splitReg::boundlines/boundaries` 输出契约不变。
