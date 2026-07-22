# Phase 2.4 局部 mesh 切割改造设计

- 日期：2026-07-22
- 状态：待评审
- 关联：`2026-07-21-mesh-cut-by-mark-design.md` §4.4、`JasMeshMarkAndCutSplit.cpp` Phase 2.4

## 1. 背景与动机

当前 Phase 2.4「从端点延长切割」用 `makeCutPlane` 在 `NON_MANIFOLD` 折线的悬空端点构造切割平面，再用 `cutTriangleByPlane` 逐三角形切开（见 `tool/cut_plane.h`）。

问题：

- `cutTriangleByPlane` 是**存根**，只算有符号距离，不真正切。
- 即便补全，"逐三角形平面切割 + 手工维护 FF 邻接"很脆弱：符号退化情形（顶点恰在面上、距离为 0）、全网格索引失效、FF 拓扑不一致都是雷区。

本设计把 Phase 2.4 改成**「提取局部 mesh → 调用稳定 cutter → 合并回主网格」**的管线，避开上述问题。

## 2. 目标 / 非目标

**目标**

- 用 `JasMeshAddCutLines::AddCutLines`（稳定黑盒）替换 `cutTriangleByPlane` 路径。
- 切完后 `extractSubRegions` 能正确把平面两侧分成不同子区域。
- 保持网格 watertight（curFaces 边界处不裂开）。

**非目标**

- 不实现 `AddCutLines` 本身（稳定、外部提供）。
- 不改 Phase 1（边分类）、Phase 2.1~2.3（flood-fill / 连折线）、Phase 3（提取多边形）。
- `makeCutPlane` / `cutTriangleByPlane` 后续可删除或保留复用其方向计算，不在本设计强制。

## 3. 契约：`JasMeshAddCutLines::AddCutLines`

```cpp
void AddCutLines(CMeshOD *pMesh,
                 vcg::Point3d &normal,
                 std::vector<vcg::Point3d> &line,
                 std::vector<int> &cutLine);
```

- **黑盒**：不读其实现；调用前**无需预处理**（FF 邻接、法向由 cutter 内部算）。
- `pMesh`：**区域局部三角形**（必须是隔离的局部 mesh，不能直接传 `m_pMesh`）。
- `normal`：这些三角形的法向（单一，区域共面）。
- `line`：**补充切割线**（输入）——本设计里只放悬空端点的延长段。
- `cutLine`：**输出**，新切割顶点在 pMesh 中的下标序列（有序）。
- **切完后 pMesh 的状态**：
  - 被分裂的原始三角形**直接被替换**（不是 `SetD`，槽位被新面接管 / 不再保留）。
  - 新增三角形带「来源原始面下标」标记（cutter 写在 face 上，merge 时读）。
  - 原始顶点下标不变，新顶点 append 在末尾。

## 4. 总体方案

对每个 `curFaces` 区域（flood-fill + 连折线之后），Phase 2.4 改为：

```
A. 提取局部 mesh       localMesh ← curFaces + 其顶点（重映射），记 Nv0/Nf0，抓取边界缝信息
B. 切割               每个悬空端点：构造 line+normal → AddCutLines(localMesh, ...)
C. merge 回 m_pMesh    新顶点/新面 append；被分裂的原始面 SetD()
E. 外部加点           落在 curFaces 边界边上的新顶点 → 插入外部邻接面
F1. 重算 FF           UpdateTopology::FaceFace(m_pMesh)
D. 标分割边           cutLine → global → 对应 face-edge 的 FFp 置空
F2. 收尾              重建 curFaces、resize m_newMark
```

> 顺序很重要：**E 必须在 F1（重算 FF）之前**（E 还会加面）；**D 必须在 F1 之后**（D 依赖完整 FF，再去置空）。

## 5. 详细步骤

### A. 提取局部 mesh

输入：`m_pMesh`、`curFaces`。

1. 新建 `CMeshOD localMesh`。
2. 收集 `curFaces` 引用到的顶点（去重），建映射：
   - `globalVertToLocal`：global vert idx → local vert idx
   - `localToGlobalVert`：local vert idx → global vert idx（原顶点部分）
3. `AddVertices` + 拷贝坐标。
4. 逐个 `curFaces` 面 `AddFace`，顶点引用重映射到 local 下标，拷贝 `IMark`。
5. 记 `Nv0 = localMesh.vert.size()`（新顶点判定用，可靠）；`Nf0 = face.size()` 仅作参考（新面判定改用来源下标，见 C）。
6. 记 `localFaceToGlobal[i] = curFaces[i]`。
7. **抓取边界缝信息**（给 E 用）：遍历 `curFaces` 每条边，若在原 `m_pMesh` 里 `FFp` 指向 curFaces 外部的面，记录「该边（global 顶点对）→ 外部邻接面 global idx」。在改动 `m_pMesh` 之前抓。
8. **不算** FF / 法向（cutter 自己算）。

### B. 切割

对每条 `NON_MANIFOLD` 折线（`polyline.type == CUT_EDGE_NON_MANIFOLD`），每个端点（start / end）满足 `!isOnMarkDiffEdge(endFaceIdx, endEdgeIdx)`：

1. `P` = 端点顶点世界坐标。
2. `D` = 外延方向，沿用 `makeCutPlane`：start 用 `normalize(v[0]-v[1])`、end 用 `normalize(v[last]-v[last-1])`。
3. `L` = `localMesh` 包围盒对角线（足够长，确保切线横穿到区域边界）。
4. `line = { P, P + D*L }`。
5. `normal` = 区域法向（取 `m_pMesh->face[curFaces[0]].N()`，区域共面）。
6. 调 `AddCutLines(&localMesh, normal, line, cutLine)`。
7. 保存本次 `cutLine`（local 下标）供 D 用。

多个悬空端点 → 多次调用，都作用在同一个 `localMesh` 上。append-only，早期 `cutLine` 下标始终有效。

### C. merge 回 m_pMesh

> 新面的判定**靠来源下标，不靠下标范围**：cutter 可能把分裂出的某个新面写回原槽位（替换），所以 local `≥ Nf0` 不可靠。一律读 face 上的「来源原始面下标」：来源 = 自身 → 未动的原始面；来源 ≠ 自身 → 新面。

1. 构建统一顶点映射 `localVertToGlobal`：
   - local `< Nv0` → `localToGlobalVert[idx]`
   - local `≥ Nv0` → 新 append 到 `m_pMesh.vert` 后的全局下标（新顶点是 append 的，范围可靠；先 append 所有新顶点，记 `newVertLocalToGlobal`）。
2. 遍历 localMesh 所有面：
   - 来源 = 自身：对应 `m_pMesh` 原始面，**不动**。
   - 来源 ≠ 自身（新面）：append 到 `m_pMesh.face`——
     - 3 个顶点引用按 `localVertToGlobal` 重映射。
     - `IMark` = 继承区域 `targetMark`。
     - 经 `localFaceToGlobal[来源]` 得到 `originGlobal`，加入 `splitOrigins`。
3. 对 `splitOrigins` 中每个 `g`：`m_pMesh->face[g].SetD()`（被新面接管）。

### E. 外部加点

目的：新切割顶点若落在 `curFaces` 的边界边上（该边与 curFaces 外部面共享），外部邻接面也要在同位置加点，否则出现 T-junction / 裂缝。

对每个新顶点 `v_new`（local `≥ Nv0`，已映射到 global）：

1. 判断它落在哪条 local 原始边上（点-在线段上判定，结合 A 步抓的边界缝信息）。
2. 若该边是边界缝边（A 步记录了 external 邻接面）：
   - 取外部邻接面 `extFace`（global）。
   - 在 `v_new`（global）处把 `extFace` 一分为二（原 1 三角形 → 2 三角形，或按边加点），新面继承 `extFace` 的 mark。
   - 标记 `extFace` `SetD()`。

> 典型情况：延长线只在穿出 `curFaces` 的那一条边界边上产生 1 个需要传播的新顶点；内部新顶点无需传播。

### F1. 重算 FF

```cpp
vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*m_pMesh);
```

在 C、E 都加完面之后做。`FaceFace` 会跳过 `IsD()` 面，为所有非删除面（含新面）建立 FF 邻接。

### D. 标分割边

把切割路径上的边置成边界（`FFp` 自指），让 `isCutEdge` 返回 true，flood-fill 不再跨越。

切割路径顶点 = `[端点 P] + cutLine`。要标记的边：

1. **(P, cutLine[0])**：端点到第一个新切割顶点（cutLine 非空时）。
2. **(cutLine[i], cutLine[i+1])**：cutLine 相邻新顶点。
3. 最末段到达 curFaces 边界：该边界边天然是区域分界（不同 mark），无需额外标记。

对每条待标记边 `(ga, gb)`：

1. 找到 `m_pMesh` 中以 `ga, gb` 为顶点的 face-edge（重算 FF 后 `FFp` 可达对方面）。
2. 把两侧面的 `FFp` 都置为自指（`&face` 且 `FFi = edge`），断开连接 → 双方都成边界 → 屏障。

### F2. 收尾

1. 重建 `curFaces`：移除被 `SetD()` 的原始下标，加入 C 步 append 的新面下标。（E 步加的外部面不属于本区域，不加进 `curFaces`。）
2. `m_newMark.resize(m_pMesh->face.size())`，新增面初始化为 0（供 `extractSubRegions` 处理）。
3. 交还控制权给主循环，`extractSubRegions(curFaces, ...)` / `markSubRegions` 照常跑。

## 6. 关键数据结构

- `globalVertToLocal` / `localToGlobalVert` / `newVertLocalToGlobal` → 合成 `localVertToGlobal`。
- `localFaceToGlobal`：local face idx → global face idx（仅原始面 `[0,Nf0)`）。
- `Nv0`：原顶点数量，区分新旧**顶点**（可靠，新顶点 append）。
- 边界缝表：`(globalVertA, globalVertB) → extGlobalFace`（A 步抓取）。
- 来源原始面下标：cutter 写在 localMesh 每个 face 上。**这是区分新旧面的唯一可靠信号**（来源=自身为原始面，≠自身为新面）。accessor 实现时定，见 §9。

## 7. 关键决策与假设

| 项 | 决策 | 备注 |
|----|------|------|
| `line` 内容 | 仅悬空端点延长段 `{P, P+D·L}` | 用户确认 |
| `normal` | 单一区域法向 | 区域共面 |
| 延长长度 `L` | localMesh 包围盒对角线 | 保证切穿到边界 |
| 每端点一次调用 | 是 | 多端点多次调同 localMesh |
| `cutLine` 语义 | 新切割顶点有序序列，相邻成边 | 用户原话「新增边=新顶点索引」 |
| 端点→首新顶点 边 | 单独标记（cutLine 不含端点） | 见 D.1 |
| mark 继承 | 新面继承区域 `targetMark` | 保证 flood-fill 归属正确 |
| 分割边表示 | `FFp` 自指（沿用 `isCutEdge` 判定） | 不引入新属性 |
| 被分裂原始面 | cutter 直接替换；我们在 m_pMesh `SetD` | 不 compact，保留下标稳定 |
| 新面判定 | 靠来源下标（≠自身），**不靠** local 下标范围 | cutter 可能复用原槽位 |
| FF 重算 | 整网格 `UpdateTopology::FaceFace` | 在 C/E 之后、D 之前 |

## 8. 与下游集成

- `extractSubRegions` / `floodFill`：靠 `isCutEdge`（FFp 自指/空）判断屏障。D 步置空后，切割路径两侧不再连通 → 正确分出子区域。
- `m_newMark`：F2 resize 后新面为 0，`extractSubRegions` 会处理。
- `curFaces`：F2 重建后含新面下标，下游迭代正确。
- 外部邻接面（E 步分裂的）：不进 `curFaces`，保持其原 mark；它们与区域的边界天然是分界。

## 9. 边界情况与风险

- **cutter 切不动 / line 不穿过任何三角形**：`cutLine` 为空 → 跳过 D，该端点不延长。可接受（保守不切）。
- **延长段同时穿出多条边界边**：E 步对每个落点分别传播。
- **区域不共面**（mark 相同但法向不一）：取单法向可能偏；目前假设同 mark 共面，若实际不共面需改用逐面法向（后续视情况）。
- **FF 重算成本**：整网格 O(n)；区域多/大时有开销，必要时可局部重算（本设计先整网格）。
- **来源下标**：cutter 在 localMesh 每个 face 上写「来源原始面下标」。语义：来源=自身 → 未动的原始面；来源≠自身 → 新面，指向它分裂自的那个原始面（local 下标）。具体 accessor 实现时与 cutter 侧对齐（PerFaceAttribute 或新增 OCF 组件）。

## 10. 测试策略

- **A 单测**：给 `curFaces`，验证 localMesh 顶点/面数、映射正确、`Nv0/Nf0` 正确。
- **B 单测**：给定折线 + 端点，验证 `line` 两点方向/长度、`normal` 选取正确（可用 stub `AddCutLines` 记录入参）。
- **C 单测**：造一个已知切割结果的 localMesh（手工填新顶点/新面 + 来源标记），验证 merge 后 `m_pMesh` 新面正确、被分裂原始面 `SetD`、顶点引用重映射正确。
- **D 单测**：给定 `cutLine`，验证对应 face-edge 的 FFp 被置自指、`isCutEdge` 返回 true。
- **E 单测**：造一个落在边界边上的新顶点，验证外部邻接面被正确一分为二、mark 继承。
- **集成测试**：含 `NON_MANIFOLD` 折线、端点悬空的场景，端到端验证 `extractSubRegions` 分出 ≥2 个子区域、网格 watertight（无 T-junction）。
