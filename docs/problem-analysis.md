# 项目问题分析（效率与正确性）

> 范围：`MeshCutByLine` 主仓库 + `external/cgalLocalMeshCut` 子模块。
> 基准：当前 `main`（提交 8290c11 / 子模块 1b6b520）。
> 目标：以代码为准，记录可复现/可由代码推断的效率问题、正确性风险与可维护性问题，作为后续优化的输入。

## 1. 关键路径概览

`SplitMeshByMarkAndEdge` 的最外层按区域循环，每个区域调用一次 `LocalMeshCutManager::cutRegion`。`cutRegion` 内部对**每条 NON_MANIFOLD 折线**执行一次 `AddCutLines`（CGAL corefine），多刀累积，最后一次性 `mergeBack` + `propagateExternal`。

成本集中点依次是：

1. 每条折线都要把**整个局部网格**复制到 CGAL 精确网格并 corefine（不是只切受影响区域）；
2. `AddCutLines` 内多处线性扫描/全量重建；
3. 折线连接阶段的若干平方级操作；
4. 每刀后全量 `FaceFace` 重算与区域重标。

---

## 2. 效率问题

### 2.1（高）每条折线都全量重切整个局部网格

- 位置：`tool/local_mesh_cut.h` 的 `cutRegion` 循环；`external/cgalLocalMeshCut/JasMeshAddCutLines.cpp` 的 `AddCutLines` → `BuildFullLocalMesh` → `Cut3D`。
- 现状：对第 k 条折线，`BuildFullLocalMesh` 把当前 `pMesh`（含前面 k-1 刀产生的所有分片）完整复制成精确网格，再与第 k 张辅助墙 corefine。整个区域被反复重切。
- 影响：`T_total = Σ_k T_corefine(区域_k)`，区域随刀数变大，近似 `O(刀数 × 区域面数)` 级别，且 corefine 常数很大（EPEC 精确构造）。
- 建议：无法轻易避免（每次切割几何都在变），但可考虑按空间范围裁剪局部网格、把互不重叠的多条折线分批并行、或仅在折线影响范围内子提取再 corefine 后回贴。

### 2.2（高）`AddCutLines` 顶点查找为线性扫描

- 位置：`JasMeshAddCutLines.cpp` 的 `FindCutVertex`，以及 `JasMeshLocalMarkAndCutSplit.cpp` 中 `RestoreToGlobal` 的 `find_cut_vertex`。
- 现状：对每条交线折线的每个点，都线性遍历 `cut_mesh.vertices()` 用精确坐标匹配。`cut.polylines` 点数是 `P`，网格顶点数是 `V`，复杂度 `O(P × V)`。
- 建议：corefine 的交线点就是网格顶点，可用 `ExactPoint -> Vertex_index` 的 `std::map`/`std::unordered_map`（ExactPoint 有序可做 map key）预建索引，把 `O(P×V)` 降到 `O(P log V)`。

### 2.3（高）每刀对每个 mark 重建边-面索引并 flood-fill

- 位置：`JasMeshAddCutLines.cpp` 步骤 5；`JasMeshLocalMarkAndCutSplit.cpp` 的 `SplitMarkRegionByCut`。
- 现状：每次 `AddCutLines` 都先遍历全部面收集 `existing_marks`，然后**对每个 mark** 调一次 `SplitMarkRegionByCut`；而该函数每次都从零重建 `edge_faces`（`O(F)`）并分配 `visited(F)`。
- 影响：`O(刀数 × mark 数 × F)`，mark 随切割增多而增多，二次膨胀。
- 建议：一次建好 `edge_faces` 复用；对多个 mark 复用同一 `visited` 位图（用代数标记而非清零）；或只处理被本次切割边影响的 mark。

### 2.4（中）折线连接存在平方级/更差路径

- 位置：`tool/polyline.h`。
- 现状：
  - `extendPolyline` 向前端 `vertexIndices.insert(begin(), otherVertex)` 为 `O(n)`，长折线总体 `O(n²)`；
  - `tryMergePolylines` 外层 `while(merged)`，内层双循环，命中的合并又 `erase` 向量元素并立即回到最外层重扫，最坏接近 `O(n³)`；
  - `connectByType` 的 `canonicalEdgeGroups` 对一条边的所有重复记录在扩展时整组标记，存在重复遍历。
- 建议：连接阶段先按端点建邻接表，线性时间拼接，前端插入改为收集后一次性反向写入；合并阶段用"端点 -> 折线"索引，一次合并一轮，避免反复重扫。

### 2.5（中）每刀重复建已有顶点坐标索引

- 位置：`JasMeshAddCutLines.cpp` 顶点解析前。
- 现状：每次 `AddCutLines` 都遍历 `pMesh` 全部顶点，把 double 坐标转 `ExactPoint` 插入 `existing_point_to_index`（`O(V log V)`）。多刀重复做同样的事。
- 建议：若局部网格在 `cutRegion` 生命周期内可共享索引，可把已有顶点精确坐标索引提升到 `cutRegion` 层维护并增量更新；或至少只对真正参与切割的顶点建索引。

### 2.6（中）每刀后全量 `FaceFace` 重算

- 位置：`JasMeshAddCutLines.cpp` 步骤 6；`tool/local_mesh_cut.h` 的 `finalizeGrow`。
- 现状：`AddCutLines` 结尾和 `finalizeGrow` 都调用 `UpdateTopology::FaceFace`，每次全量遍历。多刀场景下同一网格被反复全量重算邻接。
- 建议：`cutRegion` 内多刀不需要每次都全量重算 FF（`propagateExternal` 前一次即可），可把 `FaceFace` 移出 `AddCutLines` 或延后到 merge 后。

### 2.7（低）`propagateExternal` 的双重线性匹配

- 位置：`tool/local_mesh_cut.h`。
- 现状：每个新顶点遍历所有缝边 `pointOnSegment` 命中（`O(新顶点 × 缝边数)`）；分割后更新邻居映射时又对每个子面 `faceHasEdge`。单区域内通常规模不大。
- 建议：按缝边的包围盒建空间索引；`pointOnSegment` 的缝边向量可预计算一次。

### 2.8（低）调试输出全量写文件、随机颜色不可复现

- 位置：`JasMeshMarkAndCutSplit.cpp` 各 `debugWrite*`。
- 现状：每次迭代把当前区域/折线全量写出；`debugSaveColoredMesh` 每次 `std::srand(time(nullptr))` 后连续随机，颜色不可复现。
- 建议：调试默认关闭时无影响；开启时按需降采样。颜色改为按 `newMark` 的确定性哈希/调色板，便于跨运行对比。

### 2.9（低）`RegionMarker::floodFill` 每区域分配全局面位图

- 位置：`tool/region_marker.h`。
- 现状：每次 `floodFill` 都 `std::vector<bool> visited(mesh->face.size(), false)`，`O(F)` 分配与清零，区域数多时累计明显。
- 建议：复用带代号的 `std::vector<int>` 访问标记，避免每次清零。

---

## 3. 正确性 / 健壮性问题

### 3.1（高，未解决）非流形边在 `Cut3D` 中被静默丢弃

- 位置：`external/cgalLocalMeshCut/JasMeshLocalMarkAndCutSplit.cpp` 的 `Cut3D`。
- 现状：CGAL `Surface_mesh` 不允许非流形边，`add_face` 失败返回 `null_face`，代码未检查返回值，只把计数写入 `CutResult::dropped_input_face_count`。被丢的面不参与切割，仍保留原几何，后续区域重标却照常进行——几何未切但 mark 已拆。
- 影响：真实非流形边区域的切割结果不完整；多刀后几何/标记不一致。
- 建议：进入 corefine 前把非流形边“劈开”（复制顶点、拆成独立边）；或检测到 `dropped_input_face_count > 0` 时该刀保守 no-op 并告警，而不是静默继续。

### 3.2（高）非流形点（star vertex）会触发 corefine 崩溃

- 位置：输入约束，`Cut3D` 无显式校验。
- 现状：同一顶点描述符被多个不相连面环共享时，corefine 在 Debug 下断言失败（`Visitor.h:768`），Release 下访问冲突。`cutRegion` 通过“折叠零面积分片 + 清理孤立顶点”保证了**每次切割后**局部网格流形，但**未校验初始 localMesh 是否流形**；若全局输入本身含 star vertex，第一条折线就会崩。
- 建议：在 `extractLocalMesh` 后做流形校验（非流形边/顶点计数），含 star vertex 的局部区域保守跳过或先拆分；把校验作为硬前置条件。

### 3.3（中）切割墙只用一个面法向，非平面区域有误差

- 位置：`tool/local_mesh_cut.h` 的 `buildCutInput`。
- 现状：`normal` 取 `localFaceToGlobal[0]` 的面法向，不是区域面积加权平均（子模块 `BuildLocalMesh` 用了面积加权，但主仓库走 `AddCutLines` 时用的是这第一个面）。弯曲/非平面区域会随远离该面误差增大，切割墙与局部面夹角偏斜。
- 建议：与子模块一致，改为区域面积加权法向；极端曲率再按连通分量取局部法向。

### 3.4（中）折线延长长度为整个包围盒对角线，可能过度切割

- 位置：`buildCutInput`。
- 现状：`diagonalLength = box.Diag()`，两端延长都取该长度，保证“切穿区域”。对多条折线会反复用很长的墙扫过区域，既慢也容易切到不属于当前悬空方向的邻域。
- 建议：用该方向实际到区域边界的距离（方向包围盒投影）代替整对角线；或在命中区域边界后截断墙。

### 3.5（中）`seamExternal` 每条缝边只保留一个外部邻接面

- 位置：`tool/local_mesh_cut.h` 的 `extractLocalMesh`。
- 现状：`seamExternal[{a,b}] = adjacentFaceIndex` 是覆盖赋值。若一条缝边有多个外部邻接面（非流形边、或同一几何边因顶点重复产生多个邻面），只保留最后一个；`propagateExternal` 只分割一个外部面，另一侧仍旧裂缝。
- 建议：值改为 `std::vector<int>`，传播时对所有邻接面做纯分割；并保持已实现的“分割后更新邻居映射”。

### 3.6（中）`boundaryVertices` 悬空判断是启发式

- 位置：`cutRegion` 收集 `boundaryVertices` 与 `extendStart/extendEnd`。
- 现状：把“非 NON_MANIFOLD 折线的顶点”当作边界顶点集，端点不在其中即判定悬空并延长。该规则依赖折线类型构成，遇到端点恰好与 MARK_DIFF/BOUNDARY 顶点重合但语义不同的几何，可能误判延长方向或漏延长。
- 建议：记录测试覆盖；必要时用“该顶点在区域内是否只有折线一侧有邻面”做几何判定，而不是仅依赖折线类型。

### 3.7（中）`mergeBack`/`propagateLocalRegionMarks` 的顺序索引映射较脆弱

- 位置：`tool/local_mesh_cut.h`。
- 现状：`propagateLocalRegionMarks` 用 `localFaceIndex - numOriginFaces` 顺序对齐 `merge.newFaceGlobals`。当前因“孤立顶点清理 + 折叠 + 去重”保证了追加面顺序一致，但任何一处跳过追加面（例如未来新增过滤）都会让后续面的 new-mark 整体错位。
- 建议：改为显式 `localFaceIndex -> globalFaceIndex` 映射（`mergeBack` 返回 map 或记录每个追加面的 local 来源），消除对“连续追加”的隐含假设。

### 3.8（中）精确算术与 double 混用的容差风险

- 位置：`JasMeshAddCutLines.cpp` 的折叠阈值 `1e-12`、新顶点按 `ExactPoint` 精确合并；`tool/local_mesh_cut.h` 的 `pointOnSegment` 距离容差 `1e-7`。
- 现状：几何判定同时存在“精确相等”与“绝对容差”两套标准，尺度相关。极小/极大坐标网格下，绝对容差可能误判（此前质心同侧判定的绝对容差已证实会误挂父面，现已移除；`pointOnSegment` 仍有同类隐患）。
- 建议：统一为相对容差（相对边长/包围盒对角线），或全链路以精确点为准，仅在最终转 double 时做一次控制。

### 3.9（低）全局静态状态与随机数非线程安全

- 位置：`JasMeshLocalMarkAndCutSplit.cpp` 的 `static int debug_call_index`；`debugSaveColoredMesh` 的 `std::srand`。
- 现状：`debug_call_index` 为静态递增，多线程同时切割会竞争；`srand/rand` 为全局状态。当前单线程可运行，但未来并行化是隐患。
- 建议：计数用局部参数传入；颜色用 `std::mt19937` 局部实例或确定性哈希，不用 `srand`。

### 3.10（低）子模块 `RestoreToGlobal` 的 `CompactEveryVector` 会重排索引

- 位置：`JasMeshLocalMarkAndCutSplit.cpp` 的 `RestoreToGlobal`（另一 API `CutLocalMesh` 使用）。
- 现状：`CompactEveryVector` 压缩已删面/顶点，外部持有的 face/vertex 下标会失效。该路径与主仓库 `AddCutLines` 流程不同，但同库维护。
- 建议：在契约中明确 `CutLocalMesh` 会整体重排并返回映射，或改为不压缩、显式标记删除，与 `AddCutLines` 的“不 SetD/不重排”语义对齐。

### 3.11（低）异常安全

- 位置：`JasMeshAddCutLines.cpp` 分片收集使用 `vertex_to_global.at(...)`。
- 现状：若 `FindCutVertex`/顶点映射出现不变量破坏，`.at` 会抛异常而非返回错误；`FindCutVertex` 找不到时返回空索引、后续 `.find` 跳过，路径不一致。
- 建议：统一为显式检查并记录错误/返回失败，避免核心切割中抛异常导致半成品网格。

---

## 4. 代码卫生 / 死代码

- `tool/cut_plane.h` 的 `CutPlaneManager`（`makeCutPlane`、`isOnMarkDiffEdge`、`signedDistance`、`intersectSegmentPlane`）已不在主流程使用，仅被 `testMakeCutPlane*`、`testIsOnMarkDiffEdge`、`testSignedDistanceAndIntersection` 覆盖，属于遗留测试/桩。
- `JasMeshMarkAndCutSplit.cpp` 的 `debugWriteSubRegionsOFF` 已无调用（Phase 2 不再有 `extractSubRegions`/`markSubRegions`），是死代码。
- `JasMeshMarkAndCutSplit.h` 的成员 `std::vector<vcg::Point3i> m_edgeMarks` 未使用。
- `tool/edge_info.h` 的 `getAdjacentFaces`、`isCutEdge` 未在主流程使用（`findCutEdges` 只用 `getEdgeType`）。
- 主仓库 `CMakeLists.txt` 使用全局 `include_directories`（vcglib、eigen、根目录），而非 target-scope，工程卫生一般，且依赖绝对路径外部的 CGAL/Boost（`PATH.txt` 记录）。

---

## 5. 测试有效性问题

- 主仓库测试使用裸 `assert`，MSVC Release 定义 `NDEBUG` 后全部失效。此前多折线流形断言在 Release 下实际不拦截，只能靠打印人工确认。
- 缺少独立的“流形校验”测试入口：`CollectTempManifoldStats` 这类检查目前只存在于诊断过程，未固化为可在 CI 的 Debug 下必然拦截的断言。
- 主仓库用裸 `assert` + `main()` 顺序执行，子模块用 Catch2，风格不一致；无自动化 CI 配置。
- 没有大规模网格或性能基准测试，平方级路径（折线连接、每刀重切）在测试集规模下不会暴露。

---

## 6. 改进优先级建议

1. **正确性优先**：处理非流形边丢面（3.1）与初始 star vertex 校验（3.2），把 `dropped_input_face_count > 0` 转成显式行为。
2. **消除高成本重复**：合并 2.2/2.3/2.6，把顶点索引、边-面索引、FF 重算移出逐刀/逐 mark 的重复路径。
3. **测试固化**：引入可在 Debug 下生效的流形断言 + 非流形输入回归 + 性能基准。
4. **代码清理**：删除 4 中死代码，统一 `AddCutLines` 与 `CutLocalMesh` 的索引/删除语义。
5. **数值一致性**：统一容差与法向策略（3.3、3.8）。
