# 调试输出功能开发经验

## 背景

在开发 MeshCutByMark 网格分割算法时，需要可视化中间结果来验证算法正确性。算法包含多个阶段（flood-fill、折线连接、切割、子区域提取），每个阶段都会产生中间数据结构。

## 设计决策

### 1. 文件格式选择

| 数据类型 | 选择格式 | 原因 |
|----------|----------|------|
| 折线 | OBJ (l 元素) | OBJ 原生支持线段，MeshLab 等工具可直接可视化 |
| 三角形网格 | OFF | OFF 是最简单的三角形网格格式，易于解析 |
| 多边形 | OBJ (f 元素) | OBJ 支持任意多边形面 |
| 带颜色网格 | OBJ + MTL | 面级 usemtl 材质（按 NewMark 着色） |

**经验**：选择格式时优先考虑工具链支持（MeshLab、Blender 等），而非格式的完备性。

### 2. 命名规范

采用 `iter_N_suffix.ext` 的命名模式：
- `N` 为主循环迭代编号，便于追踪算法执行流程
- `suffix` 描述数据含义（cur_faces、polylines、sub_region_J）
- 同一迭代的文件自然归组，便于批量查看

**经验**：调试文件命名应包含上下文信息（迭代编号、阶段名称），避免覆盖历史输出。

### 3. 顶点索引重映射

输出子网格时需要将全局顶点索引映射为局部索引：
```cpp
std::vector<int> vertMap;           // 全局 -> 局部映射
std::unordered_map<int, int> globalToLocal;
```

**经验**：OBJ/OFF 格式要求顶点从 0 开始连续编号，输出子网格时必须重映射。使用 unordered_map 保证 O(1) 查找。

### 4. 颜色管理方案

使用 `std::map<int, vcg::Color4b>` 管理区域颜色：
- key 为 `splitReg::newMark`，每个区域（NewMark）一种颜色
- 随机颜色避免固定调色板在区域数多时重复
- 输出采用面级颜色：OBJ + MTL（每个 NewMark 一个材质，面通过 `usemtl` 引用），
  不再输出顶点颜色，避免接缝处颜色平均造成区域边界模糊

**经验**：随机颜色虽然每次运行不同，但同一运行内各区域颜色唯一，足够用于调试。

## 实现细节

### 调试开关设计

```cpp
bool m_debug = false;                            // 默认关闭
std::string m_debugOutputDir = "debug_output/";  // 默认目录
int m_debugIterCounter = 0;                      // 自动递增
```

**经验**：调试功能默认关闭，避免影响正常运行性能。输出目录可配置，方便不同场景使用。

### 文件写入模式

所有输出方法都遵循相同模式：
```cpp
void debugWriteXxx(...) {
    if (!m_debug || data.empty()) return;  // 前置检查
    debugEnsureDir();                       // 确保目录存在
    std::ofstream ofs(path);               // 打开文件
    if (!ofs.is_open()) return;            // 容错处理
    // ... 写入逻辑 ...
    ofs.close();
}
```

**经验**：调试代码不应影响主逻辑的异常处理，所有 I/O 操作都需要容错。

### 输出时机

在算法关键步骤后立即输出：
- flood-fill 后 → 输出连通区域
- 折线连接后 → 输出折线
- 切割（cutRegion）后 → 区域重标由 AddCutLines 在 local mesh 完成，不再单独输出子区域文件
- 最终结果 → 输出边界多边形和带颜色网格

**经验**：输出时机应选择在数据结构完整但未被后续修改的时刻，确保输出内容与算法状态一致。

## 踩坑记录

### 1. OBJ 索引从 1 开始

OBJ 格式的顶点索引从 1 开始，而 C++ 数组从 0 开始。写入时需要 `+1`：
```cpp
ofs << " " << (globalToLocal[vi] + 1);
```

### 2. OFF 格式 header

OFF 格式需要严格的 header：
```
OFF
顶点数 面数 0    # 第三个数是边数，通常为 0
```

### 3. 面颜色输出（OBJ + MTL）

VCGlib 的面颜色是 `Color4b`（0-255），写入 MTL 的 `Kd` 需除以 255 转 0~1：
```cpp
materialStream << "Kd " << (color.X() / 255.0f) << " " << ...;
```
OBJ 面行通过 `usemtl newmark_N` 引用材质；顶点行只写坐标，不写颜色。

### 4. 多边形 OBJ 的全局偏移

当多个区域写入同一个 OBJ 文件时，顶点索引需要累加偏移：
```cpp
ofs << " " << (globalToLocal[vi] + 1 + globalVertOffset);
globalVertOffset += vertMap.size();
```

## 改进建议

1. **增量输出**：当前每次迭代都写文件，大网格时可能很慢。可考虑只在 debug level 高时输出详细数据。
2. **二进制格式**：文本 OBJ/OFF 对大网格效率低，可考虑 PLY 二进制格式。
3. **可视化集成**：可开发 MeshLab 插件直接加载调试输出，无需手动打开文件。
4. **时间戳**：文件名加入时间戳，避免多次运行时覆盖。
