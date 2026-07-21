// tool/edge_info.h
#ifndef EDGE_INFO_H
#define EDGE_INFO_H

#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>

// Forward declaration - CMeshO is defined in cmesh.h with vcglib dependency
class CMeshO;

namespace MeshCutByMark {

// 切割边类型
enum CutEdgeType {
    CUT_EDGE_NONE,           // 普通边（非切割边）
    CUT_EDGE_MARK_DIFF,      // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD,   // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY        // 只被 1 个三角形使用（孔洞边缘）
};

// 切割边信息
struct CutEdge {
    int v0, v1;              // 端点顶点索引 (v0 < v1)
    int faceIdx;             // 所属面索引
    int edgeIdx;             // 边在面中的索引 (0, 1, 2)
    CutEdgeType type;        // 切割边类型
};

// 边的哈希函数
struct EdgeHash {
    size_t operator()(const std::pair<int,int>& e) const {
        int lo = std::min(e.first, e.second);
        int hi = std::max(e.first, e.second);
        return std::hash<int>()(lo) ^ (std::hash<int>()(hi) << 1);
    }
};

// 边的相等比较函数（归一化后比较，确保 {a,b} == {b,a}）
struct EdgeEqual {
    bool operator()(const std::pair<int,int>& a, const std::pair<int,int>& b) const {
        auto na = std::minmax(a.first, a.second);
        auto nb = std::minmax(b.first, b.second);
        return na == nb;
    }
};

// 边信息管理器
class EdgeInfoManager {
public:
    // 构建边信息
    void buildEdgeInfo(CMeshO* mesh);

    // 获取边的类型
    CutEdgeType getEdgeType(int v0, int v1) const;

    // 获取边的所有邻接面
    std::vector<int> getAdjacentFaces(int v0, int v1) const;

    // 检查边是否是切割边
    bool isCutEdge(int v0, int v1) const;

private:
    CMeshO* m_mesh;
    std::unordered_map<std::pair<int,int>, std::vector<int>, EdgeHash, EdgeEqual> m_edgeToFaces;
    std::unordered_map<std::pair<int,int>, CutEdgeType, EdgeHash, EdgeEqual> m_edgeTypes;
};

} // namespace MeshCutByMark

#endif // EDGE_INFO_H
