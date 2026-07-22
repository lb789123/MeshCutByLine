// tool/edge_info.h
#ifndef EDGE_INFO_H
#define EDGE_INFO_H

#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>

#include "cmesh.h"

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

// --- Method implementations ---

inline void EdgeInfoManager::buildEdgeInfo(CMeshO* mesh) {
    m_mesh = mesh;
    m_edgeToFaces.clear();
    m_edgeTypes.clear();

    // Traverse all triangles, build edge->face mapping
    for (int i = 0; i < (int)mesh->face.size(); i++) {
        if (mesh->face[i].IsD()) continue; // skip deleted faces

        for (int j = 0; j < 3; j++) {
            int v0 = mesh->face[i].V(j)->Index();
            int v1 = mesh->face[i].V((j + 1) % 3)->Index();

            // Normalize so v0 < v1
            if (v0 > v1) std::swap(v0, v1);

            m_edgeToFaces[{v0, v1}].push_back(i);
        }
    }

    // Classify each edge
    for (const auto& entry : m_edgeToFaces) {
        const auto& edge = entry.first;
        const auto& faces = entry.second;
        CutEdgeType type = CUT_EDGE_NONE;

        if (faces.size() == 1) {
            type = CUT_EDGE_BOUNDARY;
        } else if (faces.size() >= 3) {
            type = CUT_EDGE_NON_MANIFOLD;
        } else if (faces.size() == 2) {
            // Check if the two faces have different marks
            int mark0 = mesh->face[faces[0]].IMark();
            int mark1 = mesh->face[faces[1]].IMark();
            if (mark0 != mark1) {
                type = CUT_EDGE_MARK_DIFF;
            }
        }

        m_edgeTypes[edge] = type;
    }
}

inline CutEdgeType EdgeInfoManager::getEdgeType(int v0, int v1) const {
    auto key = std::minmax(v0, v1);
    auto it = m_edgeTypes.find(key);
    if (it != m_edgeTypes.end()) {
        return it->second;
    }
    return CUT_EDGE_NONE;
}

inline std::vector<int> EdgeInfoManager::getAdjacentFaces(int v0, int v1) const {
    auto key = std::minmax(v0, v1);
    auto it = m_edgeToFaces.find(key);
    if (it != m_edgeToFaces.end()) {
        return it->second;
    }
    return {};
}

inline bool EdgeInfoManager::isCutEdge(int v0, int v1) const {
    CutEdgeType type = getEdgeType(v0, v1);
    return type != CUT_EDGE_NONE;
}

} // namespace MeshCutByMark

#endif // EDGE_INFO_H
