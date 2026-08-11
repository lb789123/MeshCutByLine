// tool/edge_info.h
#ifndef EDGE_INFO_H
#define EDGE_INFO_H

#include <vector>
#include <unordered_map>
#include <utility>
#include <algorithm>

#include "cmesh.h"

namespace MeshCutByMark
{

// 切割边类型
enum CutEdgeType
{
    CUT_EDGE_NONE,           // 普通边（非切割边）
    CUT_EDGE_MARK_DIFF,      // 相邻三角形 mark 不同
    CUT_EDGE_NON_MANIFOLD,   // 被 3+ 三角形共享
    CUT_EDGE_BOUNDARY        // 只被 1 个三角形使用（孔洞边缘）
};

// 切割边信息
struct CutEdge
{
    int v0, v1;              // 端点顶点索引 (v0 < v1)
    int faceIdx;             // 所属面索引
    int edgeIdx;             // 边在面中的索引 (0, 1, 2)
    CutEdgeType type;        // 切割边类型
};

// 边的哈希函数
struct EdgeHash
{
    size_t operator()(const std::pair<int,int>& edge) const
    {
        int minVertex = std::min(edge.first, edge.second);
        int maxVertex = std::max(edge.first, edge.second);
        return std::hash<int>()(minVertex) ^ (std::hash<int>()(maxVertex) << 1);
    }
};

// 边的相等比较函数（归一化后比较，确保 {vertexA,vertexB} == {vertexB,vertexA}）
struct EdgeEqual
{
    bool operator()(const std::pair<int,int>& vertexA, const std::pair<int,int>& vertexB) const
    {
        auto normalizedA = std::minmax(vertexA.first, vertexA.second);
        auto normalizedB = std::minmax(vertexB.first, vertexB.second);
        return normalizedA == normalizedB;
    }
};

// 边信息管理器
class EdgeInfoManager
{
public:
    // 构建边信息
    void buildEdgeInfo(CMeshOD* mesh);

    // 获取边的类型
    CutEdgeType getEdgeType(int vertex0, int vertex1) const;

    // 获取边的所有邻接面
    std::vector<int> getAdjacentFaces(int vertex0, int vertex1) const;

    // 检查边是否是切割边
    bool isCutEdge(int vertex0, int vertex1) const;

private:
    CMeshOD* m_mesh;
    std::unordered_map<std::pair<int,int>, std::vector<int>, EdgeHash, EdgeEqual> m_edgeToFaces;
    std::unordered_map<std::pair<int,int>, CutEdgeType, EdgeHash, EdgeEqual> m_edgeTypes;
};

// --- Method implementations ---

// 构建边信息：统计每条边的邻接面并分类切割边
inline void EdgeInfoManager::buildEdgeInfo(CMeshOD* mesh)
{
    m_mesh = mesh;
    m_edgeToFaces.clear();
    m_edgeTypes.clear();

    // Traverse all triangles, build edge->face mapping
    for (int faceIndex = 0; faceIndex < (int)mesh->face.size(); faceIndex++)
    {
        if (mesh->face[faceIndex].IsD())
        {
            continue; // skip deleted faces
        }

        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++)
        {
            int vertex0 = mesh->face[faceIndex].V(cornerIndex)->Index();
            int vertex1 = mesh->face[faceIndex].V((cornerIndex + 1) % 3)->Index();

            // Normalize so vertex0 < vertex1
            if (vertex0 > vertex1)
            {
                std::swap(vertex0, vertex1);
            }

            m_edgeToFaces[{vertex0, vertex1}].push_back(faceIndex);
        }
    }

    // Classify each edge
    for (const auto& entry : m_edgeToFaces)
    {
        const auto& edge = entry.first;
        const auto& faces = entry.second;
        CutEdgeType type = CUT_EDGE_NONE;

        if (faces.size() == 1)
        {
            type = CUT_EDGE_BOUNDARY;
        }
        else if (faces.size() >= 3)
        {
            type = CUT_EDGE_NON_MANIFOLD;
        }
        else if (faces.size() == 2)
        {
            // Check if the two faces have different marks
            int mark0 = mesh->face[faces[0]].IMark();
            int mark1 = mesh->face[faces[1]].IMark();
            if (mark0 != mark1)
            {
                type = CUT_EDGE_MARK_DIFF;
            }
        }

        m_edgeTypes[edge] = type;
    }
}

// 获取边（归一化后）的类型
inline CutEdgeType EdgeInfoManager::getEdgeType(int vertex0, int vertex1) const
{
    auto key = std::minmax(vertex0, vertex1);
    auto typeEntry = m_edgeTypes.find(key);
    if (typeEntry != m_edgeTypes.end())
    {
        return typeEntry->second;
    }
    return CUT_EDGE_NONE;
}

// 获取边的所有邻接面
inline std::vector<int> EdgeInfoManager::getAdjacentFaces(int vertex0, int vertex1) const
{
    auto key = std::minmax(vertex0, vertex1);
    auto facesEntry = m_edgeToFaces.find(key);
    if (facesEntry != m_edgeToFaces.end())
    {
        return facesEntry->second;
    }
    return {};
}

// 判断边是否为切割边
inline bool EdgeInfoManager::isCutEdge(int vertex0, int vertex1) const
{
    CutEdgeType type = getEdgeType(vertex0, vertex1);
    return type != CUT_EDGE_NONE;
}

} // namespace MeshCutByMark

#endif // EDGE_INFO_H
