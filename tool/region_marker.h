// tool/region_marker.h
#ifndef REGION_MARKER_H
#define REGION_MARKER_H

#include <algorithm>
#include <vector>
#include <queue>
#include "edge_info.h"

namespace MeshCutByMark
{

    // Region marker manager: flood-fill for connected regions and marking new regions
    class RegionMarker
    {
    public:
        // Initialize newMark storage for all faces
        void initNewMark(CMeshOD *mesh);

        // Flood-fill to find connected region starting from startFaceIdx
        // Only crosses edges that are not cut edges and faces with the same targetMark
        std::vector<int> floodFill(
            int startFaceIdx,
            int targetMark,
            CMeshOD *mesh,
            const EdgeInfoManager &edgeInfo
        );

        // Extract sub-regions from curFaces after cutting
        // Returns connected components separated by cut edges
        std::vector<std::vector<int>> extractSubRegions(
            const std::vector<int> &curFaces,
            CMeshOD *mesh
        );

        // Mark sub-regions with incrementing new mark values
        void markSubRegions(
            const std::vector<std::vector<int>> &subRegions,
            CMeshOD *mesh,
            int &newMarkCounter
        );

        // Get newMark value for a specific face
        int getNewMark(int faceIdx) const;

        // Set newMark value for a specific face
        void setNewMark(int faceIdx, int value);

        // 扩展 m_newMark 到 newSize，新增元素置 0（不重置已有）
        void growNewMark(size_t newSize);

    private:
        // Check if an edge is a cut edge (boundary edge with no valid FF adjacency neighbor)
        bool isCutEdge(int faceIdx, int edgeIdx, CMeshOD *mesh);

        // Per-face newMark storage (parallel to mesh->face)
        std::vector<int> m_newMark;

        // 可复用访问标记：floodFill 每次调用递增 token，避免分配并清零全局面位图。
        std::vector<int> m_visitMark;
        int m_visitToken = 0;
    };

    // --- Implementations ---

    inline void RegionMarker::initNewMark(CMeshOD *mesh)
    {
        // Reset the per-face mark storage to all zeros
        m_newMark.assign(mesh->face.size(), 0);
        m_visitMark.assign(mesh->face.size(), 0);
        m_visitToken = 0;
    }

    inline int RegionMarker::getNewMark(int faceIdx) const
    {
        // Return the new mark of a face, or 0 when the index is out of range
        if (faceIdx >= 0 && faceIdx < (int)m_newMark.size())
        {
            return m_newMark[faceIdx];
        }
        return 0;
    }

    inline void RegionMarker::setNewMark(int faceIdx, int value)
    {
        // Store a new mark for a face when the index is valid
        if (faceIdx >= 0 && faceIdx < (int)m_newMark.size())
        {
            m_newMark[faceIdx] = value;
        }
    }

    inline void RegionMarker::growNewMark(size_t newSize)
    {
        // Grow the storage with zero-initialized entries when needed
        if (newSize > m_newMark.size())
        {
            m_newMark.resize(newSize, 0);
        }
    }

    inline bool RegionMarker::isCutEdge(int faceIdx, int edgeIdx, CMeshOD *mesh)
    {
        // Check if FF adjacency is enabled
        if (!mesh->face.IsFFAdjacencyEnabled())
        {
            return false;
        }

        // Get the adjacent face across this edge
        CFaceOD *adjacentFace = mesh->face[faceIdx].FFp(edgeIdx);
        if (adjacentFace == nullptr)
        {
            return true; // boundary edge -> cut edge
        }

        int adjacentFaceIndex = static_cast<int>(adjacentFace - &mesh->face[0]);
        if (adjacentFaceIndex < 0 || adjacentFaceIndex == faceIdx)
        {
            return true; // boundary or self-referencing -> cut edge
        }

        return false;
    }

    inline std::vector<int> RegionMarker::floodFill(
        int startFaceIdx,
        int targetMark,
        CMeshOD *mesh,
        const EdgeInfoManager & edgeInfo
    )
    {
        // Collect the connected region around startFaceIdx crossing only same-mark, non-cut edges
        std::vector<int> result;
        std::queue<int> queue;

        if (m_visitMark.size() < mesh->face.size())
        {
            m_visitMark.resize(mesh->face.size(), 0);
        }
        ++m_visitToken;
        if (m_visitToken == 0)
        {
            std::fill(m_visitMark.begin(), m_visitMark.end(), 0);
            ++m_visitToken;
        }
        const int currentToken = m_visitToken;

        queue.push(startFaceIdx);
        m_visitMark[startFaceIdx] = currentToken;

        while (!queue.empty())
        {
            int faceIndex = queue.front();
            queue.pop();
            result.push_back(faceIndex);

            // Traverse the three edges of this face
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                // Skip cut edges classified by the edge-info manager, including
                // mark-diff, boundary, and non-manifold edges.
                int vertex0 = mesh->face[faceIndex].V(edgeIndex)->Index();
                int vertex1 = mesh->face[faceIndex].V((edgeIndex + 1) % 3)->Index();
                if (edgeInfo.isCutEdge(vertex0, vertex1))
                {
                    continue;
                }

                // Get adjacent face index
                int adjacentFaceIndex = static_cast<int>(
                    mesh->face[faceIndex].FFp(edgeIndex) - &mesh->face[0]
                );

                // Check if adjacent face is valid and not visited
                if (adjacentFaceIndex < 0 ||
                    m_visitMark[adjacentFaceIndex] == currentToken)
                {
                    continue;
                }

                // Check if the adjacent face has the same mark
                if (mesh->face[adjacentFaceIndex].IMark() != targetMark)
                {
                    continue;
                }

                m_visitMark[adjacentFaceIndex] = currentToken;
                queue.push(adjacentFaceIndex);
            }
        }

        return result;
    }

    inline std::vector<std::vector<int>> RegionMarker::extractSubRegions(
        const std::vector<int> &curFaces,
        CMeshOD *mesh
    )
    {
        // Split curFaces into connected components separated by cut edges
        std::vector<std::vector<int>> subRegions;

        // Build a set for fast lookup of faces in curFaces
        std::vector<bool> inCurFaces(mesh->face.size(), false);
        for (int faceIndex : curFaces)
        {
            inCurFaces[faceIndex] = true;
        }

        for (int faceIndex : curFaces)
        {
            if (m_newMark[faceIndex] != 0)
            {
                continue; // already marked (positive = assigned, negative = in progress)
            }

            // Flood-fill from this face
            std::vector<int> region;
            std::queue<int> queue;
            queue.push(faceIndex);
            m_newMark[faceIndex] = -1; // temporary mark: being processed

            while (!queue.empty())
            {
                int currentFace = queue.front();
                queue.pop();
                region.push_back(currentFace);

                // Traverse the three edges
                for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
                {
                    // Skip cut edges
                    if (isCutEdge(currentFace, edgeIndex, mesh))
                    {
                        continue;
                    }

                    // Get adjacent face index
                    int adjacentFaceIndex = static_cast<int>(
                        mesh->face[currentFace].FFp(edgeIndex) - &mesh->face[0]
                    );

                    if (adjacentFaceIndex < 0)
                    {
                        continue;
                    }

                    // Check if already processed or in progress
                    if (m_newMark[adjacentFaceIndex] != 0)
                    {
                        continue;
                    }

                    // Check if still in curFaces
                    if (!inCurFaces[adjacentFaceIndex])
                    {
                        continue;
                    }

                    m_newMark[adjacentFaceIndex] = -1; // temporary mark
                    queue.push(adjacentFaceIndex);
                }
            }

            subRegions.push_back(region);
        }

        return subRegions;
    }

    inline void RegionMarker::markSubRegions(
        const std::vector<std::vector<int>> &subRegions,
        CMeshOD * /*mesh*/,
        int &newMarkCounter
    )
    {
        // Assign one incrementing new mark value to every face of each sub-region
        for (const auto &region : subRegions)
        {
            for (int faceIndex : region)
            {
                m_newMark[faceIndex] = newMarkCounter;
            }
            newMarkCounter++;
        }
    }

} // namespace MeshCutByMark

#endif // REGION_MARKER_H
