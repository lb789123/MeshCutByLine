// tool/region_marker.h
#ifndef REGION_MARKER_H
#define REGION_MARKER_H

#include <vector>
#include <queue>
#include "edge_info.h"

namespace MeshCutByMark {

// Region marker manager: flood-fill for connected regions and marking new regions
class RegionMarker {
public:
    // Initialize newMark storage for all faces
    void initNewMark(CMeshO* mesh);

    // Flood-fill to find connected region starting from startFaceIdx
    // Only crosses edges that are not cut edges and faces with the same targetMark
    std::vector<int> floodFill(
        int startFaceIdx,
        int targetMark,
        CMeshO* mesh,
        const EdgeInfoManager& edgeInfo
    );

    // Extract sub-regions from curFaces after cutting
    // Returns connected components separated by cut edges
    std::vector<std::vector<int>> extractSubRegions(
        const std::vector<int>& curFaces,
        CMeshO* mesh
    );

    // Mark sub-regions with incrementing new mark values
    void markSubRegions(
        const std::vector<std::vector<int>>& subRegions,
        CMeshO* mesh,
        int& newMarkCounter
    );

    // Get newMark value for a specific face
    int getNewMark(int faceIdx) const;

    // Set newMark value for a specific face
    void setNewMark(int faceIdx, int value);

private:
    // Check if an edge is a cut edge (boundary edge with no valid FF adjacency neighbor)
    bool isCutEdge(int faceIdx, int edgeIdx, CMeshO* mesh);

    // Per-face newMark storage (parallel to mesh->face)
    std::vector<int> m_newMark;
};

// --- Implementations ---

inline void RegionMarker::initNewMark(CMeshO* mesh) {
    m_newMark.assign(mesh->face.size(), 0);
}

inline int RegionMarker::getNewMark(int faceIdx) const {
    if (faceIdx >= 0 && faceIdx < (int)m_newMark.size())
        return m_newMark[faceIdx];
    return 0;
}

inline void RegionMarker::setNewMark(int faceIdx, int value) {
    if (faceIdx >= 0 && faceIdx < (int)m_newMark.size())
        m_newMark[faceIdx] = value;
}

inline bool RegionMarker::isCutEdge(int faceIdx, int edgeIdx, CMeshO* mesh) {
    // Check if FF adjacency is enabled
    if (!mesh->face.IsFFAdjacencyEnabled()) {
        return false;
    }

    // Get the adjacent face across this edge
    CFaceO* adjFace = mesh->face[faceIdx].FFp(edgeIdx);
    if (adjFace == nullptr) {
        return true;  // boundary edge -> cut edge
    }

    int adjFaceIdx = static_cast<int>(adjFace - &mesh->face[0]);
    if (adjFaceIdx < 0 || adjFaceIdx == faceIdx) {
        return true;  // boundary or self-referencing -> cut edge
    }

    return false;
}

inline std::vector<int> RegionMarker::floodFill(
    int startFaceIdx,
    int targetMark,
    CMeshO* mesh,
    const EdgeInfoManager& /*edgeInfo*/
) {
    std::vector<int> result;
    std::queue<int> queue;

    std::vector<bool> visited(mesh->face.size(), false);
    queue.push(startFaceIdx);
    visited[startFaceIdx] = true;

    while (!queue.empty()) {
        int faceIdx = queue.front();
        queue.pop();
        result.push_back(faceIdx);

        // Traverse the three edges of this face
        for (int j = 0; j < 3; j++) {
            // Skip cut edges (boundary edges)
            if (isCutEdge(faceIdx, j, mesh))
                continue;

            // Get adjacent face index
            int adjFaceIdx = static_cast<int>(
                mesh->face[faceIdx].FFp(j) - &mesh->face[0]
            );

            // Check if adjacent face is valid and not visited
            if (adjFaceIdx < 0 || visited[adjFaceIdx])
                continue;

            // Check if the adjacent face has the same mark
            if (mesh->face[adjFaceIdx].IMark() != targetMark)
                continue;

            visited[adjFaceIdx] = true;
            queue.push(adjFaceIdx);
        }
    }

    return result;
}

inline std::vector<std::vector<int>> RegionMarker::extractSubRegions(
    const std::vector<int>& curFaces,
    CMeshO* mesh
) {
    std::vector<std::vector<int>> subRegions;

    // Build a set for fast lookup of faces in curFaces
    std::vector<bool> inCurFaces(mesh->face.size(), false);
    for (int faceIdx : curFaces) {
        inCurFaces[faceIdx] = true;
    }

    for (int faceIdx : curFaces) {
        if (m_newMark[faceIdx] != 0)
            continue;  // already marked (positive = assigned, negative = in progress)

        // Flood-fill from this face
        std::vector<int> region;
        std::queue<int> queue;
        queue.push(faceIdx);
        m_newMark[faceIdx] = -1;  // temporary mark: being processed

        while (!queue.empty()) {
            int curFace = queue.front();
            queue.pop();
            region.push_back(curFace);

            // Traverse the three edges
            for (int j = 0; j < 3; j++) {
                // Skip cut edges
                if (isCutEdge(curFace, j, mesh))
                    continue;

                // Get adjacent face index
                int adjFaceIdx = static_cast<int>(
                    mesh->face[curFace].FFp(j) - &mesh->face[0]
                );

                if (adjFaceIdx < 0)
                    continue;

                // Check if already processed or in progress
                if (m_newMark[adjFaceIdx] != 0)
                    continue;

                // Check if still in curFaces
                if (!inCurFaces[adjFaceIdx])
                    continue;

                m_newMark[adjFaceIdx] = -1;  // temporary mark
                queue.push(adjFaceIdx);
            }
        }

        subRegions.push_back(region);
    }

    return subRegions;
}

inline void RegionMarker::markSubRegions(
    const std::vector<std::vector<int>>& subRegions,
    CMeshO* /*mesh*/,
    int& newMarkCounter
) {
    for (const auto& region : subRegions) {
        for (int faceIdx : region) {
            m_newMark[faceIdx] = newMarkCounter;
        }
        newMarkCounter++;
    }
}

} // namespace MeshCutByMark

#endif // REGION_MARKER_H
