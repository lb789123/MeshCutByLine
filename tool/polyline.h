// tool/polyline.h
#ifndef POLYLINE_H
#define POLYLINE_H

#include <vector>
#include <unordered_map>
#include "edge_info.h"

namespace MeshCutByMark
{

    // Polyline: a connected sequence of vertices formed by linking cut edges
    struct Polyline
    {
        std::vector<int> vertexIndices;   // vertex sequence of the polyline
        int startFaceIdx;                 // face index at the start endpoint
        int startEdgeIdx;                 // edge index within that face at the start
        int endFaceIdx;                   // face index at the end endpoint
        int endEdgeIdx;                   // edge index within that face at the end
        bool isClosed = false;            // whether this polyline forms a closed loop
        CutEdgeType type = CUT_EDGE_NONE; // dominant edge type of this polyline
    };

    // PolylineManager: connects scattered cut edges into continuous polylines
    class PolylineManager
    {
    public:
        // Connect cut edges into continuous polylines (type-aware)
        std::vector<Polyline> connectEdgesToPolylines(
            const std::vector<CutEdge> &cutEdges,
            CMeshOD *mesh);

    private:
        // Build vertex -> list-of-edge-indices mapping (filtered by type)
        std::unordered_map<int, std::vector<int>> buildVertexToEdgesMap(
            const std::vector<CutEdge> &cutEdges,
            const std::vector<int> &edgeIndices);

        // Extend a polyline in one direction from currentVertex
        void extendPolyline(
            Polyline &polyline,
            int &currentVertex,
            const std::vector<CutEdge> &cutEdges,
            const std::unordered_map<int, std::vector<int>> &vertexToEdges,
            const std::unordered_map<std::pair<int, int>, std::vector<int>, EdgeHash, EdgeEqual> &canonicalEdgeGroups,
            std::vector<bool> &used,
            bool forward);

        // Connect edges of a specific type into polylines
        std::vector<Polyline> connectByType(
            const std::vector<CutEdge> &cutEdges,
            const std::vector<int> &edgeIndices,
            CutEdgeType type);

        // Try to merge polylines with matching endpoints
        void tryMergePolylines(std::vector<Polyline> &polylines);
    };

    // --- Implementations ---

    inline std::vector<Polyline> PolylineManager::connectEdgesToPolylines(
        const std::vector<CutEdge> &cutEdges,
        CMeshOD *mesh)
    {
        // Collect all edges grouped by type, connect same-type edges, then merge compatible polylines
        std::vector<Polyline> result;

        if (cutEdges.empty())
        {
            return result;
        }

        // Phase 1: Separate edges by type
        std::vector<int> markDiffEdges, boundaryEdges, nonManifoldEdges;
        for (int edgeIndex = 0; edgeIndex < (int)cutEdges.size(); edgeIndex++)
        {
            switch (cutEdges[edgeIndex].type)
            {
            case CUT_EDGE_MARK_DIFF:
                markDiffEdges.push_back(edgeIndex);
                break;
            case CUT_EDGE_BOUNDARY:
                boundaryEdges.push_back(edgeIndex);
                break;
            case CUT_EDGE_NON_MANIFOLD:
                nonManifoldEdges.push_back(edgeIndex);
                break;
            default:
                break;
            }
        }

        // Phase 2: Connect same-type edges into polylines
        auto markDiffPolylines = connectByType(cutEdges, markDiffEdges, CUT_EDGE_MARK_DIFF);
        auto boundaryPolylines = connectByType(cutEdges, boundaryEdges, CUT_EDGE_BOUNDARY);
        auto nonManifoldPolylines = connectByType(cutEdges, nonManifoldEdges, CUT_EDGE_NON_MANIFOLD);

        // Phase 4: Try to merge MARK_DIFF + BOUNDARY polylines
        std::vector<Polyline> mergedPolylines;
        mergedPolylines.insert(mergedPolylines.end(), markDiffPolylines.begin(), markDiffPolylines.end());
        mergedPolylines.insert(mergedPolylines.end(), boundaryPolylines.begin(), boundaryPolylines.end());
        tryMergePolylines(mergedPolylines);

        // Phase 5: Combine all polylines
        result.insert(result.end(), mergedPolylines.begin(), mergedPolylines.end());
        result.insert(result.end(), nonManifoldPolylines.begin(), nonManifoldPolylines.end());

        // Phase 6: Check if polylines are closed
        for (auto &polyline : result)
        {
            if (polyline.vertexIndices.size() >= 3 &&
                polyline.vertexIndices.front() == polyline.vertexIndices.back())
            {
                polyline.isClosed = true;
            }
        }

        return result;
    }

    inline std::vector<Polyline> PolylineManager::connectByType(
        const std::vector<CutEdge> &cutEdges,
        const std::vector<int> &edgeIndices,
        CutEdgeType type)
    {
        // Build polylines from edges of a single type, skipping already-used edges
        std::vector<Polyline> polylines;

        if (edgeIndices.empty())
        {
            return polylines;
        }

        // Track which edges have been used
        std::vector<bool> used(cutEdges.size(), false);

        // 预扫描：同一几何边会被相邻面各记录一次（方向相反），
        // 只保留第一条，其余重复记录提前标记为已使用；
        // 同时按几何边分组记录所有索引，供扩展时一并标记反向/重复边
        std::unordered_map<std::pair<int, int>, int, EdgeHash, EdgeEqual> firstEdgeByKey;
        std::unordered_map<std::pair<int, int>, std::vector<int>, EdgeHash, EdgeEqual> canonicalEdgeGroups;
        for (int edgeIndex : edgeIndices)
        {
            std::pair<int, int> edgePair = {cutEdges[edgeIndex].v0, cutEdges[edgeIndex].v1};
            canonicalEdgeGroups[edgePair].push_back(edgeIndex);
            auto foundEntry = firstEdgeByKey.find(edgePair);
            if (foundEntry == firstEdgeByKey.end())
            {
                firstEdgeByKey[edgePair] = edgeIndex;
            }
            else
            {
                used[edgeIndex] = true;
            }
        }

        // 邻接表只包含保留的边，反向/重复记录不再参与连接
        std::vector<int> keptEdges;
        for (int edgeIndex : edgeIndices)
        {
            if (!used[edgeIndex])
            {
                keptEdges.push_back(edgeIndex);
            }
        }
        auto vertexToEdges = buildVertexToEdgesMap(cutEdges, keptEdges);

        for (int edgeIndex : edgeIndices)
        {
            if (used[edgeIndex])
            {
                continue;
            }

            Polyline polyline;
            polyline.type = type;
            used[edgeIndex] = true;

            // Start from this edge
            int startVertex = cutEdges[edgeIndex].v0;
            int endVertex = cutEdges[edgeIndex].v1;
            polyline.vertexIndices.push_back(startVertex);
            polyline.vertexIndices.push_back(endVertex);

            // Record endpoint face/edge info
            polyline.startFaceIdx = cutEdges[edgeIndex].faceIdx;
            polyline.startEdgeIdx = cutEdges[edgeIndex].edgeIdx;
            polyline.endFaceIdx = cutEdges[edgeIndex].faceIdx;
            polyline.endEdgeIdx = cutEdges[edgeIndex].edgeIdx;

            // Extend towards startVertex direction (prepend)
            extendPolyline(polyline, startVertex, cutEdges, vertexToEdges, canonicalEdgeGroups, used, false);

            // Extend towards endVertex direction (append)
            extendPolyline(polyline, endVertex, cutEdges, vertexToEdges, canonicalEdgeGroups, used, true);

            polylines.push_back(polyline);
        }

        return polylines;
    }

    inline std::unordered_map<int, std::vector<int>> PolylineManager::buildVertexToEdgesMap(
        const std::vector<CutEdge> &cutEdges,
        const std::vector<int> &edgeIndices)
    {
        // Map every endpoint vertex to the list of incident edges
        std::unordered_map<int, std::vector<int>> vertexToEdges;

        for (int edgeIndex : edgeIndices)
        {
            vertexToEdges[cutEdges[edgeIndex].v0].push_back(edgeIndex);
            vertexToEdges[cutEdges[edgeIndex].v1].push_back(edgeIndex);
        }

        return vertexToEdges;
    }

    inline void PolylineManager::extendPolyline(
        Polyline &polyline,
        int &currentVertex,
        const std::vector<CutEdge> &cutEdges,
        const std::unordered_map<int, std::vector<int>> &vertexToEdges,
        const std::unordered_map<std::pair<int, int>, std::vector<int>, EdgeHash, EdgeEqual> &canonicalEdgeGroups,
        std::vector<bool> &used,
        bool forward)
    {
        // Append or prepend vertices while unused edges continue from currentVertex.
        // 反向扩展先收集前缀，再一次性写回，避免 vector 前端插入的 O(n²) 成本。
        std::vector<int> prependedVertices;
        while (true)
        {
            bool foundNext = false;

            auto edgesIt = vertexToEdges.find(currentVertex);
            if (edgesIt == vertexToEdges.end())
            {
                break;
            }

            for (int edgeIndex : edgesIt->second)
            {
                if (used[edgeIndex])
                {
                    continue;
                }

                // 链接当前边时，把同一条几何边的反向/重复记录一并标记，防止折返
                used[edgeIndex] = true;
                std::pair<int, int> edgePair = {cutEdges[edgeIndex].v0, cutEdges[edgeIndex].v1};
                auto groupIt = canonicalEdgeGroups.find(edgePair);
                if (groupIt != canonicalEdgeGroups.end())
                {
                    for (int groupEdgeIndex : groupIt->second)
                    {
                        used[groupEdgeIndex] = true;
                    }
                }

                int otherVertex = (cutEdges[edgeIndex].v0 == currentVertex) ? cutEdges[edgeIndex].v1 : cutEdges[edgeIndex].v0;

                if (forward)
                {
                    polyline.vertexIndices.push_back(otherVertex);
                    polyline.endFaceIdx = cutEdges[edgeIndex].faceIdx;
                    polyline.endEdgeIdx = cutEdges[edgeIndex].edgeIdx;
                }
                else
                {
                    prependedVertices.push_back(otherVertex);
                    polyline.startFaceIdx = cutEdges[edgeIndex].faceIdx;
                    polyline.startEdgeIdx = cutEdges[edgeIndex].edgeIdx;
                }

                currentVertex = otherVertex;
                foundNext = true;
                break;
            }

            if (!foundNext)
            {
                break;
            }
        }

        if (!prependedVertices.empty())
        {
            std::vector<int> mergedVertices;
            mergedVertices.reserve(prependedVertices.size() +
                polyline.vertexIndices.size());
            mergedVertices.insert(mergedVertices.end(),
                prependedVertices.rbegin(), prependedVertices.rend());
            mergedVertices.insert(mergedVertices.end(),
                polyline.vertexIndices.begin(), polyline.vertexIndices.end());
            polyline.vertexIndices = std::move(mergedVertices);
        }
    }

    inline void PolylineManager::tryMergePolylines(std::vector<Polyline> &polylines)
    {
        // 按「端点 -> 折线」索引逐轮批量合并，避免反复从头扫描与向量 erase
        // 导致的 O(n³) 退化。每轮结束后统一压缩已删除折线。
        bool merged = true;
        while (merged)
        {
            merged = false;

            std::map<int, std::vector<int>> startMap;
            std::map<int, std::vector<int>> endMap;
            for (int polylineIndex = 0;
                 polylineIndex < (int)polylines.size(); ++polylineIndex)
            {
                const Polyline &polyline = polylines[polylineIndex];
                if (polyline.isClosed)
                {
                    continue;
                }
                startMap[polyline.vertexIndices.front()].push_back(
                    polylineIndex);
                endMap[polyline.vertexIndices.back()].push_back(
                    polylineIndex);
            }

            std::vector<char> removed(polylines.size(), 0);

            for (int polylineIndex = 0;
                 polylineIndex < (int)polylines.size(); ++polylineIndex)
            {
                if (removed[polylineIndex] || polylines[polylineIndex].isClosed)
                {
                    continue;
                }

                const int firstStart = polylines[polylineIndex].vertexIndices.front();
                const int firstEnd = polylines[polylineIndex].vertexIndices.back();
                int candidateIndex = -1;
                int mergeCase = -1;

                auto pickCandidateByStart =
                    [&](const std::vector<int> &indices, int expectedStart)
                {
                    for (int candidate : indices)
                    {
                        if (candidate == polylineIndex || removed[candidate] ||
                            polylines[candidate].isClosed)
                        {
                            continue;
                        }
                        if (polylines[candidate].vertexIndices.front() ==
                            expectedStart)
                        {
                            return candidate;
                        }
                    }
                    return -1;
                };

                auto pickCandidateByEnd =
                    [&](const std::vector<int> &indices, int expectedEnd)
                {
                    for (int candidate : indices)
                    {
                        if (candidate == polylineIndex || removed[candidate] ||
                            polylines[candidate].isClosed)
                        {
                            continue;
                        }
                        if (polylines[candidate].vertexIndices.back() ==
                            expectedEnd)
                        {
                            return candidate;
                        }
                    }
                    return -1;
                };

                candidateIndex = pickCandidateByStart(startMap[firstEnd], firstEnd);
                if (candidateIndex >= 0)
                {
                    mergeCase = 0;  // first.end -> second.start
                }
                if (candidateIndex < 0)
                {
                    candidateIndex = pickCandidateByEnd(endMap[firstStart], firstStart);
                    if (candidateIndex >= 0)
                    {
                        mergeCase = 1;  // first.start <- second.end
                    }
                }
                if (candidateIndex < 0)
                {
                    candidateIndex = pickCandidateByEnd(endMap[firstEnd], firstEnd);
                    if (candidateIndex >= 0)
                    {
                        mergeCase = 2;  // first.end <- reversed second.end
                    }
                }
                if (candidateIndex < 0)
                {
                    candidateIndex = pickCandidateByStart(startMap[firstStart], firstStart);
                    if (candidateIndex >= 0)
                    {
                        mergeCase = 3;  // first.start <- reversed second.start
                    }
                }

                if (candidateIndex < 0)
                {
                    continue;
                }

                Polyline &firstPolyline = polylines[polylineIndex];
                Polyline &secondPolyline = polylines[candidateIndex];

                if (mergeCase == 0)
                {
                    for (int vertexIndex = 1;
                         vertexIndex < (int)secondPolyline.vertexIndices.size();
                         ++vertexIndex)
                    {
                        firstPolyline.vertexIndices.push_back(
                            secondPolyline.vertexIndices[vertexIndex]);
                    }
                    firstPolyline.endFaceIdx = secondPolyline.endFaceIdx;
                    firstPolyline.endEdgeIdx = secondPolyline.endEdgeIdx;
                }
                else if (mergeCase == 1)
                {
                    std::vector<int> mergedVertices;
                    mergedVertices.reserve(secondPolyline.vertexIndices.size() +
                        firstPolyline.vertexIndices.size() - 1);
                    mergedVertices.insert(mergedVertices.end(),
                        secondPolyline.vertexIndices.begin(),
                        secondPolyline.vertexIndices.end() - 1);
                    mergedVertices.insert(mergedVertices.end(),
                        firstPolyline.vertexIndices.begin(),
                        firstPolyline.vertexIndices.end());
                    firstPolyline.vertexIndices = std::move(mergedVertices);
                    firstPolyline.startFaceIdx = secondPolyline.startFaceIdx;
                    firstPolyline.startEdgeIdx = secondPolyline.startEdgeIdx;
                }
                else if (mergeCase == 2)
                {
                    std::vector<int> reversedCandidate(
                        secondPolyline.vertexIndices.rbegin(),
                        secondPolyline.vertexIndices.rend());
                    for (int vertexIndex = 1;
                         vertexIndex < (int)reversedCandidate.size();
                         ++vertexIndex)
                    {
                        firstPolyline.vertexIndices.push_back(
                            reversedCandidate[vertexIndex]);
                    }
                    firstPolyline.endFaceIdx = secondPolyline.startFaceIdx;
                    firstPolyline.endEdgeIdx = secondPolyline.startEdgeIdx;
                }
                else
                {
                    std::vector<int> reversedCandidate(
                        secondPolyline.vertexIndices.rbegin(),
                        secondPolyline.vertexIndices.rend());
                    std::vector<int> mergedVertices;
                    mergedVertices.reserve(reversedCandidate.size() +
                        firstPolyline.vertexIndices.size() - 1);
                    mergedVertices.insert(mergedVertices.end(),
                        reversedCandidate.begin(), reversedCandidate.end() - 1);
                    mergedVertices.insert(mergedVertices.end(),
                        firstPolyline.vertexIndices.begin(),
                        firstPolyline.vertexIndices.end());
                    firstPolyline.vertexIndices = std::move(mergedVertices);
                    firstPolyline.startFaceIdx = secondPolyline.endFaceIdx;
                    firstPolyline.startEdgeIdx = secondPolyline.endEdgeIdx;
                }

                if (firstPolyline.vertexIndices.front() ==
                    firstPolyline.vertexIndices.back())
                {
                    firstPolyline.isClosed = true;
                }

                removed[candidateIndex] = 1;
                merged = true;
            }

            if (!merged)
            {
                break;
            }

            std::vector<Polyline> compacted;
            compacted.reserve(polylines.size());
            for (int polylineIndex = 0;
                 polylineIndex < (int)polylines.size(); ++polylineIndex)
            {
                if (!removed[polylineIndex])
                {
                    compacted.push_back(std::move(polylines[polylineIndex]));
                }
            }
            polylines = std::move(compacted);
        }
    }

} // namespace MeshCutByMark

#endif // POLYLINE_H
