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

        // Build vertex -> edges mapping for this type
        auto vertexToEdges = buildVertexToEdgesMap(cutEdges, edgeIndices);

        // Track which edges have been used
        std::vector<bool> used(cutEdges.size(), false);

        // NON_MANIFOLD: 记录已使用的边（canonical key），跳过反向边
        std::unordered_map<std::pair<int, int>, int, EdgeHash, EdgeEqual> useEdges;

        for (int edgeIndex : edgeIndices)
        {
            if (used[edgeIndex])
            {
                continue;
            }

            // NON_MANIFOLD: 检查 canonical key 是否已使用
            if (type == CUT_EDGE_NON_MANIFOLD)
            {
                std::pair<int, int> edgePair = {cutEdges[edgeIndex].v0, cutEdges[edgeIndex].v1};
                std::pair<int, int> reverseEdgePair = {cutEdges[edgeIndex].v1, cutEdges[edgeIndex].v0};

                if (useEdges.find(reverseEdgePair) != useEdges.end())
                {
                    continue;
                }
                useEdges[edgePair] = edgeIndex;

                for (int innerEdgeIndex = 0; innerEdgeIndex < (int)cutEdges.size(); ++innerEdgeIndex)
                {
                    std::pair<int, int> innerEdgePair = {cutEdges[innerEdgeIndex].v0, cutEdges[innerEdgeIndex].v1};
                    std::pair<int, int> innerReversePair = {cutEdges[innerEdgeIndex].v1, cutEdges[innerEdgeIndex].v0};

                    if (useEdges.find(innerReversePair) != useEdges.end())
                    {
                        used[innerEdgeIndex] = true;
                    }
                    if (useEdges.find(innerEdgePair) != useEdges.end())
                    {
                        used[innerEdgeIndex] = true;
                    }
                }
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
            extendPolyline(polyline, startVertex, cutEdges, vertexToEdges, used, false);

            // Extend towards endVertex direction (append)
            extendPolyline(polyline, endVertex, cutEdges, vertexToEdges, used, true);

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
        std::vector<bool> &used,
        bool forward)
    {
        // Append or prepend vertices while unused edges continue from currentVertex
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

                used[edgeIndex] = true;

                int otherVertex = (cutEdges[edgeIndex].v0 == currentVertex) ? cutEdges[edgeIndex].v1 : cutEdges[edgeIndex].v0;

                if (forward)
                {
                    polyline.vertexIndices.push_back(otherVertex);
                    polyline.endFaceIdx = cutEdges[edgeIndex].faceIdx;
                    polyline.endEdgeIdx = cutEdges[edgeIndex].edgeIdx;
                }
                else
                {
                    polyline.vertexIndices.insert(polyline.vertexIndices.begin(), otherVertex);
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
    }

    inline void PolylineManager::tryMergePolylines(std::vector<Polyline> &polylines)
    {
        // Try to merge polylines with matching endpoints
        bool merged = true;
        while (merged)
        {
            merged = false;
            for (int polylineIndex = 0; polylineIndex < (int)polylines.size(); polylineIndex++)
            {
                if (polylines[polylineIndex].isClosed)
                {
                    continue;
                }

                for (int candidateIndex = polylineIndex + 1; candidateIndex < (int)polylines.size(); candidateIndex++)
                {
                    if (polylines[candidateIndex].isClosed)
                    {
                        continue;
                    }

                    auto &firstPolyline = polylines[polylineIndex];
                    auto &secondPolyline = polylines[candidateIndex];

                    // Check if firstPolyline's end matches secondPolyline's start
                    if (firstPolyline.vertexIndices.back() == secondPolyline.vertexIndices.front())
                    {
                        // Merge secondPolyline into firstPolyline
                        for (int vertexIndex = 1; vertexIndex < (int)secondPolyline.vertexIndices.size(); vertexIndex++)
                        {
                            firstPolyline.vertexIndices.push_back(secondPolyline.vertexIndices[vertexIndex]);
                        }
                        firstPolyline.endFaceIdx = secondPolyline.endFaceIdx;
                        firstPolyline.endEdgeIdx = secondPolyline.endEdgeIdx;

                        // Check if closed
                        if (firstPolyline.vertexIndices.front() == firstPolyline.vertexIndices.back())
                        {
                            firstPolyline.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + candidateIndex);
                        merged = true;
                        break;
                    }

                    // Check if firstPolyline's start matches secondPolyline's end
                    if (firstPolyline.vertexIndices.front() == secondPolyline.vertexIndices.back())
                    {
                        // Merge secondPolyline into firstPolyline (prepend)
                        std::vector<int> mergedVertices;
                        for (int vertexIndex = 0; vertexIndex < (int)secondPolyline.vertexIndices.size() - 1; vertexIndex++)
                        {
                            mergedVertices.push_back(secondPolyline.vertexIndices[vertexIndex]);
                        }
                        mergedVertices.insert(mergedVertices.end(), firstPolyline.vertexIndices.begin(), firstPolyline.vertexIndices.end());
                        firstPolyline.vertexIndices = mergedVertices;
                        firstPolyline.startFaceIdx = secondPolyline.startFaceIdx;
                        firstPolyline.startEdgeIdx = secondPolyline.startEdgeIdx;

                        // Check if closed
                        if (firstPolyline.vertexIndices.front() == firstPolyline.vertexIndices.back())
                        {
                            firstPolyline.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + candidateIndex);
                        merged = true;
                        break;
                    }

                    // Check if firstPolyline's end matches secondPolyline's end (reverse secondPolyline)
                    if (firstPolyline.vertexIndices.back() == secondPolyline.vertexIndices.back())
                    {
                        // Reverse secondPolyline and merge
                        std::vector<int> reversedCandidate(secondPolyline.vertexIndices.rbegin(), secondPolyline.vertexIndices.rend());
                        for (int vertexIndex = 1; vertexIndex < (int)reversedCandidate.size(); vertexIndex++)
                        {
                            firstPolyline.vertexIndices.push_back(reversedCandidate[vertexIndex]);
                        }
                        firstPolyline.endFaceIdx = secondPolyline.startFaceIdx;
                        firstPolyline.endEdgeIdx = secondPolyline.startEdgeIdx;

                        // Check if closed
                        if (firstPolyline.vertexIndices.front() == firstPolyline.vertexIndices.back())
                        {
                            firstPolyline.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + candidateIndex);
                        merged = true;
                        break;
                    }

                    // Check if firstPolyline's start matches secondPolyline's start (reverse secondPolyline)
                    if (firstPolyline.vertexIndices.front() == secondPolyline.vertexIndices.front())
                    {
                        // Reverse secondPolyline and prepend
                        std::vector<int> reversedCandidate(secondPolyline.vertexIndices.rbegin(), secondPolyline.vertexIndices.rend());
                        std::vector<int> mergedVertices;
                        for (int vertexIndex = 0; vertexIndex < (int)reversedCandidate.size() - 1; vertexIndex++)
                        {
                            mergedVertices.push_back(reversedCandidate[vertexIndex]);
                        }
                        mergedVertices.insert(mergedVertices.end(), firstPolyline.vertexIndices.begin(), firstPolyline.vertexIndices.end());
                        firstPolyline.vertexIndices = mergedVertices;
                        firstPolyline.startFaceIdx = secondPolyline.endFaceIdx;
                        firstPolyline.startEdgeIdx = secondPolyline.endEdgeIdx;

                        // Check if closed
                        if (firstPolyline.vertexIndices.front() == firstPolyline.vertexIndices.back())
                        {
                            firstPolyline.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + candidateIndex);
                        merged = true;
                        break;
                    }
                }
                if (merged)
                {
                    break;
                }
            }
        }
    }

} // namespace MeshCutByMark

#endif // POLYLINE_H
