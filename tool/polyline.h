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
        std::vector<Polyline> result;

        if (cutEdges.empty())
            return result;

        // Phase 1: Separate edges by type
        std::vector<int> markDiffEdges, boundaryEdges, nonManifoldEdges;
        for (int i = 0; i < (int)cutEdges.size(); i++)
        {
            switch (cutEdges[i].type)
            {
            case CUT_EDGE_MARK_DIFF:
                markDiffEdges.push_back(i);
                break;
            case CUT_EDGE_BOUNDARY:
                boundaryEdges.push_back(i);
                break;
            case CUT_EDGE_NON_MANIFOLD:
                nonManifoldEdges.push_back(i);
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
        std::vector<Polyline> polylines;

        if (edgeIndices.empty())
            return polylines;

        // Build vertex -> edges mapping for this type
        auto vertexToEdges = buildVertexToEdgesMap(cutEdges, edgeIndices);

        // Track which edges have been used
        std::vector<bool> used(cutEdges.size(), false);

        // NON_MANIFOLD: 记录已使用的边（canonical key），跳过反向边
        std::unordered_map<std::pair<int, int>, int, EdgeHash, EdgeEqual> useEdges;

        for (int idx : edgeIndices)
        {
            if (used[idx])
                continue;

            // NON_MANIFOLD: 检查 canonical key 是否已使用
            if (type == CUT_EDGE_NON_MANIFOLD)
            {

                std::pair<int, int> e = {cutEdges[idx].v0, cutEdges[idx].v1};
                std::pair<int, int> re = {cutEdges[idx].v1, cutEdges[idx].v0};

                if (useEdges.find(re) != useEdges.end())
                    continue;
                useEdges[e] = idx;
            }

            Polyline polyline;
            polyline.type = type;
            used[idx] = true;

            // Start from this edge
            int startV = cutEdges[idx].v0;
            int endV = cutEdges[idx].v1;
            polyline.vertexIndices.push_back(startV);
            polyline.vertexIndices.push_back(endV);

            // Record endpoint face/edge info
            polyline.startFaceIdx = cutEdges[idx].faceIdx;
            polyline.startEdgeIdx = cutEdges[idx].edgeIdx;
            polyline.endFaceIdx = cutEdges[idx].faceIdx;
            polyline.endEdgeIdx = cutEdges[idx].edgeIdx;

            // Extend towards startV direction (prepend)
            extendPolyline(polyline, startV, cutEdges, vertexToEdges, used, false);

            // Extend towards endV direction (append)
            extendPolyline(polyline, endV, cutEdges, vertexToEdges, used, true);

            polylines.push_back(polyline);
        }

        return polylines;
    }

    inline std::unordered_map<int, std::vector<int>> PolylineManager::buildVertexToEdgesMap(
        const std::vector<CutEdge> &cutEdges,
        const std::vector<int> &edgeIndices)
    {
        std::unordered_map<int, std::vector<int>> vertexToEdges;

        for (int idx : edgeIndices)
        {
            vertexToEdges[cutEdges[idx].v0].push_back(idx);
            vertexToEdges[cutEdges[idx].v1].push_back(idx);
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
        while (true)
        {
            bool found = false;

            auto it = vertexToEdges.find(currentVertex);
            if (it == vertexToEdges.end())
                break;

            for (int edgeIdx : it->second)
            {
                if (used[edgeIdx])
                    continue;

                used[edgeIdx] = true;

                int otherV = (cutEdges[edgeIdx].v0 == currentVertex) ? cutEdges[edgeIdx].v1 : cutEdges[edgeIdx].v0;

                if (forward)
                {
                    polyline.vertexIndices.push_back(otherV);
                    polyline.endFaceIdx = cutEdges[edgeIdx].faceIdx;
                    polyline.endEdgeIdx = cutEdges[edgeIdx].edgeIdx;
                }
                else
                {
                    polyline.vertexIndices.insert(polyline.vertexIndices.begin(), otherV);
                    polyline.startFaceIdx = cutEdges[edgeIdx].faceIdx;
                    polyline.startEdgeIdx = cutEdges[edgeIdx].edgeIdx;
                }

                currentVertex = otherV;
                found = true;
                break;
            }

            if (!found)
                break;
        }
    }

    inline void PolylineManager::tryMergePolylines(std::vector<Polyline> &polylines)
    {
        // Try to merge polylines with matching endpoints
        bool merged = true;
        while (merged)
        {
            merged = false;
            for (int i = 0; i < (int)polylines.size(); i++)
            {
                if (polylines[i].isClosed)
                    continue;

                for (int j = i + 1; j < (int)polylines.size(); j++)
                {
                    if (polylines[j].isClosed)
                        continue;

                    auto &polyI = polylines[i];
                    auto &polyJ = polylines[j];

                    // Check if polyI's end matches polyJ's start
                    if (polyI.vertexIndices.back() == polyJ.vertexIndices.front())
                    {
                        // Merge polyJ into polyI
                        for (int k = 1; k < (int)polyJ.vertexIndices.size(); k++)
                        {
                            polyI.vertexIndices.push_back(polyJ.vertexIndices[k]);
                        }
                        polyI.endFaceIdx = polyJ.endFaceIdx;
                        polyI.endEdgeIdx = polyJ.endEdgeIdx;

                        // Check if closed
                        if (polyI.vertexIndices.front() == polyI.vertexIndices.back())
                        {
                            polyI.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + j);
                        merged = true;
                        break;
                    }

                    // Check if polyI's start matches polyJ's end
                    if (polyI.vertexIndices.front() == polyJ.vertexIndices.back())
                    {
                        // Merge polyJ into polyI (prepend)
                        std::vector<int> newVerts;
                        for (int k = 0; k < (int)polyJ.vertexIndices.size() - 1; k++)
                        {
                            newVerts.push_back(polyJ.vertexIndices[k]);
                        }
                        newVerts.insert(newVerts.end(), polyI.vertexIndices.begin(), polyI.vertexIndices.end());
                        polyI.vertexIndices = newVerts;
                        polyI.startFaceIdx = polyJ.startFaceIdx;
                        polyI.startEdgeIdx = polyJ.startEdgeIdx;

                        // Check if closed
                        if (polyI.vertexIndices.front() == polyI.vertexIndices.back())
                        {
                            polyI.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + j);
                        merged = true;
                        break;
                    }

                    // Check if polyI's end matches polyJ's end (reverse polyJ)
                    if (polyI.vertexIndices.back() == polyJ.vertexIndices.back())
                    {
                        // Reverse polyJ and merge
                        std::vector<int> reversedJ(polyJ.vertexIndices.rbegin(), polyJ.vertexIndices.rend());
                        for (int k = 1; k < (int)reversedJ.size(); k++)
                        {
                            polyI.vertexIndices.push_back(reversedJ[k]);
                        }
                        polyI.endFaceIdx = polyJ.startFaceIdx;
                        polyI.endEdgeIdx = polyJ.startEdgeIdx;

                        // Check if closed
                        if (polyI.vertexIndices.front() == polyI.vertexIndices.back())
                        {
                            polyI.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + j);
                        merged = true;
                        break;
                    }

                    // Check if polyI's start matches polyJ's start (reverse polyJ)
                    if (polyI.vertexIndices.front() == polyJ.vertexIndices.front())
                    {
                        // Reverse polyJ and prepend
                        std::vector<int> reversedJ(polyJ.vertexIndices.rbegin(), polyJ.vertexIndices.rend());
                        std::vector<int> newVerts;
                        for (int k = 0; k < (int)reversedJ.size() - 1; k++)
                        {
                            newVerts.push_back(reversedJ[k]);
                        }
                        newVerts.insert(newVerts.end(), polyI.vertexIndices.begin(), polyI.vertexIndices.end());
                        polyI.vertexIndices = newVerts;
                        polyI.startFaceIdx = polyJ.endFaceIdx;
                        polyI.startEdgeIdx = polyJ.endEdgeIdx;

                        // Check if closed
                        if (polyI.vertexIndices.front() == polyI.vertexIndices.back())
                        {
                            polyI.isClosed = true;
                        }

                        polylines.erase(polylines.begin() + j);
                        merged = true;
                        break;
                    }
                }
                if (merged)
                    break;
            }
        }
    }

} // namespace MeshCutByMark

#endif // POLYLINE_H
