// tool/polyline.h
#ifndef POLYLINE_H
#define POLYLINE_H

#include <vector>
#include <unordered_map>
#include "edge_info.h"

namespace MeshCutByMark {

// Polyline: a connected sequence of vertices formed by linking cut edges
struct Polyline {
    std::vector<int> vertexIndices;  // vertex sequence of the polyline
    int startFaceIdx;                // face index at the start endpoint
    int startEdgeIdx;                // edge index within that face at the start
    int endFaceIdx;                  // face index at the end endpoint
    int endEdgeIdx;                  // edge index within that face at the end
};

// PolylineManager: connects scattered cut edges into continuous polylines
class PolylineManager {
public:
    // Connect cut edges into continuous polylines
    std::vector<Polyline> connectEdgesToPolylines(
        const std::vector<CutEdge>& cutEdges,
        CMeshO* mesh
    );

private:
    // Build vertex -> list-of-edge-indices mapping
    std::unordered_map<int, std::vector<int>> buildVertexToEdgesMap(
        const std::vector<CutEdge>& cutEdges
    );

    // Extend a polyline in one direction from currentVertex
    void extendPolyline(
        Polyline& polyline,
        int& currentVertex,
        const std::vector<CutEdge>& cutEdges,
        const std::unordered_map<int, std::vector<int>>& vertexToEdges,
        std::vector<bool>& used,
        bool forward
    );
};

// --- Implementations ---

inline std::vector<Polyline> PolylineManager::connectEdgesToPolylines(
    const std::vector<CutEdge>& cutEdges,
    CMeshO* mesh
) {
    std::vector<Polyline> polylines;

    if (cutEdges.empty()) return polylines;

    // Build vertex -> edges mapping
    auto vertexToEdges = buildVertexToEdgesMap(cutEdges);

    // Track which edges have been used
    std::vector<bool> used(cutEdges.size(), false);

    for (size_t i = 0; i < cutEdges.size(); i++) {
        if (used[i]) continue;

        Polyline polyline;
        used[i] = true;

        // Start from this edge
        int startV = cutEdges[i].v0;
        int endV = cutEdges[i].v1;
        polyline.vertexIndices.push_back(startV);
        polyline.vertexIndices.push_back(endV);

        // Record endpoint face/edge info
        polyline.startFaceIdx = cutEdges[i].faceIdx;
        polyline.startEdgeIdx = cutEdges[i].edgeIdx;
        polyline.endFaceIdx = cutEdges[i].faceIdx;
        polyline.endEdgeIdx = cutEdges[i].edgeIdx;

        // Extend towards startV direction (prepend)
        extendPolyline(polyline, startV, cutEdges, vertexToEdges, used, false);

        // Extend towards endV direction (append)
        extendPolyline(polyline, endV, cutEdges, vertexToEdges, used, true);

        polylines.push_back(polyline);
    }

    return polylines;
}

inline std::unordered_map<int, std::vector<int>> PolylineManager::buildVertexToEdgesMap(
    const std::vector<CutEdge>& cutEdges
) {
    std::unordered_map<int, std::vector<int>> vertexToEdges;

    for (size_t i = 0; i < cutEdges.size(); i++) {
        vertexToEdges[cutEdges[i].v0].push_back(static_cast<int>(i));
        vertexToEdges[cutEdges[i].v1].push_back(static_cast<int>(i));
    }

    return vertexToEdges;
}

inline void PolylineManager::extendPolyline(
    Polyline& polyline,
    int& currentVertex,
    const std::vector<CutEdge>& cutEdges,
    const std::unordered_map<int, std::vector<int>>& vertexToEdges,
    std::vector<bool>& used,
    bool forward
) {
    while (true) {
        bool found = false;

        auto it = vertexToEdges.find(currentVertex);
        if (it == vertexToEdges.end()) break;

        for (int edgeIdx : it->second) {
            if (used[edgeIdx]) continue;

            used[edgeIdx] = true;

            int otherV = (cutEdges[edgeIdx].v0 == currentVertex) ?
                         cutEdges[edgeIdx].v1 :
                         cutEdges[edgeIdx].v0;

            if (forward) {
                polyline.vertexIndices.push_back(otherV);
                polyline.endFaceIdx = cutEdges[edgeIdx].faceIdx;
                polyline.endEdgeIdx = cutEdges[edgeIdx].edgeIdx;
            } else {
                polyline.vertexIndices.insert(polyline.vertexIndices.begin(), otherV);
                polyline.startFaceIdx = cutEdges[edgeIdx].faceIdx;
                polyline.startEdgeIdx = cutEdges[edgeIdx].edgeIdx;
            }

            currentVertex = otherV;
            found = true;
            break;
        }

        if (!found) break;
    }
}

} // namespace MeshCutByMark

#endif // POLYLINE_H
