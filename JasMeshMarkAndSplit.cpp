#include "JasMeshMarkAndSplit.h"
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <map>

JasMeshMarkAndSplit::JasMeshMarkAndSplit()
{
}

JasMeshMarkAndSplit::~JasMeshMarkAndSplit()
{
}

std::vector<MeshCutByMark::CutEdge> JasMeshMarkAndSplit::findCutEdges(const std::vector<int>& curFaces) {
    std::vector<MeshCutByMark::CutEdge> cutEdges;

    for (int faceIdx : curFaces) {
        for (int j = 0; j < 3; j++) {
            int v0 = m_pMesh->face[faceIdx].V(j)->Index();
            int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();

            if (v0 > v1) std::swap(v0, v1);

            MeshCutByMark::CutEdgeType type = m_edgeInfoManager.getEdgeType(v0, v1);

            if (type != MeshCutByMark::CUT_EDGE_NONE) {
                MeshCutByMark::CutEdge edge;
                edge.v0 = v0;
                edge.v1 = v1;
                edge.faceIdx = faceIdx;
                edge.edgeIdx = j;
                edge.type = type;
                cutEdges.push_back(edge);
            }
        }
    }

    return cutEdges;
}

std::vector<std::vector<int>> JasMeshMarkAndSplit::extractBoundaryEdges(
    const std::vector<int>& regionFaces
) {
    std::vector<std::vector<int>> boundaries;

    // 找到所有边界边（只被一个面使用的边）
    std::vector<std::pair<int,int>> boundaryEdges;
    std::unordered_map<std::pair<int,int>, int, MeshCutByMark::EdgeHash, MeshCutByMark::EdgeEqual> edgeCount;

    for (int faceIdx : regionFaces) {
        for (int j = 0; j < 3; j++) {
            int v0 = m_pMesh->face[faceIdx].V(j)->Index();
            int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();

            if (v0 > v1) std::swap(v0, v1);

            edgeCount[{v0, v1}]++;
        }
    }

    // 只保留边界边
    for (const auto& [edge, count] : edgeCount) {
        if (count == 1) {
            boundaryEdges.push_back(edge);
        }
    }

    // 构建边界边的邻接关系
    std::unordered_map<int, std::vector<int>> vertexToEdges;
    for (int i = 0; i < (int)boundaryEdges.size(); i++) {
        vertexToEdges[boundaryEdges[i].first].push_back(i);
        vertexToEdges[boundaryEdges[i].second].push_back(i);
    }

    // 沿边界遍历形成闭合环
    std::vector<bool> used(boundaryEdges.size(), false);
    for (int i = 0; i < (int)boundaryEdges.size(); i++) {
        if (used[i]) continue;

        std::vector<int> boundary;
        int startV = boundaryEdges[i].first;
        int curV = boundaryEdges[i].second;
        boundary.push_back(startV);
        used[i] = true;

        while (curV != startV) {
            boundary.push_back(curV);

            // 找到下一条边界边
            bool found = false;
            for (int edgeIdx : vertexToEdges[curV]) {
                if (!used[edgeIdx]) {
                    used[edgeIdx] = true;
                    curV = (boundaryEdges[edgeIdx].first == curV) ?
                           boundaryEdges[edgeIdx].second :
                           boundaryEdges[edgeIdx].first;
                    found = true;
                    break;
                }
            }

            if (!found) break; // 异常情况
        }

        if (boundary.size() >= 3) {
            boundaries.push_back(boundary);
        }
    }

    return boundaries;
}

void JasMeshMarkAndSplit::SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs)
{
	// Phase 1: 初始化
	m_newMarkCounter = 1;
	m_regionMarker.initNewMark(m_pMesh);

	// 构建边信息
	m_edgeInfoManager.buildEdgeInfo(m_pMesh);

	// 确保 FF 邻接可用并计算拓扑
	if (!m_pMesh->face.IsFFAdjacencyEnabled()) {
		m_pMesh->face.EnableFFAdjacency();
	}
	vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*m_pMesh);

	// 计算法向量
	vcg::tri::UpdateNormal<CMeshOD>::PerFace(*m_pMesh);

	// Phase 2: 延长线切割并标记新区域
	for (int i = 0; i < (int)m_pMesh->face.size(); i++) {
		if (m_pMesh->face[i].IsD()) continue;
		if (m_regionMarker.getNewMark(i) > 0) continue;  // 已处理

		// 2.1 flood-fill 找连通区域
		int targetMark = m_pMesh->face[i].IMark();
		std::vector<int> curFaces = m_regionMarker.floodFill(i, targetMark, m_pMesh, m_edgeInfoManager);

		// 2.2 找 curFaces 的切割边
		std::vector<MeshCutByMark::CutEdge> cutEdges = findCutEdges(curFaces);

		// 2.3 将切割边连接成连续折线
		std::vector<MeshCutByMark::Polyline> polylines =
			m_polylineManager.connectEdgesToPolylines(cutEdges, m_pMesh);

		// 2.4 从端点延长切割
		for (const auto& polyline : polylines) {
			// 检查首端点
			if (!m_cutPlaneManager.isOnMarkDiffEdge(
					polyline.startFaceIdx, polyline.startEdgeIdx, m_pMesh)) {
				vcg::Plane3d plane = m_cutPlaneManager.makeCutPlane(polyline, true, m_pMesh);
				for (int faceIdx : curFaces) {
					if (!m_pMesh->face[faceIdx].IsD()) {
						m_cutPlaneManager.cutTriangleByPlane(faceIdx, plane, m_pMesh);
					}
				}
			}

			// 检查尾端点
			if (!m_cutPlaneManager.isOnMarkDiffEdge(
					polyline.endFaceIdx, polyline.endEdgeIdx, m_pMesh)) {
				vcg::Plane3d plane = m_cutPlaneManager.makeCutPlane(polyline, false, m_pMesh);
				for (int faceIdx : curFaces) {
					if (!m_pMesh->face[faceIdx].IsD()) {
						m_cutPlaneManager.cutTriangleByPlane(faceIdx, plane, m_pMesh);
					}
				}
			}
		}

		// 2.5 通过拣选得到切割后的子区域
		std::vector<std::vector<int>> subRegions =
			m_regionMarker.extractSubRegions(curFaces, m_pMesh);

		// 2.6 每个子区域标记新 mark
		m_regionMarker.markSubRegions(subRegions, m_pMesh, m_newMarkCounter);
	}

	// Phase 3: 根据新标记提取多边形
	std::map<int, std::vector<int>> markToFaces;
	for (int i = 0; i < (int)m_pMesh->face.size(); i++) {
		if (!m_pMesh->face[i].IsD()) {
			markToFaces[m_regionMarker.getNewMark(i)].push_back(i);
		}
	}

	// 输出结果
	for (const auto& [newMark, faces] : markToFaces) {
		// 提取边界边
		std::vector<std::vector<int>> boundaries = extractBoundaryEdges(faces);

		// 构造 splitReg
		splitReg reg;
		reg.mark = m_pMesh->face[faces[0]].IMark();
		reg.newMark = newMark;
		reg.inTris = faces;
		reg.normal = m_pMesh->face[faces[0]].N();
		reg.boundlines = boundaries.empty() ? std::vector<int>() : boundaries[0];

		retRegs.push_back(reg);
	}
}
