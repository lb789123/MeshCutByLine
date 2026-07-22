#include "JasMeshMarkAndSplit.h"
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <filesystem>
#include <fstream>
#include <set>
#include <map>
#include <cstdlib>
#include <ctime>

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

// ============ 调试输出辅助方法 ============

void JasMeshMarkAndSplit::debugEnsureDir() {
	std::filesystem::create_directories(m_debugOutputDir);
}

void JasMeshMarkAndSplit::debugWritePolylines(
	int iterIdx,
	const std::vector<MeshCutByMark::Polyline>& polylines)
{
	if (!m_debug || polylines.empty()) return;
	debugEnsureDir();

	std::string path = m_debugOutputDir + "iter_" + std::to_string(iterIdx) + "_polylines.obj";
	std::ofstream ofs(path);
	if (!ofs.is_open()) return;

	// 收集折线涉及的所有顶点（去重，保持顺序）
	std::vector<int> vertMap;
	std::unordered_map<int, int> globalToLocal;
	for (const auto& pl : polylines) {
		for (int vi : pl.vertexIndices) {
			if (globalToLocal.find(vi) == globalToLocal.end()) {
				int localIdx = static_cast<int>(vertMap.size());
				globalToLocal[vi] = localIdx;
				vertMap.push_back(vi);
			}
		}
	}

	// 写入顶点
	for (int vi : vertMap) {
		const auto& p = m_pMesh->vert[vi].P();
		ofs << "v " << p.X() << " " << p.Y() << " " << p.Z() << "\n";
	}

	// 写入每条折线（OBJ 索引从 1 开始）
	for (const auto& pl : polylines) {
		ofs << "l";
		for (int vi : pl.vertexIndices) {
			ofs << " " << (globalToLocal[vi] + 1);
		}
		ofs << "\n";
	}

	ofs.close();
}

void JasMeshMarkAndSplit::debugWriteFacesOFF(
	int iterIdx,
	const char* suffix,
	const std::vector<int>& faceIndices)
{
	if (!m_debug || faceIndices.empty()) return;
	debugEnsureDir();

	std::string path = m_debugOutputDir + "iter_" + std::to_string(iterIdx) + "_" + suffix + ".off";
	std::ofstream ofs(path);
	if (!ofs.is_open()) return;

	// 收集涉及的顶点并建立全局->局部映射
	std::vector<int> vertMap;
	std::unordered_map<int, int> globalToLocal;
	for (int fi : faceIndices) {
		for (int j = 0; j < 3; j++) {
			int vi = m_pMesh->face[fi].V(j)->Index();
			if (globalToLocal.find(vi) == globalToLocal.end()) {
				int localIdx = static_cast<int>(vertMap.size());
				globalToLocal[vi] = localIdx;
				vertMap.push_back(vi);
			}
		}
	}

	int nV = static_cast<int>(vertMap.size());
	int nF = static_cast<int>(faceIndices.size());

	ofs << "OFF\n";
	ofs << nV << " " << nF << " 0\n";

	// 写入顶点
	for (int vi : vertMap) {
		const auto& p = m_pMesh->vert[vi].P();
		ofs << p.X() << " " << p.Y() << " " << p.Z() << "\n";
	}

	// 写入三角面（局部索引）
	for (int fi : faceIndices) {
		ofs << "3";
		for (int j = 0; j < 3; j++) {
			int vi = m_pMesh->face[fi].V(j)->Index();
			ofs << " " << globalToLocal[vi];
		}
		ofs << "\n";
	}

	ofs.close();
}

void JasMeshMarkAndSplit::debugWriteSubRegionsOFF(
	int iterIdx,
	const std::vector<std::vector<int>>& subRegions)
{
	if (!m_debug) return;
	for (int j = 0; j < (int)subRegions.size(); j++) {
		std::string suffix = "sub_region_" + std::to_string(j);
		debugWriteFacesOFF(iterIdx, suffix.c_str(), subRegions[j]);
	}
}

void JasMeshMarkAndSplit::debugWritePolygonsOBJ(
	const std::map<int, std::vector<int>>& markToFaces)
{
	if (!m_debug) return;
	debugEnsureDir();

	std::string path = m_debugOutputDir + "final_polygons.obj";
	std::ofstream ofs(path);
	if (!ofs.is_open()) return;

	int globalVertOffset = 0;
	for (const auto& [newMark, faces] : markToFaces) {
		// 提取边界
		std::vector<std::vector<int>> boundaries = extractBoundaryEdges(faces);
		if (boundaries.empty()) continue;

		// 收集本区域涉及的顶点
		std::vector<int> vertMap;
		std::unordered_map<int, int> globalToLocal;
		for (const auto& boundary : boundaries) {
			for (int vi : boundary) {
				if (globalToLocal.find(vi) == globalToLocal.end()) {
					int localIdx = static_cast<int>(vertMap.size());
					globalToLocal[vi] = localIdx;
					vertMap.push_back(vi);
				}
			}
		}

		ofs << "# newMark = " << newMark << "\n";

		// 写入顶点
		for (int vi : vertMap) {
			const auto& p = m_pMesh->vert[vi].P();
			ofs << "v " << p.X() << " " << p.Y() << " " << p.Z() << "\n";
		}

		// 写入多边形面（OBJ 索引从 1 开始，加上全局偏移）
		for (const auto& boundary : boundaries) {
			ofs << "f";
			for (int vi : boundary) {
				ofs << " " << (globalToLocal[vi] + 1 + globalVertOffset);
			}
			ofs << "\n";
		}

		globalVertOffset += static_cast<int>(vertMap.size());
	}

	ofs.close();
}

void JasMeshMarkAndSplit::debugSaveColoredMesh(const std::vector<splitReg>& regs)
{
	if (!m_debug || regs.empty() || !m_pMesh) return;
	debugEnsureDir();

	// 启用面颜色
	if (!m_pMesh->face.IsColorEnabled()) {
		m_pMesh->face.EnableColor();
	}

	// 为每个区域生成随机颜色，用 map 管理 (索引 i -> 颜色)
	std::map<int, vcg::Color4b> colorMap;
	std::srand(static_cast<unsigned>(std::time(nullptr)));
	for (int i = 0; i < (int)regs.size(); i++) {
		unsigned char r = static_cast<unsigned char>(std::rand() % 256);
		unsigned char g = static_cast<unsigned char>(std::rand() % 256);
		unsigned char b = static_cast<unsigned char>(std::rand() % 256);
		colorMap[i] = vcg::Color4b(r, g, b, 255);
	}

	// 给每个区域的三角形赋颜色
	for (int i = 0; i < (int)regs.size(); i++) {
		for (int fi : regs[i].inTris) {
			m_pMesh->face[fi].C() = colorMap[i];
		}
	}

	// 保存带颜色的 OBJ（顶点颜色 = 所属面颜色的平均）
	std::string path = m_debugOutputDir + "colored_mesh.obj";
	std::ofstream ofs(path);
	if (!ofs.is_open()) return;

	// 计算顶点颜色：每个顶点取相邻面颜色的平均
	int nV = m_pMesh->VN();
	int nF = m_pMesh->FN();
	std::vector<vcg::Point4f> vertColors(nV, vcg::Point4f(0, 0, 0, 0));
	std::vector<int> vertFaceCount(nV, 0);

	for (int i = 0; i < nF; i++) {
		if (m_pMesh->face[i].IsD()) continue;
		vcg::Color4b fc = m_pMesh->face[i].C();
		for (int j = 0; j < 3; j++) {
			int vi = m_pMesh->face[i].V(j)->Index();
			vertColors[vi].X() += fc.X() / 255.0f;
			vertColors[vi].Y() += fc.Y() / 255.0f;
			vertColors[vi].Z() += fc.Z() / 255.0f;
			vertFaceCount[vi]++;
		}
	}

	for (int i = 0; i < nV; i++) {
		if (vertFaceCount[i] > 0) {
			vertColors[i].X() /= vertFaceCount[i];
			vertColors[i].Y() /= vertFaceCount[i];
			vertColors[i].Z() /= vertFaceCount[i];
		}
	}

	// 写入 OBJ（带顶点颜色）
	for (int i = 0; i < nV; i++) {
		const auto& p = m_pMesh->vert[i].P();
		ofs << "v " << p.X() << " " << p.Y() << " " << p.Z()
		    << " " << vertColors[i].X()
		    << " " << vertColors[i].Y()
		    << " " << vertColors[i].Z() << "\n";
	}

	for (int i = 0; i < nF; i++) {
		if (m_pMesh->face[i].IsD()) continue;
		ofs << "f"
		    << " " << (m_pMesh->face[i].V(0)->Index() + 1)
		    << " " << (m_pMesh->face[i].V(1)->Index() + 1)
		    << " " << (m_pMesh->face[i].V(2)->Index() + 1) << "\n";
	}

	ofs.close();
}

void JasMeshMarkAndSplit::SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs)
{
	// Phase 1: 初始化
	m_newMarkCounter = 1;
	m_debugIterCounter = 0;
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

		// 调试输出：flood-fill 区域
		debugWriteFacesOFF(m_debugIterCounter, "cur_faces", curFaces);

		// 2.2 找 curFaces 的切割边
		std::vector<MeshCutByMark::CutEdge> cutEdges = findCutEdges(curFaces);

		// 2.3 将切割边连接成连续折线
		std::vector<MeshCutByMark::Polyline> polylines =
			m_polylineManager.connectEdgesToPolylines(cutEdges, m_pMesh);

		// 调试输出：折线
		debugWritePolylines(m_debugIterCounter, polylines);

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

		// 调试输出：子区域
		debugWriteSubRegionsOFF(m_debugIterCounter, subRegions);

		// 2.6 每个子区域标记新 mark
		m_regionMarker.markSubRegions(subRegions, m_pMesh, m_newMarkCounter);

		m_debugIterCounter++;
	}

	// Phase 3: 根据新标记提取多边形
	std::map<int, std::vector<int>> markToFaces;
	for (int i = 0; i < (int)m_pMesh->face.size(); i++) {
		if (!m_pMesh->face[i].IsD()) {
			markToFaces[m_regionMarker.getNewMark(i)].push_back(i);
		}
	}

	// 调试输出：最终多边形
	debugWritePolygonsOBJ(markToFaces);

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

	// 调试输出：带颜色的网格
	debugSaveColoredMesh(retRegs);
}
