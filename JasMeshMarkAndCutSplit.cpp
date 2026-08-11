#include "JasMeshMarkAndCutSplit.h"
#include "JasMeshLocalMarkAndCutSplitInternal.h"
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <filesystem>
#include <fstream>
#include <set>
#include <map>
#include <cstdlib>
#include <ctime>

// 构造函数
JasMeshMarkAndCutSplit::JasMeshMarkAndCutSplit()
{
}

// 析构函数
JasMeshMarkAndCutSplit::~JasMeshMarkAndCutSplit()
{
}

// 查找给定面集合中的切割边
std::vector<MeshCutByMark::CutEdge> JasMeshMarkAndCutSplit::findCutEdges(const std::vector<int>& curFaces)
{
    std::vector<MeshCutByMark::CutEdge> cutEdges;

    for (int faceIdx : curFaces)
    {
        for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
        {
            int vertex0 = m_pMesh->face[faceIdx].V(edgeIndex)->Index();
            int vertex1 = m_pMesh->face[faceIdx].V((edgeIndex + 1) % 3)->Index();

            if (vertex0 > vertex1)
            {
                std::swap(vertex0, vertex1);
            }

            MeshCutByMark::CutEdgeType type = m_edgeInfoManager.getEdgeType(vertex0, vertex1);

            if (type != MeshCutByMark::CUT_EDGE_NONE)
            {
                MeshCutByMark::CutEdge edge;
                edge.v0 = vertex0;
                edge.v1 = vertex1;
                edge.faceIdx = faceIdx;
                edge.edgeIdx = edgeIndex;
                edge.type = type;
                cutEdges.push_back(edge);
            }
        }
    }

    return cutEdges;
}

// 提取区域的边界边并连接成闭合环
std::vector<std::vector<int>> JasMeshMarkAndCutSplit::extractBoundaryEdges(
    const std::vector<int>& regionFaces
)
{
    std::vector<std::vector<int>> boundaries;

    // 找到所有边界边（只被一个面使用的边）
    std::vector<std::pair<int, int>> boundaryEdges;
    std::unordered_map<std::pair<int, int>, int, MeshCutByMark::EdgeHash, MeshCutByMark::EdgeEqual> edgeCount;

    for (int faceIdx : regionFaces)
    {
        for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
        {
            int vertex0 = m_pMesh->face[faceIdx].V(edgeIndex)->Index();
            int vertex1 = m_pMesh->face[faceIdx].V((edgeIndex + 1) % 3)->Index();

            if (vertex0 > vertex1)
            {
                std::swap(vertex0, vertex1);
            }

            edgeCount[{vertex0, vertex1}]++;
        }
    }

    // 只保留边界边
    for (const auto& [edge, count] : edgeCount)
    {
        if (count == 1)
        {
            boundaryEdges.push_back(edge);
        }
    }

    // 构建边界边的邻接关系
    std::unordered_map<int, std::vector<int>> vertexToEdges;
    for (int boundaryEdgeIndex = 0; boundaryEdgeIndex < (int)boundaryEdges.size(); boundaryEdgeIndex++)
    {
        vertexToEdges[boundaryEdges[boundaryEdgeIndex].first].push_back(boundaryEdgeIndex);
        vertexToEdges[boundaryEdges[boundaryEdgeIndex].second].push_back(boundaryEdgeIndex);
    }

    // 沿边界遍历形成闭合环
    std::vector<bool> used(boundaryEdges.size(), false);
    for (int boundaryEdgeIndex = 0; boundaryEdgeIndex < (int)boundaryEdges.size(); boundaryEdgeIndex++)
    {
        if (used[boundaryEdgeIndex])
        {
            continue;
        }

        std::vector<int> boundary;
        int startVertex = boundaryEdges[boundaryEdgeIndex].first;
        int currentVertex = boundaryEdges[boundaryEdgeIndex].second;
        boundary.push_back(startVertex);
        used[boundaryEdgeIndex] = true;

        while (currentVertex != startVertex)
        {
            boundary.push_back(currentVertex);

            // 找到下一条边界边
            bool found = false;
            for (int edgeIdx : vertexToEdges[currentVertex])
            {
                if (!used[edgeIdx])
                {
                    used[edgeIdx] = true;
                    currentVertex = (boundaryEdges[edgeIdx].first == currentVertex)
                                        ? boundaryEdges[edgeIdx].second
                                        : boundaryEdges[edgeIdx].first;
                    found = true;
                    break;
                }
            }

            if (!found)
            {
                break; // 异常情况
            }
        }

        if (boundary.size() >= 3)
        {
            boundaries.push_back(boundary);
        }
    }

    return boundaries;
}

// ============ 调试输出辅助方法 ============

// 确保调试输出目录存在
void JasMeshMarkAndCutSplit::debugEnsureDir()
{
    std::filesystem::create_directories(m_debugOutputDir);
}

// 将折线写入 OBJ 调试文件
void JasMeshMarkAndCutSplit::debugWritePolylines(
    int iterIdx,
    const std::vector<MeshCutByMark::Polyline>& polylines
)
{
    if (!m_debug || polylines.empty())
    {
        return;
    }
    debugEnsureDir();

    std::string path = m_debugOutputDir + "iter_" + std::to_string(iterIdx) + "_polylines.obj";
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        return;
    }

    // 收集折线涉及的所有顶点（去重，保持顺序）
    std::vector<int> vertMap;
    std::unordered_map<int, int> globalToLocal;
    for (const auto& polyline : polylines)
    {
        for (int vertexIndex : polyline.vertexIndices)
        {
            if (globalToLocal.find(vertexIndex) == globalToLocal.end())
            {
                int localIdx = static_cast<int>(vertMap.size());
                globalToLocal[vertexIndex] = localIdx;
                vertMap.push_back(vertexIndex);
            }
        }
    }

    // 写入顶点
    for (int vertexIndex : vertMap)
    {
        const auto& point = m_pMesh->vert[vertexIndex].P();
        ofs << "v " << point.X() << " " << point.Y() << " " << point.Z() << "\n";
    }

    // 写入每条折线（OBJ 索引从 1 开始）
    for (const auto& polyline : polylines)
    {
        ofs << "l";
        for (int vertexIndex : polyline.vertexIndices)
        {
            ofs << " " << (globalToLocal[vertexIndex] + 1);
        }
        ofs << "\n";
    }

    ofs.close();
}

// 将面集合写入 OFF 调试文件
void JasMeshMarkAndCutSplit::debugWriteFacesOFF(
    int iterIdx,
    const char* suffix,
    const std::vector<int>& faceIndices
)
{
    if (!m_debug || faceIndices.empty())
    {
        return;
    }
    debugEnsureDir();

    std::string path = m_debugOutputDir + "iter_" + std::to_string(iterIdx) + "_" + suffix + ".off";
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        return;
    }

    // 收集涉及的顶点并建立全局->局部映射
    std::vector<int> vertMap;
    std::unordered_map<int, int> globalToLocal;
    for (int faceIndex : faceIndices)
    {
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++)
        {
            int vertexIndex = m_pMesh->face[faceIndex].V(cornerIndex)->Index();
            if (globalToLocal.find(vertexIndex) == globalToLocal.end())
            {
                int localIdx = static_cast<int>(vertMap.size());
                globalToLocal[vertexIndex] = localIdx;
                vertMap.push_back(vertexIndex);
            }
        }
    }

    int vertexCount = static_cast<int>(vertMap.size());
    int faceCount = static_cast<int>(faceIndices.size());

    ofs << "OFF\n";
    ofs << vertexCount << " " << faceCount << " 0\n";

    // 写入顶点
    for (int vertexIndex : vertMap)
    {
        const auto& point = m_pMesh->vert[vertexIndex].P();
        ofs << point.X() << " " << point.Y() << " " << point.Z() << "\n";
    }

    // 写入三角面（局部索引）
    for (int faceIndex : faceIndices)
    {
        ofs << "3";
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++)
        {
            int vertexIndex = m_pMesh->face[faceIndex].V(cornerIndex)->Index();
            ofs << " " << globalToLocal[vertexIndex];
        }
        ofs << "\n";
    }

    ofs.close();
}

// 将多个子区域写入 OFF 调试文件
void JasMeshMarkAndCutSplit::debugWriteSubRegionsOFF(
    int iterIdx,
    const std::vector<std::vector<int>>& subRegions
)
{
    if (!m_debug)
    {
        return;
    }
    for (int regionIndex = 0; regionIndex < (int)subRegions.size(); regionIndex++)
    {
        std::string suffix = "sub_region_" + std::to_string(regionIndex);
        debugWriteFacesOFF(iterIdx, suffix.c_str(), subRegions[regionIndex]);
    }
}

// 将最终多边形写入 OBJ 调试文件
void JasMeshMarkAndCutSplit::debugWritePolygonsOBJ(
    const std::map<int, std::vector<int>>& markToFaces
)
{
    if (!m_debug)
    {
        return;
    }
    debugEnsureDir();

    std::string path = m_debugOutputDir + "final_polygons.obj";
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        return;
    }

    int globalVertexOffset = 0;
    for (const auto& [newMark, faces] : markToFaces)
    {
        // 提取边界
        std::vector<std::vector<int>> boundaries = extractBoundaryEdges(faces);
        if (boundaries.empty())
        {
            continue;
        }

        // 收集本区域涉及的顶点
        std::vector<int> vertMap;
        std::unordered_map<int, int> globalToLocal;
        for (const auto& boundary : boundaries)
        {
            for (int vertexIndex : boundary)
            {
                if (globalToLocal.find(vertexIndex) == globalToLocal.end())
                {
                    int localIdx = static_cast<int>(vertMap.size());
                    globalToLocal[vertexIndex] = localIdx;
                    vertMap.push_back(vertexIndex);
                }
            }
        }

        ofs << "# newMark = " << newMark << "\n";

        // 写入顶点
        for (int vertexIndex : vertMap)
        {
            const auto& point = m_pMesh->vert[vertexIndex].P();
            ofs << "v " << point.X() << " " << point.Y() << " " << point.Z() << "\n";
        }

        // 写入多边形面（OBJ 索引从 1 开始，加上全局偏移）
        for (const auto& boundary : boundaries)
        {
            ofs << "f";
            for (int vertexIndex : boundary)
            {
                ofs << " " << (globalToLocal[vertexIndex] + 1 + globalVertexOffset);
            }
            ofs << "\n";
        }

        globalVertexOffset += static_cast<int>(vertMap.size());
    }

    ofs.close();
}

// 保存带随机颜色的网格 OBJ 调试文件
void JasMeshMarkAndCutSplit::debugSaveColoredMesh(const std::vector<splitReg>& regs)
{
    if (!m_debug || regs.empty() || !m_pMesh)
    {
        return;
    }
    debugEnsureDir();

    // 启用面颜色
    if (!m_pMesh->face.IsColorEnabled())
    {
        m_pMesh->face.EnableColor();
    }

    // 为每个区域生成随机颜色，用 map 管理（区域索引 -> 颜色）
    std::map<int, vcg::Color4b> colorMap;
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    for (int regionIndex = 0; regionIndex < (int)regs.size(); regionIndex++)
    {
        unsigned char red = static_cast<unsigned char>(std::rand() % 256);
        unsigned char green = static_cast<unsigned char>(std::rand() % 256);
        unsigned char blue = static_cast<unsigned char>(std::rand() % 256);
        colorMap[regionIndex] = vcg::Color4b(red, green, blue, 255);
    }

    // 给每个区域的三角形赋颜色
    for (int regionIndex = 0; regionIndex < (int)regs.size(); regionIndex++)
    {
        for (int faceIndex : regs[regionIndex].inTris)
        {
            m_pMesh->face[faceIndex].C() = colorMap[regionIndex];
        }
    }

    // 保存带颜色的 OBJ（顶点颜色 = 所属面颜色的平均）
    std::string path = m_debugOutputDir + "colored_mesh.obj";
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
        return;
    }

    // 计算顶点颜色：每个顶点取相邻面颜色的平均
    int vertexCount = m_pMesh->VN();
    int faceCount = m_pMesh->FN();
    std::vector<vcg::Point4f> vertColors(vertexCount, vcg::Point4f(0, 0, 0, 0));
    std::vector<int> vertFaceCount(vertexCount, 0);

    for (int faceIndex = 0; faceIndex < faceCount; faceIndex++)
    {
        if (m_pMesh->face[faceIndex].IsD())
        {
            continue;
        }
        vcg::Color4b faceColor = m_pMesh->face[faceIndex].C();
        for (int cornerIndex = 0; cornerIndex < 3; cornerIndex++)
        {
            int vertexIndex = m_pMesh->face[faceIndex].V(cornerIndex)->Index();
            vertColors[vertexIndex].X() += faceColor.X() / 255.0f;
            vertColors[vertexIndex].Y() += faceColor.Y() / 255.0f;
            vertColors[vertexIndex].Z() += faceColor.Z() / 255.0f;
            vertFaceCount[vertexIndex]++;
        }
    }

    for (int vertexIndex = 0; vertexIndex < vertexCount; vertexIndex++)
    {
        if (vertFaceCount[vertexIndex] > 0)
        {
            vertColors[vertexIndex].X() /= vertFaceCount[vertexIndex];
            vertColors[vertexIndex].Y() /= vertFaceCount[vertexIndex];
            vertColors[vertexIndex].Z() /= vertFaceCount[vertexIndex];
        }
    }

    // 写入 OBJ（带顶点颜色）
    for (int vertexIndex = 0; vertexIndex < vertexCount; vertexIndex++)
    {
        const auto& point = m_pMesh->vert[vertexIndex].P();
        ofs << "v " << point.X() << " " << point.Y() << " " << point.Z()
            << " " << vertColors[vertexIndex].X()
            << " " << vertColors[vertexIndex].Y()
            << " " << vertColors[vertexIndex].Z() << "\n";
    }

    for (int faceIndex = 0; faceIndex < faceCount; faceIndex++)
    {
        if (m_pMesh->face[faceIndex].IsD())
        {
            continue;
        }
        ofs << "f"
            << " " << (m_pMesh->face[faceIndex].V(0)->Index() + 1)
            << " " << (m_pMesh->face[faceIndex].V(1)->Index() + 1)
            << " " << (m_pMesh->face[faceIndex].V(2)->Index() + 1) << "\n";
    }

    ofs.close();
}

// 主流程：按 mark 与切割边将网格分割为简单多边形
void JasMeshMarkAndCutSplit::SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs)
{
    // Phase 1: 初始化
    m_newMarkCounter = 1;
    m_debugIterCounter = 0;
    m_regionMarker.initNewMark(m_pMesh);

    // 构建边信息
    m_edgeInfoManager.buildEdgeInfo(m_pMesh);

    // 确保 FF 邻接可用并计算拓扑
    if (!m_pMesh->face.IsFFAdjacencyEnabled())
    {
        m_pMesh->face.EnableFFAdjacency();
    }
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*m_pMesh);

    // 计算法向量
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(*m_pMesh);

    // Phase 2: 延长线切割并标记新区域
    for (int faceIndex = 0; faceIndex < (int)m_pMesh->face.size(); faceIndex++)
    {
        if (m_pMesh->face[faceIndex].IsD())
        {
            continue;
        }
        if (m_regionMarker.getNewMark(faceIndex) > 0)
        {
            continue; // 已处理
        }

        // 2.1 flood-fill 找连通区域
        int targetMark = m_pMesh->face[faceIndex].IMark();
        std::vector<int> curFaces = m_regionMarker.floodFill(faceIndex, targetMark, m_pMesh, m_edgeInfoManager);

        // 调试输出：flood-fill 区域
        debugWriteFacesOFF(m_debugIterCounter, "cur_faces", curFaces);

        // 2.2 找 curFaces 的切割边
        std::vector<MeshCutByMark::CutEdge> cutEdges = findCutEdges(curFaces);

        // 2.3 将切割边连接成连续折线
        std::vector<MeshCutByMark::Polyline> polylines =
            m_polylineManager.connectEdgesToPolylines(cutEdges, m_pMesh);

        // 调试输出：折线
        debugWritePolylines(m_debugIterCounter, polylines);

        // 2.4 从端点延长切割：局部 mesh + cutter + 合并回主网格（targetMark 在上文已定义）
        m_localMeshCut.cutRegion(m_pMesh, curFaces, polylines, targetMark, m_regionMarker);

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
    for (int faceIndex = 0; faceIndex < (int)m_pMesh->face.size(); faceIndex++)
    {
        if (!m_pMesh->face[faceIndex].IsD())
        {
            markToFaces[m_regionMarker.getNewMark(faceIndex)].push_back(faceIndex);
        }
    }

    // 调试输出：最终多边形
    debugWritePolygonsOBJ(markToFaces);

    // 输出结果
    for (const auto& [newMark, faces] : markToFaces)
    {
        // 提取边界环（外圈 + 洞），由外部库 cgalLocalMeshCut 提供
        std::vector<std::vector<int>> boundaries =
            jaslmc::SubRegionBoundary(*m_pMesh, faces);

        // 构造 splitReg
        splitReg region;
        region.mark = m_pMesh->face[faces[0]].IMark();
        region.newMark = newMark;
        region.inTris = faces;
        region.normal = m_pMesh->face[faces[0]].N();
        region.boundlines = boundaries.empty() ? std::vector<int>() : boundaries[0];
        region.boundaries = boundaries;

        retRegs.push_back(region);
    }

    // 调试输出：带颜色的网格
    debugSaveColoredMesh(retRegs);
}
