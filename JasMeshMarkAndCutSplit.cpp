#include "JasMeshMarkAndCutSplit.h"
#include "JasMeshLocalMarkAndCutSplitInternal.h"
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <filesystem>
#include <fstream>
#include <future>
#include <set>
#include <map>
#include <utility>
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

    // 每个区域的 NewMark 分配一种颜色（newMark -> 颜色）
    std::map<int, vcg::Color4b> newMarkColorMap;
    std::srand(static_cast<unsigned>(std::time(nullptr)));
    for (const auto& region : regs)
    {
        unsigned char red = static_cast<unsigned char>(std::rand() % 256);
        unsigned char green = static_cast<unsigned char>(std::rand() % 256);
        unsigned char blue = static_cast<unsigned char>(std::rand() % 256);
        newMarkColorMap[region.newMark] = vcg::Color4b(red, green, blue, 255);
    }

    // 给每个区域的三角形赋面颜色（内存中的面颜色，便于查看）
    for (const auto& region : regs)
    {
        const vcg::Color4b& regionColor = newMarkColorMap[region.newMark];
        for (int faceIndex : region.inTris)
        {
            m_pMesh->face[faceIndex].C() = regionColor;
        }
    }

    // 输出面颜色 OBJ：每个 NewMark 一个 MTL 材质，面级 usemtl，不再输出顶点颜色
    std::string objectPath = m_debugOutputDir + "colored_mesh.obj";
    std::string materialPath = m_debugOutputDir + "colored_mesh.mtl";
    std::ofstream objectStream(objectPath);
    if (!objectStream.is_open())
    {
        return;
    }
    std::ofstream materialStream(materialPath);
    if (!materialStream.is_open())
    {
        return;
    }

    objectStream << "mtllib colored_mesh.mtl\n";
    for (int vertexIndex = 0; vertexIndex < (int)m_pMesh->vert.size(); vertexIndex++)
    {
        const auto& point = m_pMesh->vert[vertexIndex].P();
        objectStream << "v " << point.X() << " " << point.Y() << " " << point.Z() << "\n";
    }

    for (const auto& entry : newMarkColorMap)
    {
        vcg::Color4b color = entry.second;
        materialStream << "newmtl newmark_" << entry.first << "\n";
        materialStream << "Kd " << (color.X() / 255.0f) << " "
            << (color.Y() / 255.0f) << " " << (color.Z() / 255.0f) << "\n";
    }

    for (const auto& region : regs)
    {
        objectStream << "usemtl newmark_" << region.newMark << "\n";
        for (int faceIndex : region.inTris)
        {
            const auto& face = m_pMesh->face[faceIndex];
            objectStream << "f " << (face.V(0)->Index() + 1)
                << " " << (face.V(1)->Index() + 1)
                << " " << (face.V(2)->Index() + 1) << "\n";
        }
    }

    objectStream.close();
    materialStream.close();
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

    // 阶段 1：串行 flood-fill 收集所有区域任务（不切割，只标记区域已访问）。
    struct RegionTask
    {
        int targetMark = 0;
        std::vector<int> curFaces;
        std::vector<MeshCutByMark::Polyline> polylines;
    };
    std::vector<RegionTask> regionTasks;
    std::vector<char> visitedRegion(m_pMesh->face.size(), 0);
    for (int faceIndex = 0; faceIndex < (int)m_pMesh->face.size(); faceIndex++)
    {
        if (m_pMesh->face[faceIndex].IsD() || visitedRegion[faceIndex])
        {
            continue;
        }

        int targetMark = m_pMesh->face[faceIndex].IMark();
        std::vector<int> curFaces =
            m_regionMarker.floodFill(faceIndex, targetMark, m_pMesh, m_edgeInfoManager);
        for (int regionFaceIndex : curFaces)
        {
            visitedRegion[regionFaceIndex] = 1;
        }

        debugWriteFacesOFF(m_debugIterCounter, "cur_faces", curFaces);
        std::vector<MeshCutByMark::CutEdge> cutEdges = findCutEdges(curFaces);
        std::vector<MeshCutByMark::Polyline> polylines =
            m_polylineManager.connectEdgesToPolylines(cutEdges, m_pMesh);
        debugWritePolylines(m_debugIterCounter, polylines);

        regionTasks.push_back({ targetMark, std::move(curFaces), std::move(polylines) });
        m_debugIterCounter++;
    }

    // 阶段 2：局部独立切割（只读全局、写独立局部结果，并行执行）。
    std::vector<MeshCutByMark::LocalCutResult> allLocalResults(regionTasks.size());
    std::vector<std::future<void>> futures;
    for (size_t taskIndex = 0; taskIndex < regionTasks.size(); ++taskIndex)
    {
        futures.push_back(std::async(std::launch::async,
            [this, &regionTasks, &allLocalResults, taskIndex]()
            {
                m_localMeshCut.prepareLocalCut(m_pMesh,
                    regionTasks[taskIndex].curFaces,
                    regionTasks[taskIndex].polylines,
                    regionTasks[taskIndex].targetMark,
                    allLocalResults[taskIndex]);
            }));
    }
    for (auto& future : futures)
    {
        future.get();
    }

    // 阶段 3：串行写回全局并同步 newMark。
    for (auto& localResult : allLocalResults)
    {
        m_localMeshCut.mergeLocalCut(m_pMesh, localResult, m_regionMarker,
            m_newMarkCounter);
    }

    // 统一缝合所有局部单元之间的拼接边：合并两侧切点顶点，对未切侧做纯分割，
    // 消除接缝裂缝；缝合后重算 FF/法向。
    m_localMeshCut.stitchAllSeams(m_pMesh, allLocalResults, m_regionMarker);
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*m_pMesh);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(*m_pMesh);

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
        // 多边形法向取组内三角形法向的累加，保证与内部三角形方向一致。
        vcg::Point3d normalSum(0, 0, 0);
        for (int faceIndex : faces)
        {
            normalSum += m_pMesh->face[faceIndex].N();
        }
        if (normalSum.Norm() > 1e-12)
        {
            normalSum.Normalize();
        }
        else
        {
            normalSum = m_pMesh->face[faces[0]].N();
        }
        region.normal = normalSum;
        region.boundlines = boundaries.empty() ? std::vector<int>() : boundaries[0];
        region.boundaries = boundaries;

        retRegs.push_back(region);
    }

    // 调试输出：带颜色的网格
    debugSaveColoredMesh(retRegs);
}
