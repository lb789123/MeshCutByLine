#include "JasMeshMarkAndCutSplit.h"
#include "JasMeshLocalMarkAndCutSplitInternal.h"
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <filesystem>
#include <fstream>
#include <set>
#include <map>
#include <utility>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <cmath>

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

namespace
{

// 精确判定点 p 是否严格位于线段 (a,b) 内部（不含端点），并输出沿线参数 t。
bool pointOnSegmentStrictExact(const jaslmc::ExactPoint& p,
    const jaslmc::ExactPoint& a, const jaslmc::ExactPoint& b,
    jaslmc::Kernel::FT& tOut)
{
    const jaslmc::Kernel::Vector_3 ab = b - a;
    const jaslmc::Kernel::Vector_3 ap = p - a;
    const jaslmc::Kernel::Vector_3 zero(0, 0, 0);
    if (CGAL::cross_product(ab, ap) != zero)
    {
        return false; // 不共线
    }
    const jaslmc::Kernel::FT denom = ab * ab;
    if (denom == 0)
    {
        return false; // 退化线段
    }
    const jaslmc::Kernel::FT t = (ab * ap) / denom;
    if (t <= 0 || t >= 1)
    {
        return false; // 不在线段内部
    }
    tOut = t;
    return true;
}

// 两个精确点之间的距离平方（double 近似，仅用于近重合判定容差）。
double exactPointDistanceSquared(const jaslmc::ExactPoint& a,
    const jaslmc::ExactPoint& b)
{
    const double dx = CGAL::to_double(a.x() - b.x());
    const double dy = CGAL::to_double(a.y() - b.y());
    const double dz = CGAL::to_double(a.z() - b.z());
    return dx * dx + dy * dy + dz * dz;
}

// 规范切点：全局下标 + 精确坐标 + double 近似（粗筛用）。
struct SharedCutPoint
{
    int vertex = -1;
    jaslmc::ExactPoint exact;
    vcg::Point3d approx;
};

// 跨区域切点共形（多边形路径的边界补丁，Phase 3 输出前调用）：
// 多边形路径各区域独立切割、独立合并，切点只出现在「拥有该切割线的
// 区域」的边界环上 —— 与邻域共享的 mark-diff 边另一侧不细分（T 形结）；
// 两侧独立计算的切点又因插值参数为 double、延长量按区域包围盒各自计算，
// 精确坐标并不相等，精确相等去重无法合并。本 pass 三步修复：
//   1. 定位：多边形路径的切点孤立顶点（orphanCutPoints）匹配到其所属
//      区域的边界边，旧路径缝点直接登记缝边，得到「边 -> 切点集」；
//   2. 规范化：同一条边上近重合（距离平方 < 1e-12，与 CutMeshExact 的
//      重合判定同容差）的切点合并为下标最小的顶点，其余重定向，未被
//      面引用的重复孤立顶点标记删除；
//   3. splice：把规范切点按精确参数顺序插入所有经过其所在线段（整边
//      或已细分子段）的输出边界环 —— polyLoops 与 SubRegionBoundary
//      两种来源一并处理，邻接区域由此共享同一顶点细分序列。
void conformSharedEdgeCutPoints(CMeshOD* mesh,
    const std::vector<MeshCutByMark::LocalCutResult>& results,
    const std::map<jaslmc::ExactPoint, int>& existingPointToVertex,
    std::vector<JasMeshMarkAndCutSplit::splitReg>& retRegs)
{
    // 全局顶点 -> 精确坐标（原顶点与切点顶点在合并阶段都登记过）。
    std::map<int, jaslmc::ExactPoint> vertexExact;
    for (const auto& entry : existingPointToVertex)
    {
        vertexExact.emplace(entry.second, entry.first);
    }

    // 被面引用的顶点集合：区分可安全删除的孤立切点与缝点顶点。
    std::set<int> faceVertices;
    for (const auto& face : mesh->face)
    {
        if (face.IsD())
        {
            continue;
        }
        for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
        {
            faceVertices.insert(face.V(edgeIndex)->Index());
        }
    }

    // 1. 定位：边 -> （顶点下标 -> 精确坐标）。
    std::map<std::pair<int, int>, std::map<int, jaslmc::ExactPoint>> edgeCutPoints;
    for (const auto& result : results)
    {
        if (result.usePolygonPath)
        {
            // CutMeshExact 的交点全部来自区域多边形边界，切点只可能落在
            // 该区域的边界边（面集中恰有 1 个面引用）上。
            std::map<std::pair<int, int>, int> edgeCount;
            for (int faceIndex : result.faceGlobals)
            {
                if (faceIndex < 0 || faceIndex >= (int)mesh->face.size() ||
                    mesh->face[faceIndex].IsD())
                {
                    continue;
                }
                for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
                {
                    const int va = mesh->face[faceIndex].V(edgeIndex)->Index();
                    const int vb =
                        mesh->face[faceIndex].V((edgeIndex + 1) % 3)->Index();
                    edgeCount[std::minmax(va, vb)]++;
                }
            }
            std::vector<std::pair<int, int>> boundaryEdges;
            for (const auto& entry : edgeCount)
            {
                if (entry.second == 1)
                {
                    boundaryEdges.push_back(entry.first);
                }
            }
            for (const auto& orphan : result.orphanCutPoints)
            {
                for (const auto& edge : boundaryEdges)
                {
                    auto aIt = vertexExact.find(edge.first);
                    auto bIt = vertexExact.find(edge.second);
                    if (aIt == vertexExact.end() || bIt == vertexExact.end())
                    {
                        continue;
                    }
                    jaslmc::Kernel::FT t;
                    if (pointOnSegmentStrictExact(orphan.second, aIt->second,
                        bIt->second, t))
                    {
                        edgeCutPoints[edge][orphan.first] = orphan.second;
                        break;
                    }
                }
            }
        }
        else
        {
            for (const auto& seam : result.seams)
            {
                for (const auto& point : seam.points)
                {
                    auto pointIt = vertexExact.find(point.globalVertexIndex);
                    if (pointIt == vertexExact.end())
                    {
                        continue;
                    }
                    edgeCutPoints[std::minmax(seam.globalVertexA,
                        seam.globalVertexB)][point.globalVertexIndex] =
                        pointIt->second;
                }
            }
        }
    }
    if (edgeCutPoints.empty())
    {
        return;
    }

    // 2. 规范化：同边近重合切点保留下标最小者（map 升序遍历天然满足），
    //    其余重定向到保留者；未被面引用的重复孤立顶点标记删除。
    std::map<int, int> vertexRedirect;
    std::vector<SharedCutPoint> canonicalPoints;
    for (auto& entry : edgeCutPoints)
    {
        std::map<int, jaslmc::ExactPoint> canonical;
        for (const auto& point : entry.second)
        {
            int target = -1;
            for (const auto& kept : canonical)
            {
                if (exactPointDistanceSquared(kept.second, point.second) < 1e-12)
                {
                    target = kept.first;
                    break;
                }
            }
            if (target < 0)
            {
                canonical[point.first] = point.second;
            }
            else
            {
                vertexRedirect[point.first] = target;
            }
        }
        for (const auto& kept : canonical)
        {
            SharedCutPoint cutPoint;
            cutPoint.vertex = kept.first;
            cutPoint.exact = kept.second;
            cutPoint.approx = vcg::Point3d(CGAL::to_double(kept.second.x()),
                CGAL::to_double(kept.second.y()),
                CGAL::to_double(kept.second.z()));
            canonicalPoints.push_back(cutPoint);
        }
    }
    for (const auto& redirect : vertexRedirect)
    {
        if (faceVertices.count(redirect.first) == 0)
        {
            mesh->vert[redirect.first].SetD();
        }
    }

    // 3. splice：按精确参数把规范切点插入经过其所在子段的环。
    std::map<int, vcg::Point3d> vertexApprox;
    auto approxOf = [&](int vertexIndex) -> const vcg::Point3d&
    {
        auto it = vertexApprox.find(vertexIndex);
        if (it == vertexApprox.end())
        {
            auto exactIt = vertexExact.find(vertexIndex);
            const vcg::Point3d p = (exactIt != vertexExact.end())
                ? vcg::Point3d(CGAL::to_double(exactIt->second.x()),
                    CGAL::to_double(exactIt->second.y()),
                    CGAL::to_double(exactIt->second.z()))
                : mesh->vert[vertexIndex].P();
            it = vertexApprox.emplace(vertexIndex, p).first;
        }
        return it->second;
    };
    auto redirectOf = [&](int vertexIndex)
    {
        auto it = vertexRedirect.find(vertexIndex);
        return (it == vertexRedirect.end()) ? vertexIndex : it->second;
    };
    for (auto& region : retRegs)
    {
        for (auto& loop : region.boundaries)
        {
            const int loopSize = (int)loop.size();
            if (loopSize < 2)
            {
                continue;
            }
            std::vector<int> rebuilt;
            rebuilt.reserve(loopSize + 1);
            bool changed = false;
            for (int i = 0; i < loopSize; i++)
            {
                const int p = redirectOf(loop[i]);
                const int q = redirectOf(loop[(i + 1) % loopSize]);
                rebuilt.push_back(p);
                if (p == q)
                {
                    changed = true; // 重定向后相邻同点，折叠
                    continue;
                }
                auto aIt = vertexExact.find(p);
                auto bIt = vertexExact.find(q);
                if (aIt == vertexExact.end() || bIt == vertexExact.end())
                {
                    continue;
                }
                // double 包围盒粗筛，避免对无关切点做精确谓词
                const vcg::Point3d& pa = approxOf(p);
                const vcg::Point3d& pb = approxOf(q);
                const double margin = 1e-6 * (1.0 +
                    std::max(std::max(std::abs(pa.X()), std::abs(pa.Y())),
                        std::max(std::abs(pb.X()), std::abs(pb.Y()))));
                std::vector<std::pair<jaslmc::Kernel::FT, int>> inserted;
                for (const auto& cutPoint : canonicalPoints)
                {
                    if (cutPoint.vertex == p || cutPoint.vertex == q)
                    {
                        continue;
                    }
                    const vcg::Point3d& pc = cutPoint.approx;
                    if (pc.X() < std::min(pa.X(), pb.X()) - margin ||
                        pc.X() > std::max(pa.X(), pb.X()) + margin ||
                        pc.Y() < std::min(pa.Y(), pb.Y()) - margin ||
                        pc.Y() > std::max(pa.Y(), pb.Y()) + margin ||
                        pc.Z() < std::min(pa.Z(), pb.Z()) - margin ||
                        pc.Z() > std::max(pa.Z(), pb.Z()) + margin)
                    {
                        continue;
                    }
                    jaslmc::Kernel::FT t;
                    if (pointOnSegmentStrictExact(cutPoint.exact, aIt->second,
                        bIt->second, t))
                    {
                        inserted.push_back({ t, cutPoint.vertex });
                    }
                }
                if (inserted.empty())
                {
                    continue;
                }
                changed = true;
                std::sort(inserted.begin(), inserted.end());
                for (const auto& tv : inserted)
                {
                    rebuilt.push_back(tv.second);
                }
            }
            if (changed)
            {
                // 折叠重定向可能产生的相邻重复顶点与首尾重复
                std::vector<int> deduped;
                for (int v : rebuilt)
                {
                    if (deduped.empty() || deduped.back() != v)
                    {
                        deduped.push_back(v);
                    }
                }
                while (deduped.size() > 1 && deduped.front() == deduped.back())
                {
                    deduped.pop_back();
                }
                loop = std::move(deduped);
            }
        }
        region.boundlines = region.boundaries.empty()
            ? std::vector<int>()
            : region.boundaries[0];
    }
}

} // namespace

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
    const int taskCount = static_cast<int>(regionTasks.size());

    // 全局星形顶点检测：如果存在非流形顶点，跳过多边形切割流程
    bool hasGlobalStarVert = false;
    {
        std::map<int, std::vector<int>> vertFaces;
        std::map<std::pair<int,int>, std::vector<int>> edgeFaces;
        for (int fi = 0; fi < (int)m_pMesh->face.size(); fi++)
        {
            if (m_pMesh->face[fi].IsD()) continue;
            for (int ei = 0; ei < 3; ei++)
            {
                int va = m_pMesh->face[fi].V(ei)->Index();
                int vb = m_pMesh->face[fi].V((ei+1)%3)->Index();
                edgeFaces[std::minmax(va,vb)].push_back(fi);
                vertFaces[va].push_back(fi);
                vertFaces[vb].push_back(fi);
            }
        }
        for (auto& [pivot, faces] : vertFaces)
        {
            std::set<int> faceSet(faces.begin(), faces.end());
            std::set<int> visited;
            int comps = 0;
            for (int sf : faceSet)
            {
                if (visited.count(sf)) continue;
                if (++comps > 1) { hasGlobalStarVert = true; break; }
                std::vector<int> stack{sf};
                visited.insert(sf);
                while (!stack.empty())
                {
                    int cur = stack.back(); stack.pop_back();
                    for (int ei = 0; ei < 3; ei++)
                    {
                        int va = m_pMesh->face[cur].V(ei)->Index();
                        int vb = m_pMesh->face[cur].V((ei+1)%3)->Index();
                        if (va != pivot && vb != pivot) continue;
                        for (int adj : edgeFaces[std::minmax(va,vb)])
                        {
                            if (!visited.count(adj) && faceSet.count(adj))
                            { visited.insert(adj); stack.push_back(adj); }
                        }
                    }
                }
            }
            if (hasGlobalStarVert) break;
        }
    }

    // 切割路径路由：由外部参数 SetCutPathMode 选定（三角形 / 多边形），
    // 两条路径最终都输出 retRegs（splitReg），仅切割策略不同：多边形路径
    // 不切分原始三角形（质心近似归属），三角形路径真实切割写回全局网格。
    // 多边形路径保留两个正确性回退（命中时该区域走三角形路径）：
    //   1. 全局星形顶点：非流形顶点区域保守处理；
    //   2. 区域边界多环（带洞）：CreateExactMesh 只追踪一个边界环，会把洞填上。
    const bool polygonMode = (m_cutPathMode == CUT_PATH_POLYGON);
    std::vector<char> usePolygonPath(taskCount, 0);
    for (int taskIndex = 0; taskIndex < taskCount; ++taskIndex)
    {
        usePolygonPath[taskIndex] =
            (polygonMode && !hasGlobalStarVert &&
             extractBoundaryEdges(regionTasks[taskIndex].curFaces).size() == 1)
                ? 1
                : 0;
    }

    #pragma omp parallel for
    for (int taskIndex = 0; taskIndex < taskCount; ++taskIndex)
    {
        allLocalResults[taskIndex].usePolygonPath = usePolygonPath[taskIndex] != 0;
        if (usePolygonPath[taskIndex])
        {
            m_localMeshCut.prepareLocalCutPolygon(m_pMesh,
                regionTasks[taskIndex].curFaces,
                regionTasks[taskIndex].polylines,
                regionTasks[taskIndex].targetMark,
                allLocalResults[taskIndex]);
        }
        else
        {
            m_localMeshCut.prepareLocalCut(m_pMesh,
                regionTasks[taskIndex].curFaces,
                regionTasks[taskIndex].polylines,
                regionTasks[taskIndex].targetMark,
                allLocalResults[taskIndex]);
        }
    }

    // 阶段 3：串行写回全局并同步 newMark。
    // 先构建「精确坐标 -> 全局顶点」索引，跨区域复用，避免切割线经过已有顶点
    // 时追加坐标相同的重复顶点；mergeLocalCut 每追加新顶点会同步更新该索引。
    std::map<jaslmc::ExactPoint, int> existingPointToVertex;
    for (int vertexIndex = 0; vertexIndex < (int)m_pMesh->vert.size(); vertexIndex++)
    {
        if (m_pMesh->vert[vertexIndex].IsD())
        {
            continue;
        }
        const vcg::Point3d& point = m_pMesh->vert[vertexIndex].P();
        existingPointToVertex[jaslmc::ExactPoint(point.X(), point.Y(), point.Z())] =
            vertexIndex;
    }
    // 串行合并：多边形路径走独立合并（不改全局面、无接缝），旧路径照旧
    // 写回全局网格；两者共用「精确坐标 -> 全局顶点」索引，切点顶点跨路径去重。
    bool anyLegacyResult = false;
    for (auto& localResult : allLocalResults)
    {
        if (localResult.usePolygonPath)
        {
            m_localMeshCut.mergeLocalCutPolygon(m_pMesh, localResult,
                m_regionMarker, m_newMarkCounter, existingPointToVertex);
        }
        else
        {
            anyLegacyResult = true;
            m_localMeshCut.mergeLocalCut(m_pMesh, localResult, m_regionMarker,
                m_newMarkCounter, existingPointToVertex);
        }
    }

    // 旧路径需要缝合拼接边消除接缝裂缝；多边形路径不产生接缝，跳过。
    // 缝合后重算 FF/法向（多边形路径网格面未动，重算无害）。
    if (anyLegacyResult)
    {
        m_localMeshCut.stitchAllSeams(m_pMesh, allLocalResults, m_regionMarker);
    }
    vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*m_pMesh);
    vcg::tri::UpdateNormal<CMeshOD>::PerFace(*m_pMesh);

    // 汇总多边形路径记录的精确边界环（newMark -> 环，全局顶点下标）。
    std::map<int, std::vector<std::vector<int>>> newMarkLoops;
    for (const auto& localResult : allLocalResults)
    {
        for (const auto& entry : localResult.polyLoops)
        {
            newMarkLoops[entry.first] = entry.second;
        }
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

    // 防御：任何 newMark==0 的面（理论上已被 mergeLocalCut 的 dropped_faces 护栏
    // 消除）都单独分配递增 newMark，绝不合并成一个假区域。
    auto orphanIt = markToFaces.find(0);
    if (orphanIt != markToFaces.end())
    {
        std::vector<int> orphanFaces = orphanIt->second;
        markToFaces.erase(orphanIt);
        for (int faceIndex : orphanFaces)
        {
            m_regionMarker.setNewMark(faceIndex, m_newMarkCounter);
            markToFaces[m_newMarkCounter].push_back(faceIndex);
            m_newMarkCounter++;
        }
    }

    // 调试输出：最终多边形
    debugWritePolygonsOBJ(markToFaces);

    // 输出结果
    for (const auto& [newMark, faces] : markToFaces)
    {
        // 边界环（外圈 + 洞）：多边形路径优先用合并阶段记录的精确环
        // （含切点孤立顶点，全局顶点下标）；旧路径或未记录时在全局网格上
        // 提取（由外部库 cgalLocalMeshCut 提供）。
        std::vector<std::vector<int>> boundaries;
        auto loopsIt = newMarkLoops.find(newMark);
        if (loopsIt != newMarkLoops.end())
        {
            boundaries = loopsIt->second;
        }
        else
        {
            boundaries = jaslmc::SubRegionBoundary(*m_pMesh, faces);
        }

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

    // 跨区域切点共形：多边形路径各区域独立切割，切点只出现在切割线
    // 所属区域的环上；把共享边上的切点统一 splice 进所有输出环，消除
    // 邻接区域边界的 T 形结与两侧近重合切点。
    conformSharedEdgeCutPoints(m_pMesh, allLocalResults,
        existingPointToVertex, retRegs);

    // 调试输出：带颜色的网格
    debugSaveColoredMesh(retRegs);
}
