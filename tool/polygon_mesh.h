// tool/polygon_mesh.h
// 多边形切割路径的工具集：作用于 jaslmc::PolygonMesh（类型定义在
// external/cgalLocalMeshCut/JasMeshLocalMarkAndCutSplitInternal.h）的
// 分组、精确点包含判定与边界环提取，供 mergeLocalCutPolygon 使用。
#ifndef POLYGON_MESH_H
#define POLYGON_MESH_H

#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include <cmath>

#include "JasMeshLocalMarkAndCutSplitInternal.h"
#include "cmesh.h"

namespace MeshCutByMark
{

// 把 polyResult 的面按「同 mark 且共享边连通」分组：
// 切缝两侧 mark 必不同（CutMeshExact 拆分时标 oldMark / oldMark+1），
// 因此同组内不会跨切缝连通；mark 值碰撞（不同刀产生相同局部 mark）的
// 不连通分片也会被拆开。组序确定：按最小面下标递增。
inline void groupPolygonFacesByMark(const jaslmc::PolygonMesh& poly,
    std::vector<std::vector<int>>& groups)
{
    std::set<int> visited;
    for (int fi = 0; fi < (int)poly.faces.size(); fi++)
    {
        const jaslmc::PolygonFace& face = poly.faces[fi];
        if (!face.isValid() || visited.count(fi))
        {
            continue;
        }
        std::vector<int> group;
        std::vector<int> stack{ fi };
        visited.insert(fi);
        while (!stack.empty())
        {
            int cur = stack.back();
            stack.pop_back();
            group.push_back(cur);
            const jaslmc::PolygonFace& curFace = poly.faces[cur];
            int n = curFace.vertexCount();
            for (int ei = 0; ei < n; ei++)
            {
                int va = curFace.vertexIndices[ei];
                int vb = curFace.vertexIndices[(ei + 1) % n];
                for (int adj : poly.getAdjacentFaces(va, vb))
                {
                    if (adj == cur || visited.count(adj))
                    {
                        continue;
                    }
                    // 只沿「同 mark」的共享边扩散，不跨切缝。
                    if (poly.faces[adj].mark != curFace.mark)
                    {
                        continue;
                    }
                    visited.insert(adj);
                    stack.push_back(adj);
                }
            }
        }
        groups.push_back(std::move(group));
    }
}

// 精确点是否在多边形环内（射线法，EPEC 精确算术）。
// dropAxis：投影时丢弃的轴（取区域法向的主轴），环与点共面。
inline bool pointInPolygonExact(const std::vector<jaslmc::ExactPoint>& loop,
    const jaslmc::ExactPoint& p, int dropAxis)
{
    int axes[2];
    for (int i = 0, k = 0; i < 3; i++)
    {
        if (i != dropAxis)
        {
            axes[k++] = i;
        }
    }
    auto coord = [](const jaslmc::ExactPoint& q, int axis) -> jaslmc::Kernel::FT
    {
        return axis == 0 ? q.x() : (axis == 1 ? q.y() : q.z());
    };
    const jaslmc::Kernel::FT px = coord(p, axes[0]);
    const jaslmc::Kernel::FT py = coord(p, axes[1]);

    int crossings = 0;
    const int n = (int)loop.size();
    for (int i = 0; i < n; i++)
    {
        const jaslmc::ExactPoint& a = loop[i];
        const jaslmc::ExactPoint& b = loop[(i + 1) % n];
        const jaslmc::Kernel::FT ay = coord(a, axes[1]);
        const jaslmc::Kernel::FT by = coord(b, axes[1]);
        if ((ay > py) == (by > py))
        {
            continue;
        }
        const jaslmc::Kernel::FT ax = coord(a, axes[0]);
        const jaslmc::Kernel::FT bx = coord(b, axes[0]);
        const jaslmc::Kernel::FT xint =
            ax + (py - ay) * (bx - ax) / (by - ay);
        if (px < xint)
        {
            crossings++;
        }
    }
    return crossings % 2 == 1;
}

// 提取一个组（polyResult 面子集）的边界环：
// 组内边界有向边 = 该边在「组内」恰有一个邻接面（切缝边在组内只算一次，
// 不会像全局判定那样被两侧抵消）；在 pinch 顶点处按最逆时针规则链接，
// 与 jaslmc::SubRegionBoundary 约定一致。输出环的顶点为 poly 顶点下标，
// 外圈在前（面积最大、相对 normal 逆时针），其余为洞（顺时针）。
inline void polygonGroupBoundaryLoops(const jaslmc::PolygonMesh& poly,
    const std::vector<int>& groupFaces, const vcg::Point3d& normal,
    std::vector<std::vector<int>>& loops)
{
    std::set<int> groupSet(groupFaces.begin(), groupFaces.end());

    // 1. 收集组内边界有向边（方向沿面绕序）。
    struct DirectedEdge { int from, to; };
    std::vector<DirectedEdge> edges;
    for (int fi : groupFaces)
    {
        const jaslmc::PolygonFace& face = poly.faces[fi];
        int n = face.vertexCount();
        for (int ei = 0; ei < n; ei++)
        {
            int va = face.vertexIndices[ei];
            int vb = face.vertexIndices[(ei + 1) % n];
            int inGroup = 0;
            for (int adj : poly.getAdjacentFaces(va, vb))
            {
                if (groupSet.count(adj))
                {
                    inGroup++;
                }
            }
            if (inGroup == 1)
            {
                edges.push_back({ va, vb });
            }
        }
    }
    if (edges.empty())
    {
        return;
    }

    // 2. 每个起点的出边列表。
    std::map<int, std::vector<int>> outgoing;
    for (int i = 0; i < (int)edges.size(); i++)
    {
        outgoing[edges[i].from].push_back(i);
    }

    // 3. 追踪环：每个顶点选最逆时针的未用边（与 SubRegionBoundary 同法）。
    vcg::Point3d n = normal;
    if (n.Norm() < 1e-12)
    {
        n = vcg::Point3d(0, 0, 1);
    }
    else
    {
        n.Normalize();
    }
    std::vector<char> used(edges.size(), 0);
    std::vector<std::vector<int>> rawLoops;
    for (size_t startEdge = 0; startEdge < edges.size(); startEdge++)
    {
        if (used[startEdge])
        {
            continue;
        }
        used[startEdge] = 1;
        std::vector<int> loop{ edges[startEdge].from };
        const int startVertex = edges[startEdge].from;
        int currentEdge = (int)startEdge;
        size_t guard = 0;
        while (guard++ <= edges.size())
        {
            const int from = edges[currentEdge].from;
            const int to = edges[currentEdge].to;
            if (to == startVertex)
            {
                break;  // 环闭合
            }
            loop.push_back(to);
            const vcg::Point3d curPos = vcg::Point3d(
                CGAL::to_double(poly.vertices[to].point.x()),
                CGAL::to_double(poly.vertices[to].point.y()),
                CGAL::to_double(poly.vertices[to].point.z()));
            const vcg::Point3d prevPos = vcg::Point3d(
                CGAL::to_double(poly.vertices[from].point.x()),
                CGAL::to_double(poly.vertices[from].point.y()),
                CGAL::to_double(poly.vertices[from].point.z()));
            const vcg::Point3d inDir = curPos - prevPos;
            int nextEdge = -1;
            double bestAngle = -4.0;  // 小于 -pi，保证任意角都能取到
            for (int candidate : outgoing[to])
            {
                if (used[candidate])
                {
                    continue;
                }
                const int nextVertex = edges[candidate].to;
                if (edges[candidate].from == to && nextVertex == from)
                {
                    continue;  // 反向边
                }
                const vcg::Point3d outDir = vcg::Point3d(
                    CGAL::to_double(poly.vertices[nextVertex].point.x()),
                    CGAL::to_double(poly.vertices[nextVertex].point.y()),
                    CGAL::to_double(poly.vertices[nextVertex].point.z())) - curPos;
                const double angle = std::atan2(
                    n * (inDir ^ outDir), inDir * outDir);
                if (angle > bestAngle)
                {
                    bestAngle = angle;
                    nextEdge = candidate;
                }
            }
            if (nextEdge < 0)
            {
                break;  // 悬挂：环不闭合，按断环输出
            }
            used[nextEdge] = 1;
            currentEdge = nextEdge;
        }
        if (loop.size() >= 3)
        {
            rawLoops.push_back(std::move(loop));
        }
    }

    // 4. 外圈在前（|面积| 最大），外圈相对 normal 逆时针、洞顺时针。
    //    有向面积 = 0.5 * Σ (p_i × p_{i+1}) · n。
    struct LoopInfo { std::vector<int> vertices; double area; };
    std::vector<LoopInfo> infos;
    for (auto& loop : rawLoops)
    {
        vcg::Point3d acc(0, 0, 0);
        const int loopSize = (int)loop.size();
        for (int i = 0; i < loopSize; i++)
        {
            const jaslmc::ExactPoint& pa = poly.vertices[loop[i]].point;
            const jaslmc::ExactPoint& pb = poly.vertices[loop[(i + 1) % loopSize]].point;
            const vcg::Point3d a(CGAL::to_double(pa.x()), CGAL::to_double(pa.y()),
                CGAL::to_double(pa.z()));
            const vcg::Point3d b(CGAL::to_double(pb.x()), CGAL::to_double(pb.y()),
                CGAL::to_double(pb.z()));
            acc += a ^ b;
        }
        const double area = 0.5 * (acc * n);
        if (std::abs(area) < 1e-9)
        {
            continue;  // 退化环
        }
        infos.push_back({ std::move(loop), area });
    }
    std::stable_sort(infos.begin(), infos.end(),
        [](const LoopInfo& lhs, const LoopInfo& rhs)
        {
            return std::abs(lhs.area) > std::abs(rhs.area);
        });
    for (size_t i = 0; i < infos.size(); i++)
    {
        const bool wantCCW = (i == 0);
        if ((infos[i].area > 0) != wantCCW)
        {
            std::reverse(infos[i].vertices.begin(), infos[i].vertices.end());
        }
        loops.push_back(std::move(infos[i].vertices));
    }
}

} // namespace MeshCutByMark

#endif // POLYGON_MESH_H
