#ifndef LOCAL_MESH_CUT_H
#define LOCAL_MESH_CUT_H

#include <array>
#include <map>
#include <set>
#include <vector>

#include "cmesh.h"
#include "polyline.h"
#include "region_marker.h"
#include "cut_mesh.h"
#include "local_cut_result.h"

namespace MeshCutByMark
{

class LocalMeshCutManager
{
public:
    // face 是否包含无向边 (vertexA, vertexB)
    static bool faceHasEdge(CMeshOD* mesh, int faceIndex, int vertexA, int vertexB)
    {
        const auto& face = mesh->face[faceIndex];
        for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
        {
            int edgeVertexA = face.V(edgeIndex)->Index();
            int edgeVertexB = face.V((edgeIndex + 1) % 3)->Index();
            if ((edgeVertexA == vertexA && edgeVertexB == vertexB) ||
                (edgeVertexA == vertexB && edgeVertexB == vertexA))
            {
                return true;
            }
        }
        return false;
    }

    // 把外部面 extG 沿 (ga, gb) 边在 splitVertices（按 a->b 顺序）处一次切成
    // n+1 个子面： (a, v0, c), (v0, v1, c), ..., (v_{n-1}, b, c)，原面删除。
    static void splitExternalFaceMulti(CMeshOD* mesh, int extG, int ga, int gb,
        const std::vector<int>& splitVertices, std::vector<int>& newSubFaces)
    {
        // 先读出所需信息（AddFace 可能 realloc mesh->face，使引用失效）
        int vertexIndices[3] = {
            mesh->face[extG].V(0)->Index(),
            mesh->face[extG].V(1)->Index(),
            mesh->face[extG].V(2)->Index()
        };
        int originalMark = mesh->face[extG].IMark();

        // 按外部面自身绕序定位缝边：V(edgeIndex) -> V(edgeIndex+1) 恰为 {ga, gb}
        int seamEdgeIndex = -1;
        for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
        {
            int edgeVertexA = vertexIndices[edgeIndex];
            int edgeVertexB = vertexIndices[(edgeIndex + 1) % 3];
            if ((edgeVertexA == ga && edgeVertexB == gb) ||
                (edgeVertexA == gb && edgeVertexB == ga))
            {
                seamEdgeIndex = edgeIndex;
                break;
            }
        }
        if (seamEdgeIndex < 0)
        {
            return;
        }

        // 按面的真实绕序取 a -> b -> c，gv 位于边 (a, b) 上，保证新面与原面同向
        int windingVertexA = vertexIndices[seamEdgeIndex];
        int windingVertexB = vertexIndices[(seamEdgeIndex + 1) % 3];
        int thirdVertexIndex = vertexIndices[(seamEdgeIndex + 2) % 3];

        // 分割点按外部面真实绕序 a -> b 重新排序（传入的 splitVertices 顺序
        // 按缝边 key 方向，可能与绕序相反，多个分割点时顺序错误会生成自交面）
        std::vector<std::pair<double, int>> orderedVertices;
        vcg::Point3d windingVectorAB =
            mesh->vert[windingVertexB].P() - mesh->vert[windingVertexA].P();
        for (int splitVertex : splitVertices)
        {
            vcg::Point3d windingVectorAP =
                mesh->vert[splitVertex].P() - mesh->vert[windingVertexA].P();
            double projectionParameter =
                windingVectorAP * windingVectorAB / windingVectorAB.SquaredNorm();
            orderedVertices.push_back({ projectionParameter, splitVertex });
        }
        std::sort(orderedVertices.begin(), orderedVertices.end());

        int previousVertex = windingVertexA;
        for (size_t splitIndex = 0; splitIndex <= orderedVertices.size(); splitIndex++)
        {
            int currentVertex =
                (splitIndex < orderedVertices.size())
                ? orderedVertices[splitIndex].second
                : windingVertexB;
            if (previousVertex == currentVertex)
            {
                continue; // 防御：相邻分割点重合
            }
            vcg::tri::Allocator<CMeshOD>::AddFace(*mesh,
                &mesh->vert[previousVertex], &mesh->vert[currentVertex],
                &mesh->vert[thirdVertexIndex]);
            mesh->face.back().IMark() = originalMark;
            newSubFaces.push_back(static_cast<int>(mesh->face.size()) - 1);
            previousVertex = currentVertex;
        }
        // 原 extG 标记删除（按下标，安全）
        mesh->face[extG].SetD();
    }

    // 统一缝合：聚合所有局部单元的拼接边切点，按坐标把两侧切点合并为同一个
    // 全局顶点，重写全局面引用，再对每条拼接边的两侧外部邻接面做纯分割。
    static void stitchAllSeams(CMeshOD* mesh,
        const std::vector<LocalCutResult>& results,
        RegionMarker& regionMarker)
    {
        // 用精确坐标（ExactPoint）去重，落实「精确坐标缝合」契约，
        // 避免 double 往返导致两侧切点判定为不相等而留下裂缝。
        std::map<jaslmc::ExactPoint, int> pointToVertex;
        std::map<int, int> mergeVertex; // 旧全局顶点 -> 保留的全局顶点
        std::map<std::pair<int, int>, std::vector<int>> seamSplitVertices;
        std::map<std::pair<int, int>, std::set<int>> seamExternalFaces;

        for (const auto& result : results)
        {
            for (const auto& seam : result.seams)
            {
                const std::pair<int, int> seamKey =
                    std::minmax(seam.globalVertexA, seam.globalVertexB);
                for (int externalFaceIndex : seam.externalFaceIndices)
                {
                    if (externalFaceIndex >= 0)
                    {
                        seamExternalFaces[seamKey].insert(externalFaceIndex);
                    }
                }
                for (const auto& point : seam.points)
                {
                    const jaslmc::ExactPoint& coordKey = point.exactPoint;
                    auto iterator = pointToVertex.find(coordKey);
                    if (iterator == pointToVertex.end())
                    {
                        pointToVertex[coordKey] = point.globalVertexIndex;
                        seamSplitVertices[seamKey].push_back(
                            point.globalVertexIndex);
                    }
                    else if (point.globalVertexIndex != iterator->second)
                    {
                        mergeVertex[point.globalVertexIndex] = iterator->second;
                    }
                }
            }
        }

        if (!mergeVertex.empty())
        {
            for (auto& face : mesh->face)
            {
                if (face.IsD())
                {
                    continue;
                }
                for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
                {
                    int vertexIndex = face.V(edgeIndex)->Index();
                    auto iterator = mergeVertex.find(vertexIndex);
                    while (iterator != mergeVertex.end())
                    {
                        vertexIndex = iterator->second;
                        iterator = mergeVertex.find(vertexIndex);
                    }
                    face.V(edgeIndex) = &mesh->vert[vertexIndex];
                }
            }

            // 合并后旧顶点不再被引用；保留 canonical 顶点，标记其余顶点删除，
            // 避免后续流形检查统计到坐标相同的冗余顶点对。
            std::set<int> canonicalVertices;
            for (const auto& entry : mergeVertex)
            {
                canonicalVertices.insert(entry.second);
            }
            for (const auto& entry : mergeVertex)
            {
                if (canonicalVertices.count(entry.first) == 0)
                {
                    mesh->vert[entry.first].SetD();
                }
            }
        }

        for (const auto& entry : seamSplitVertices)
        {
            const std::pair<int, int> seamKey = entry.first;
            std::vector<int> splitVertices;
            std::set<int> seenSplitVertices;
            for (int vertexIndex : entry.second)
            {
                if (seenSplitVertices.insert(vertexIndex).second)
                {
                    splitVertices.push_back(vertexIndex);
                }
            }
            if (splitVertices.empty())
            {
                continue;
            }
            for (int externalFaceIndex : seamExternalFaces[seamKey])
            {
                if (externalFaceIndex < 0 ||
                    externalFaceIndex >= (int)mesh->face.size() ||
                    mesh->face[externalFaceIndex].IsD())
                {
                    continue;
                }
                // 若该邻接面已经被另一侧区域切割，缝边已分段，只靠上面的
                // 顶点合并即可；若仍以 (A,B) 为完整边，说明该侧未被切割，
                // 需要按最终切点做纯分割。
                if (!faceHasEdge(mesh, externalFaceIndex,
                    seamKey.first, seamKey.second))
                {
                    continue;
                }
                std::vector<int> newSubFaces;
                splitExternalFaceMulti(mesh, externalFaceIndex, seamKey.first,
                    seamKey.second, splitVertices, newSubFaces);
                regionMarker.growNewMark(mesh->face.size());
                const int inheritedNewMark =
                    regionMarker.getNewMark(externalFaceIndex);
                for (int subFaceIndex : newSubFaces)
                {
                    regionMarker.setNewMark(subFaceIndex, inheritedNewMark);
                }

                // 该外部面被本次缝边分割后，其全局 face 下标已不再代表完整面；
                // 若其他缝边也引用这个外部面，需要把引用重定向到包含对应缝边的子面，
                // 否则后续缝边会因 faceHasEdge 失败而漏掉纯分割。
                for (auto& otherEntry : seamExternalFaces)
                {
                    if (otherEntry.first == seamKey)
                    {
                        continue;
                    }
                    if (otherEntry.second.erase(externalFaceIndex) == 0)
                    {
                        continue;
                    }
                    for (int subFaceIndex : newSubFaces)
                    {
                        if (faceHasEdge(mesh, subFaceIndex,
                            otherEntry.first.first, otherEntry.first.second))
                        {
                            otherEntry.second.insert(subFaceIndex);
                            break;
                        }
                    }
                }
            }
        }
    }

    // 并行阶段：只做局部提取、批量切割和局部拼接边收集，不写全局 mesh。
    void prepareLocalCut(CMeshOD* mesh, const std::vector<int>& curFaces,
        const std::vector<Polyline>& polylines, int targetMark,
        LocalCutResult& result)
    {
        std::set<int> boundaryVertices;
        for (const auto& polyline : polylines)
        {
            if (polyline.type == CUT_EDGE_NON_MANIFOLD)
            {
                continue;
            }
            for (int vertexIndex : polyline.vertexIndices)
            {
                boundaryVertices.insert(vertexIndex);
            }
        }

        vcg::Box3d box;
        for (int faceIndex : curFaces)
        {
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                box.Add(mesh->face[faceIndex].V(edgeIndex)->P());
            }
        }
        double diagonalLength = box.Diag();
        if (diagonalLength < 1e-9)
        {
            diagonalLength = 1.0;
        }
        vcg::Point3d regionNormal = mesh->face[curFaces.front()].N();
        if (regionNormal.Norm() < 1e-9)
        {
            regionNormal = vcg::Point3d(0, 0, 1);
        }

        std::vector<jaslmc::ExactPoint> normals;
        std::vector<std::vector<jaslmc::ExactPoint>> lines;
        for (const auto& polyline : polylines)
        {
            if (polyline.type != CUT_EDGE_NON_MANIFOLD)
            {
                continue;
            }
            const bool extendStart =
                boundaryVertices.count(polyline.vertexIndices.front()) == 0;
            const bool extendEnd =
                boundaryVertices.count(polyline.vertexIndices.back()) == 0;
            std::vector<jaslmc::ExactPoint> exactLine;
            if (extendStart)
            {
                vcg::Point3d direction =
                    mesh->vert[polyline.vertexIndices[0]].P() -
                    mesh->vert[polyline.vertexIndices[1]].P();
                direction.Normalize();
                vcg::Point3d point =
                    mesh->vert[polyline.vertexIndices.front()].P();
                point += direction * diagonalLength;
                exactLine.push_back(jaslmc::ExactPoint(
                    point.X(), point.Y(), point.Z()));
            }
            for (int vertexIndex : polyline.vertexIndices)
            {
                const auto& point = mesh->vert[vertexIndex].P();
                exactLine.push_back(jaslmc::ExactPoint(
                    point.X(), point.Y(), point.Z()));
            }
            if (extendEnd)
            {
                int vertexCount = (int)polyline.vertexIndices.size();
                vcg::Point3d direction =
                    mesh->vert[polyline.vertexIndices[vertexCount - 1]].P() -
                    mesh->vert[polyline.vertexIndices[vertexCount - 2]].P();
                direction.Normalize();
                vcg::Point3d point =
                    mesh->vert[polyline.vertexIndices.back()].P();
                point += direction * diagonalLength;
                exactLine.push_back(jaslmc::ExactPoint(
                    point.X(), point.Y(), point.Z()));
            }
            normals.push_back(jaslmc::ExactPoint(
                regionNormal.X(), regionNormal.Y(), regionNormal.Z()));
            lines.push_back(std::move(exactLine));
        }

        result.faceGlobals = curFaces;
        result.targetMark = targetMark;
        jaslmc::CutFacesExact(*mesh, curFaces, normals, lines, result.exact);
    }

    // 串行阶段：把并行阶段算好的局部网格写回全局，并把局部拼接边映射为全局。
    // existingPointToVertex：全网格「精确坐标 -> 全局顶点」索引，跨区域复用，
    // 切割线经过已有顶点时复用已有顶点，避免坐标相同、索引不同的重复顶点。
    void mergeLocalCut(CMeshOD* mesh, LocalCutResult& result,
        RegionMarker& regionMarker, int& newMarkCounter,
        std::map<jaslmc::ExactPoint, int>& existingPointToVertex)
    {
        const jaslmc::ExactCutResult& exact = result.exact;

        // 非流形顶点区域保守跳过：不做切割，该区域整体作为一个未切区域赋
        // 一个 newMark，保留原几何与 IMark。
        if (exact.skipped)
        {
            regionMarker.growNewMark(mesh->face.size());
            for (int faceIndex : result.faceGlobals)
            {
                if (faceIndex < 0 ||
                    faceIndex >= (int)mesh->face.size() ||
                    mesh->face[faceIndex].IsD())
                {
                    continue;
                }
                regionMarker.setNewMark(faceIndex, newMarkCounter);
            }
            newMarkCounter++;
            return;
        }

        std::map<jaslmc::ExactMesh::Vertex_index, int> vertex_to_global;
        std::map<jaslmc::ExactPoint, int> new_point_to_index;
        for (auto vertexIndex : exact.mesh.vertices())
        {
            int originalIndex = exact.vertex_global_map[vertexIndex];
            if (originalIndex >= 0)
            {
                vertex_to_global[vertexIndex] = originalIndex;
                continue;
            }
            jaslmc::ExactPoint point = exact.mesh.point(vertexIndex);
            auto existing = existingPointToVertex.find(point);
            if (existing != existingPointToVertex.end())
            {
                vertex_to_global[vertexIndex] = existing->second;
                continue;
            }
            auto duplicate = new_point_to_index.find(point);
            if (duplicate != new_point_to_index.end())
            {
                vertex_to_global[vertexIndex] = duplicate->second;
                continue;
            }
            auto newVertex =
                vcg::tri::Allocator<CMeshOD>::AddVertices(*mesh, 1);
            newVertex->P() = vcg::Point3d(CGAL::to_double(point.x()),
                CGAL::to_double(point.y()), CGAL::to_double(point.z()));
            const int globalIndex = mesh->vn - 1;
            vertex_to_global[vertexIndex] = globalIndex;
            new_point_to_index[point] = globalIndex;
            existingPointToVertex[point] = globalIndex;
        }

        std::vector<int> newFaceGlobals;
        std::map<jaslmc::ExactMesh::Face_index, int> exactFaceToGlobal;
        for (auto faceIndex : exact.mesh.faces())
        {
            std::array<int, 3> globalVertices;
            int corner = 0;
            for (auto vertexIndex :
                CGAL::vertices_around_face(exact.mesh.halfedge(faceIndex),
                    exact.mesh))
            {
                if (corner >= 3)
                {
                    break;
                }
                globalVertices[corner++] =
                    vertex_to_global.at(vertexIndex);
            }
            if (corner != 3)
            {
                continue;
            }
            const int globalFaceIndex = exact.face_global_map[faceIndex];
            if (globalFaceIndex >= 0 &&
                globalFaceIndex < (int)mesh->face.size() &&
                !mesh->face[globalFaceIndex].IsD())
            {
                mesh->face[globalFaceIndex].V(0) =
                    &mesh->vert[globalVertices[0]];
                mesh->face[globalFaceIndex].V(1) =
                    &mesh->vert[globalVertices[1]];
                mesh->face[globalFaceIndex].V(2) =
                    &mesh->vert[globalVertices[2]];
                mesh->face[globalFaceIndex].IMark() = result.targetMark;
                exactFaceToGlobal[faceIndex] = globalFaceIndex;
            }
            else
            {
                auto newFace =
                    vcg::tri::Allocator<CMeshOD>::AddFaces(*mesh, 1);
                newFace->V(0) = &mesh->vert[globalVertices[0]];
                newFace->V(1) = &mesh->vert[globalVertices[1]];
                newFace->V(2) = &mesh->vert[globalVertices[2]];
                newFace->IMark() = result.targetMark;
                exactFaceToGlobal[faceIndex] =
                    (int)mesh->face.size() - 1;
                newFaceGlobals.push_back(exactFaceToGlobal[faceIndex]);
            }
        }

        result.seams.clear();
        for (const auto& seam : exact.seams)
        {
            SeamCutLine seamLine;
            seamLine.globalVertexA = seam.global_vertex_a;
            seamLine.globalVertexB = seam.global_vertex_b;
            seamLine.externalFaceIndices = seam.external_faces;
            for (const auto& point : seam.points)
            {
                auto iterator = vertex_to_global.find(point.local_vertex);
                if (iterator == vertex_to_global.end())
                {
                    continue;
                }
                SeamCutPoint cutPoint;
                cutPoint.globalVertexIndex = iterator->second;
                cutPoint.t = point.t;
                cutPoint.point = vcg::Point3d(
                    CGAL::to_double(point.point.x()),
                    CGAL::to_double(point.point.y()),
                    CGAL::to_double(point.point.z()));
                cutPoint.exactPoint = point.point;
                seamLine.points.push_back(cutPoint);
            }
            result.seams.push_back(std::move(seamLine));
        }

        // 主流程在 stitchAllSeams 之后统一重算 FF/法向；这里只需把
        // newMark 存储扩到当前 face 数量，避免逐区域重复全量拓扑计算。
        regionMarker.growNewMark(mesh->face.size());

        std::map<int, int> localMarkToGlobalMark;
        for (const auto& entry : exactFaceToGlobal)
        {
            const int globalFaceIndex = entry.second;
            if (globalFaceIndex < 0 ||
                globalFaceIndex >= (int)mesh->face.size() ||
                mesh->face[globalFaceIndex].IsD())
            {
                continue;
            }
            const int localMark = exact.face_mark_map[entry.first];
            auto iterator = localMarkToGlobalMark.find(localMark);
            if (iterator == localMarkToGlobalMark.end())
            {
                localMarkToGlobalMark[localMark] = newMarkCounter;
                regionMarker.setNewMark(globalFaceIndex, newMarkCounter);
                newMarkCounter++;
            }
            else
            {
                regionMarker.setNewMark(globalFaceIndex, iterator->second);
            }
        }

        // 被 CGAL 拒绝（非流形边）的面未进入 ExactMesh、未被切割，保留原几何，
        // 但必须分配 newMark，否则会掉进 newMark=0 垃圾区并被 Phase3 合成假区域。
        for (int globalFaceIndex : exact.dropped_faces)
        {
            if (globalFaceIndex < 0 ||
                globalFaceIndex >= (int)mesh->face.size() ||
                mesh->face[globalFaceIndex].IsD())
            {
                continue;
            }
            regionMarker.setNewMark(globalFaceIndex, newMarkCounter++);
        }

        std::vector<int> curFaces;
        for (int faceIndex : result.faceGlobals)
        {
            if (!mesh->face[faceIndex].IsD())
            {
                curFaces.push_back(faceIndex);
            }
        }
        for (int faceIndex : newFaceGlobals)
        {
            curFaces.push_back(faceIndex);
        }
        result.faceGlobals = std::move(curFaces);
    }

    // 并行阶段：多边形切割（独立于 prepareLocalCut）
    void prepareLocalCutPolygon(CMeshOD* mesh, const std::vector<int>& curFaces,
        const std::vector<Polyline>& polylines, int targetMark,
        LocalCutResult& result)
    {
        // 1. 从 curFaces 边界环构建 CGAL 单多边形面 ExactMesh
        jaslmc::ExactMesh exMesh;
        jaslmc::CreateExactMesh(mesh, curFaces, exMesh);

        // 星形顶点检测（仅当面数 >= 3 时）
        if (curFaces.size() >= 3)
        {
            std::map<int, std::vector<int>> vertFaces;
            std::map<std::pair<int,int>, std::vector<int>> edgeFaces;
            for (int fi : curFaces)
            {
                if (mesh->face[fi].IsD()) continue;
                for (int ei = 0; ei < 3; ei++)
                {
                    int va = mesh->face[fi].V(ei)->Index();
                    int vb = mesh->face[fi].V((ei+1)%3)->Index();
                    edgeFaces[std::minmax(va,vb)].push_back(fi);
                    vertFaces[va].push_back(fi);
                    vertFaces[vb].push_back(fi);
                }
            }
            bool hasStarVert = false;
            for (auto& [pivot, faces] : vertFaces)
            {
                std::set<int> faceSet(faces.begin(), faces.end());
                std::set<int> visited;
                int comps = 0;
                for (int sf : faceSet)
                {
                    if (visited.count(sf)) continue;
                    if (++comps > 1) { hasStarVert = true; break; }
                    std::vector<int> stack{sf};
                    visited.insert(sf);
                    while (!stack.empty())
                    {
                        int cur = stack.back(); stack.pop_back();
                        for (int ei = 0; ei < 3; ei++)
                        {
                            int va = mesh->face[cur].V(ei)->Index();
                            int vb = mesh->face[cur].V((ei+1)%3)->Index();
                            if (va != pivot && vb != pivot) continue;
                            for (int adj : edgeFaces[std::minmax(va,vb)])
                            {
                                if (!visited.count(adj) && faceSet.count(adj))
                                { visited.insert(adj); stack.push_back(adj); }
                            }
                        }
                    }
                }
                if (hasStarVert) break;
            }
            if (hasStarVert)
            {
                result.exact.skipped = true;
                result.faceGlobals = curFaces;
                result.targetMark = targetMark;
                return;
            }
        }

        // 2. 收集边界顶点
        std::set<int> boundaryVertices;
        for (const auto& polyline : polylines)
        {
            if (polyline.type == CUT_EDGE_NON_MANIFOLD) continue;
            for (int vi : polyline.vertexIndices) boundaryVertices.insert(vi);
        }

        // 3. 计算延长距离
        vcg::Box3d box;
        for (int fi : curFaces)
            for (int ei = 0; ei < 3; ei++)
                box.Add(mesh->face[fi].V(ei)->P());
        double diag = box.Diag();
        if (diag < 1e-9) diag = 1.0;
        vcg::Point3d regionNormal = mesh->face[curFaces.front()].N();
        if (regionNormal.Norm() < 1e-9) regionNormal = vcg::Point3d(0, 0, 1);

        // 4. 构建切割线
        std::vector<jaslmc::ExactPoint> normals;
        std::vector<std::vector<jaslmc::ExactPoint>> cutLines;
        for (const auto& polyline : polylines)
        {
            if (polyline.type != CUT_EDGE_NON_MANIFOLD) continue;
            std::vector<jaslmc::ExactPoint> exactLine;
            bool extStart = boundaryVertices.count(polyline.vertexIndices.front()) == 0;
            bool extEnd = boundaryVertices.count(polyline.vertexIndices.back()) == 0;
            if (extStart)
            {
                vcg::Point3d dir = mesh->vert[polyline.vertexIndices[0]].P() -
                    mesh->vert[polyline.vertexIndices[1]].P();
                dir.Normalize();
                vcg::Point3d pt = mesh->vert[polyline.vertexIndices.front()].P() + dir * diag;
                exactLine.push_back(jaslmc::ExactPoint(pt.X(), pt.Y(), pt.Z()));
            }
            for (int vi : polyline.vertexIndices)
            {
                const auto& pt = mesh->vert[vi].P();
                exactLine.push_back(jaslmc::ExactPoint(pt.X(), pt.Y(), pt.Z()));
            }
            if (extEnd)
            {
                int n = (int)polyline.vertexIndices.size();
                vcg::Point3d dir = mesh->vert[polyline.vertexIndices[n-1]].P() -
                    mesh->vert[polyline.vertexIndices[n-2]].P();
                dir.Normalize();
                vcg::Point3d pt = mesh->vert[polyline.vertexIndices.back()].P() + dir * diag;
                exactLine.push_back(jaslmc::ExactPoint(pt.X(), pt.Y(), pt.Z()));
            }
            normals.push_back(jaslmc::ExactPoint(regionNormal.X(), regionNormal.Y(), regionNormal.Z()));
            cutLines.push_back(std::move(exactLine));
        }

        // 5. 2D 多边形切割
        result.faceGlobals = curFaces;
        result.targetMark = targetMark;
        jaslmc::CutMeshExact(exMesh, normals, cutLines, result.exact);
    }

    // 从 allLocalResults 构建全局 PolygonMesh，局部转全局，缝合边加点。
    static void buildGlobalPolygonAndStitch(
        CMeshOD* mesh,
        const std::vector<LocalCutResult>& allLocalResults,
        const EdgeInfoManager& edgeInfo,
        jaslmc::PolygonMesh& globalPoly)
    {
        globalPoly.clear();
        std::map<jaslmc::ExactPoint, int> exactToPoly;
        for (int vi = 0; vi < (int)mesh->vert.size(); vi++) {
            if (mesh->vert[vi].IsD()) continue;
            const vcg::Point3d& p = mesh->vert[vi].P();
            jaslmc::ExactPoint ep(p.X(), p.Y(), p.Z());
            if (exactToPoly.find(ep) == exactToPoly.end())
                exactToPoly[ep] = globalPoly.addVertex(ep, vi);
        }
        struct SeamInfo { int polyVA, polyVB; std::vector<jaslmc::ExactPoint> cutPoints; };
        std::map<std::pair<int,int>, SeamInfo> seamEdges;
        for (const auto& lr : allLocalResults) {
            if (lr.exact.skipped) continue;
            const jaslmc::PolygonMesh& lp = lr.exact.polyResult;
            std::map<int,int> localToGlobal;
            for (int lvi = 0; lvi < (int)lp.vertices.size(); lvi++) {
                const jaslmc::PolygonVertex& lv = lp.vertices[lvi];
                jaslmc::ExactPoint ep = lv.point;
                if (lv.globalIndex >= 0) {
                    const vcg::Point3d& gp = mesh->vert[lv.globalIndex].P();
                    ep = jaslmc::ExactPoint(gp.X(), gp.Y(), gp.Z());
                }
                auto it = exactToPoly.find(ep);
                if (it != exactToPoly.end()) localToGlobal[lvi] = it->second;
                else {
                    int gi = (lv.globalIndex >= 0) ? lv.globalIndex : -1;
                    localToGlobal[lvi] = globalPoly.addVertex(ep, gi);
                    exactToPoly[ep] = localToGlobal[lvi];
                }
            }
            for (int lfi = 0; lfi < (int)lp.faces.size(); lfi++) {
                const jaslmc::PolygonFace& lf = lp.faces[lfi];
                if (!lf.isValid()) continue;
                std::vector<int> gvis;
                for (int lvi : lf.vertexIndices) {
                    auto it = localToGlobal.find(lvi);
                    if (it != localToGlobal.end()) gvis.push_back(it->second);
                }
                if (gvis.size() < 3) continue;
                globalPoly.addFace(gvis, lf.mark);
                int nv = (int)gvis.size();
                for (int ei = 0; ei < nv; ei++) {
                    int va = gvis[ei], vb = gvis[(ei+1) % nv];
                    auto adj = globalPoly.getAdjacentFaces(va, vb);
                    if (adj.size() >= 2) {
                        auto key = std::make_pair(std::min(va,vb), std::max(va,vb));
                        if (seamEdges.find(key) == seamEdges.end()) {
                            SeamInfo s; s.polyVA = std::min(va,vb); s.polyVB = std::max(va,vb);
                            seamEdges[key] = s;
                        }
                    }
                }
            }
            for (const auto& seam : lr.exact.seams) {
                const vcg::Point3d& pa = mesh->vert[seam.global_vertex_a].P();
                const vcg::Point3d& pb = mesh->vert[seam.global_vertex_b].P();
                jaslmc::ExactPoint epa(pa.X(), pa.Y(), pa.Z());
                jaslmc::ExactPoint epb(pb.X(), pb.Y(), pb.Z());
                auto itA = exactToPoly.find(epa); auto itB = exactToPoly.find(epb);
                if (itA == exactToPoly.end() || itB == exactToPoly.end()) continue;
                auto key = std::make_pair(std::min(itA->second, itB->second), std::max(itA->second, itB->second));
                auto si = seamEdges.find(key);
                if (si != seamEdges.end())
                    for (const auto& pt : seam.points) si->second.cutPoints.push_back(pt.point);
            }
        }
        for (auto& [key, seam] : seamEdges) {
            if (seam.cutPoints.empty()) continue;
            std::sort(seam.cutPoints.begin(), seam.cutPoints.end());
            seam.cutPoints.erase(std::unique(seam.cutPoints.begin(), seam.cutPoints.end()), seam.cutPoints.end());
            const auto& epA = globalPoly.vertices[seam.polyVA].point;
            const auto& epB = globalPoly.vertices[seam.polyVB].point;
            auto dirAB = epB - epA;
            double lenSq = CGAL::to_double(dirAB.squared_length());
            if (lenSq < 1e-20) continue;
            std::vector<std::pair<double, jaslmc::ExactPoint>> ordered;
            for (const auto& pt : seam.cutPoints) {
                double t = CGAL::to_double((pt - epA) * dirAB) / lenSq;
                ordered.push_back({t, pt});
            }
            std::sort(ordered.begin(), ordered.end());
            int curA = seam.polyVA;
            for (const auto& [t, pt] : ordered) {
                if (pt == globalPoly.vertices[curA].point) continue;
                if (pt == globalPoly.vertices[seam.polyVB].point) continue;
                curA = globalPoly.splitEdge(curA, seam.polyVB, pt);
            }
        }
    }
};

} // namespace MeshCutByMark

#endif // LOCAL_MESH_CUT_H
