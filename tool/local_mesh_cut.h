// tool/local_mesh_cut.h
#ifndef LOCAL_MESH_CUT_H
#define LOCAL_MESH_CUT_H

#include <vector>
#include <map>
#include <unordered_map>
#include <set>
#include <utility>
#include <algorithm>
#include "cmesh.h"
#include "polyline.h"
#include "region_marker.h"
#include "cut_mesh.h"
#include "cut_plane.h"
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>

namespace MeshCutByMark
{

    class LocalMeshCutManager
    {
    public:
        // 局部 mesh + 各种映射
        struct LocalMesh
        {
            CMeshOD mesh;
            int Nv0 = 0;                              // 原顶点数（新顶点 local 下标 >= Nv0）
            std::vector<int> localToGlobalVert;       // localVertIdx -> globalVertIdx（仅 < Nv0）
            std::vector<int> localFaceToGlobal;       // localFaceIdx -> globalFaceIdx（仅原始面）
            // 边界缝：local 原始边的两个 local 顶点 -> 外部邻接面 global idx
            std::map<std::pair<int, int>, int> seamExternal;
        };

        // 步骤 A：从 m_pMesh 的 curFaces 提取局部 mesh
        // 提取局部 mesh：结果写入输出参数 localMesh，避免返回结构体造成拷贝。
        void extractLocalMesh(CMeshOD *mesh, const std::vector<int> &curFaces,
                              LocalMesh &localMesh);

        struct CutInput
        {
            std::vector<vcg::Point3d> line;
            vcg::Point3d normal;
        };

        // 步骤 B：构造延长段 line + 区域 normal
        CutInput buildCutInput(
            const Polyline &polyline,
            bool isStart,
            const LocalMesh &lm,
            CMeshOD *mesh)
        {
            // Build an extended cut line and region normal from a polyline endpoint
            CutInput cutInput;
            const auto &vertexIndices = polyline.vertexIndices;
            int endpointVertexIndex = isStart ? vertexIndices.front() : vertexIndices.back();

            vcg::Point3d endpointPoint = mesh->vert[endpointVertexIndex].P();
            vcg::Point3d cutDirection;
            if (isStart)
            {
                cutDirection = mesh->vert[vertexIndices[0]].P() - mesh->vert[vertexIndices[1]].P();
            }
            else
            {
                int vertexCount = (int)vertexIndices.size();
                cutDirection = mesh->vert[vertexIndices[vertexCount - 1]].P() - mesh->vert[vertexIndices[vertexCount - 2]].P();
            }
            cutDirection.Normalize();

            // L = localMesh 包围盒对角线
            vcg::Box3d box;
            for (int vertexIndex = 0; vertexIndex < (int)lm.mesh.vert.size(); vertexIndex++)
            {
                box.Add(lm.mesh.vert[vertexIndex].P());
            }
            double diagonalLength = box.Diag();
            if (diagonalLength < 1e-9)
            {
                diagonalLength = 1.0;
            }

            cutInput.line.push_back(endpointPoint);
            cutInput.line.push_back(endpointPoint + cutDirection * diagonalLength);

            // normal = 区域法向（取第一个 curFaces 面法向）
            cutInput.normal = mesh->face[lm.localFaceToGlobal[0]].N();
            if (cutInput.normal.Norm() < 1e-9)
            {
                cutInput.normal = vcg::Point3d(0, 0, 1);
            }
            return cutInput;
        }

        // 步骤 D：把 cutLines 上的边标成边界（FFp 自指）
        void markCutEdges(CMeshOD *mesh, const std::vector<std::vector<int>> &cutLines)
        {
            // Make both sides of each cut line edge point to itself in FF adjacency

            // 收集所有要标记的 global 顶点对
            std::set<std::pair<int, int>> edgeSet;
            for (const auto &cutLine : cutLines)
            {
                for (size_t vertexIndex = 0; vertexIndex + 1 < cutLine.size(); vertexIndex++)
                {
                    edgeSet.insert(std::minmax(cutLine[vertexIndex], cutLine[vertexIndex + 1]));
                }
            }
            if (edgeSet.empty())
            {
                return;
            }

            // 遍历面边，命中则两侧 FFp 自指
            for (int faceIndex = 0; faceIndex < (int)mesh->face.size(); faceIndex++)
            {
                if (mesh->face[faceIndex].IsD())
                {
                    continue;
                }
                for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
                {
                    int vertexA = mesh->face[faceIndex].V(edgeIndex)->Index();
                    int vertexB = mesh->face[faceIndex].V((edgeIndex + 1) % 3)->Index();
                    if (edgeSet.count(std::minmax(vertexA, vertexB)))
                    {
                        CFaceOD *adjacentFace = mesh->face[faceIndex].FFp(edgeIndex);
                        mesh->face[faceIndex].FFp(edgeIndex) = &mesh->face[faceIndex];
                        mesh->face[faceIndex].FFi(edgeIndex) = edgeIndex;
                        if (adjacentFace != nullptr && adjacentFace != &mesh->face[faceIndex])
                        {
                            // 断开对面的同一条边
                            for (int adjacentEdgeIndex = 0; adjacentEdgeIndex < 3; adjacentEdgeIndex++)
                            {
                                if (adjacentFace->FFp(adjacentEdgeIndex) == &mesh->face[faceIndex])
                                {
                                    adjacentFace->FFp(adjacentEdgeIndex) = adjacentFace;
                                    adjacentFace->FFi(adjacentEdgeIndex) = adjacentEdgeIndex;
                                }
                            }
                        }
                    }
                }
            }
        }

        // 步骤 C 的返回：merge 回主网格后给上层用的映射
        struct MergeResult
        {
            std::vector<int> newFaceGlobals;    // append 进 mesh 的额外分片 global 下标
            std::vector<int> vertLocalToGlobal; // localVertIdx -> globalVertIdx（含新顶点）
        };

        // 判断 local 面是否引用新顶点（local 下标 >= Nv0）
        static bool faceHasNewVert(const LocalMesh &lm, int localFaceIdx)
        {
            // Return true when any vertex of the face is a newly added local vertex
            const CFaceOD &face = lm.mesh.face[localFaceIdx];
            for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
            {
                int localVertexIndex = static_cast<int>(face.V(vertexIndex) - &lm.mesh.vert[0]);
                if (localVertexIndex >= lm.Nv0)
                {
                    return true;
                }
            }
            return false;
        }

        // 步骤 C：把 cutter 切过的 local mesh merge 回主网格。
        // 语义（与 cutter 契约一致）：被切开的原始面槽位被 cutter 原位重写
        // （不 SetD），额外分片 append 在 local 末尾；merge 时：
        //   - local 面 i < Nf0 且引用新顶点 → 原位改写对应 global 面（localFaceToGlobal[i]）
        //   - local 面 i >= Nf0（额外分片）→ append 新 global 面
        //   - 未动原始面（无新顶点）→ 保持
        // 拓扑保持连续：全程不 SetD。
        MergeResult mergeBack(CMeshOD *mesh, LocalMesh &lm, int targetMark)
        {
            // Append new vertices and rewrite/append faces touched by the cutter
            MergeResult result;

            // 1) append 所有新顶点（local >= Nv0）到 *mesh*，一次性批量加（避免多次 realloc）
            int newVertexCount = static_cast<int>(lm.mesh.vert.size()) - lm.Nv0;
            int firstNewGlobalVertex = static_cast<int>(mesh->vert.size());
            if (newVertexCount > 0)
            {
                vcg::tri::Allocator<CMeshOD>::AddVertices(*mesh, newVertexCount);
            }
            result.vertLocalToGlobal = lm.localToGlobalVert; // < Nv0 部分
            for (int newVertexIndex = 0; newVertexIndex < newVertexCount; newVertexIndex++)
            {
                mesh->vert[firstNewGlobalVertex + newVertexIndex].P() = lm.mesh.vert[lm.Nv0 + newVertexIndex].P();
                result.vertLocalToGlobal.push_back(firstNewGlobalVertex + newVertexIndex);
            }

            // 2) 遍历 local 面
            int numOriginFaces = static_cast<int>(lm.localFaceToGlobal.size());
            for (int localFaceIndex = 0; localFaceIndex < (int)lm.mesh.face.size(); localFaceIndex++)
            {
                if (lm.mesh.face[localFaceIndex].IsD())
                {
                    continue;
                }
                if (!faceHasNewVert(lm, localFaceIndex))
                {
                    continue; // 未动原始面，保持
                }

                int globalVertexA = result.vertLocalToGlobal[static_cast<int>(lm.mesh.face[localFaceIndex].V(0) - &lm.mesh.vert[0])];
                int globalVertexB = result.vertLocalToGlobal[static_cast<int>(lm.mesh.face[localFaceIndex].V(1) - &lm.mesh.vert[0])];
                int globalVertexC = result.vertLocalToGlobal[static_cast<int>(lm.mesh.face[localFaceIndex].V(2) - &lm.mesh.vert[0])];

                if (localFaceIndex < numOriginFaces)
                {
                    // 被切开原始面槽位已被 cutter 原位重写：改写对应 global 面（不 SetD）
                    int globalFaceIndex = lm.localFaceToGlobal[localFaceIndex];
                    if (globalFaceIndex >= 0 && globalFaceIndex < (int)mesh->face.size() && !mesh->face[globalFaceIndex].IsD())
                    {
                        mesh->face[globalFaceIndex].V(0) = &mesh->vert[globalVertexA];
                        mesh->face[globalFaceIndex].V(1) = &mesh->vert[globalVertexB];
                        mesh->face[globalFaceIndex].V(2) = &mesh->vert[globalVertexC];
                        mesh->face[globalFaceIndex].IMark() = targetMark;
                    }
                    else
                    {
                        // 防御：原面缺失/已删除则 append
                        vcg::tri::Allocator<CMeshOD>::AddFace(
                            *mesh, &mesh->vert[globalVertexA], &mesh->vert[globalVertexB], &mesh->vert[globalVertexC]);
                        mesh->face.back().IMark() = targetMark;
                        result.newFaceGlobals.push_back(static_cast<int>(mesh->face.size()) - 1);
                    }
                }
                else
                {
                    // 额外分片：append
                    vcg::tri::Allocator<CMeshOD>::AddFace(
                        *mesh, &mesh->vert[globalVertexA], &mesh->vert[globalVertexB], &mesh->vert[globalVertexC]);
                    int newGlobalFaceIndex = static_cast<int>(mesh->face.size()) - 1;
                    mesh->face[newGlobalFaceIndex].IMark() = targetMark;
                    result.newFaceGlobals.push_back(newGlobalFaceIndex);
                }
            }
            return result;
        }

        // 步骤 E：缝边上的新顶点 -> 把外部邻接面在加点处一分为二
        // 注意：MergeResult 必须已声明（本方法定义在 MergeResult 之后）
        void propagateExternal(CMeshOD *mesh, const LocalMesh &lm, const MergeResult &merge)
        {
            // Split external faces where new seam vertices land on their boundary edges

            // 对每个新顶点（local >= Nv0）：判断是否落在某条缝边上
            for (int localVertexIndex = lm.Nv0; localVertexIndex < (int)lm.mesh.vert.size(); localVertexIndex++)
            {
                vcg::Point3d vertexPoint = lm.mesh.vert[localVertexIndex].P();
                // 找它落在哪条缝边（local 顶点对）上
                for (const auto &seamEntry : lm.seamExternal)
                {
                    int localVertexA = seamEntry.first.first, localVertexB = seamEntry.first.second;
                    int externalFaceIndex = seamEntry.second;
                    if (mesh->face[externalFaceIndex].IsD())
                    {
                        continue;
                    }
                    vcg::Point3d segmentPointA = lm.mesh.vert[localVertexA].P();
                    vcg::Point3d segmentPointB = lm.mesh.vert[localVertexB].P();
                    if (!pointOnSegment(vertexPoint, segmentPointA, segmentPointB))
                    {
                        continue;
                    }

                    // 新顶点 global
                    int globalVertexIndex = (localVertexIndex < (int)merge.vertLocalToGlobal.size()) ? merge.vertLocalToGlobal[localVertexIndex] : -1;
                    if (globalVertexIndex < 0)
                    {
                        continue;
                    }

                    // 外部面的三个顶点，找出缝边两端的 global 下标
                    int globalVertexA = lm.localToGlobalVert.size() > (size_t)localVertexA ? lm.localToGlobalVert[localVertexA] : -1;
                    int globalVertexB = lm.localToGlobalVert.size() > (size_t)localVertexB ? lm.localToGlobalVert[localVertexB] : -1;
                    splitExternalFace(mesh, externalFaceIndex, globalVertexA, globalVertexB, globalVertexIndex);
                    break; // 该新顶点已处理
                }
            }
        }

        // p 是否落在线段 a-b 上：投影参数 t in [0,1] 且距离 < tolerance。
        // 注意：VCG Point3d 的点乘是 operator*（无 .Dot 方法）。
        static bool pointOnSegment(const vcg::Point3d &p, const vcg::Point3d &a, const vcg::Point3d &b)
        {
            // Test whether point p lies on segment a-b within a small tolerance
            vcg::Point3d segmentVectorAB = b - a, segmentVectorAP = p - a;
            double projectionParameter = (segmentVectorAP * segmentVectorAB) / (segmentVectorAB * segmentVectorAB);
            if (projectionParameter < -1e-9 || projectionParameter > 1 + 1e-9)
            {
                return false;
            }
            vcg::Point3d projectionPoint = a + segmentVectorAB * projectionParameter;
            return (projectionPoint - p).Norm() < 1e-7;
        }

        // 把外部面 extG 沿 (ga, gb) 边在 gv 处一分为二。用下标访问，不跨 AddFace 持引用。
        static void splitExternalFace(CMeshOD *mesh, int extG, int ga, int gb, int gv)
        {
            // Split the external face into two faces at the seam vertex, then delete the original

            // 先读出所需信息（AddFace 可能 realloc mesh->face，使引用失效）
            int vertexIndices[3] = {
                mesh->face[extG].V(0)->Index(),
                mesh->face[extG].V(1)->Index(),
                mesh->face[extG].V(2)->Index()};
            int thirdVertexIndex = -1;
            for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
            {
                if (vertexIndices[vertexIndex] != ga && vertexIndices[vertexIndex] != gb)
                {
                    thirdVertexIndex = vertexIndices[vertexIndex];
                }
            }
            int originalMark = mesh->face[extG].IMark();
            if (thirdVertexIndex < 0)
            {
                return;
            }

            // 新面 A: (ga, gv, gc)
            vcg::tri::Allocator<CMeshOD>::AddFace(*mesh, &mesh->vert[ga], &mesh->vert[gv], &mesh->vert[thirdVertexIndex]);
            mesh->face.back().IMark() = originalMark;
            // 新面 B: (gv, gb, gc)
            vcg::tri::Allocator<CMeshOD>::AddFace(*mesh, &mesh->vert[gv], &mesh->vert[gb], &mesh->vert[thirdVertexIndex]);
            mesh->face.back().IMark() = originalMark;
            // 原 extG 标记删除（按下标，安全）
            mesh->face[extG].SetD();
        }

        // 步骤 F：resize m_newMark、重建 curFaces、重算 FF。在 D 之前调用 FF，D 之后不改拓扑。
        // 注意调用顺序：cutRegion 里先 finalizeTopology（重算 FF）→ 再 markCutEdges。
        void finalizeGrow(RegionMarker &regionMarker, CMeshOD *mesh)
        {
            // Grow mark storage and recompute face-face adjacency and normals
            regionMarker.growNewMark(mesh->face.size());
            vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*mesh);
            vcg::tri::UpdateNormal<CMeshOD>::PerFace(*mesh);
        }

        // 重建 curFaces：移除 SetD 原始面，加入新面
        static void rebuildCurFaces(
            std::vector<int> &curFaces,
            CMeshOD *mesh,
            const MergeResult &merge)
        {
            // Rebuild the working face list keeping live originals and appending new faces
            std::vector<int> result;
            for (int globalFaceIndex : curFaces)
            {
                if (globalFaceIndex < (int)mesh->face.size() && !mesh->face[globalFaceIndex].IsD())
                {
                    result.push_back(globalFaceIndex);
                }
            }
            for (int newFaceIndex : merge.newFaceGlobals)
            {
                result.push_back(newFaceIndex);
            }
            curFaces = result;
        }

        // 总装：对一个区域跑 A→B→cutter→C→E→F(FF)→D→F(curFaces)
        void cutRegion(
            CMeshOD *mesh,
            std::vector<int> &curFaces,
            const std::vector<Polyline> &polylines,
            int targetMark,
            RegionMarker &regionMarker)
        {
            // Cut one region: extract, cut dangling NON_MANIFOLD ends, merge back and finalize topology
            LocalMesh localMesh;
            extractLocalMesh(mesh, curFaces, localMesh);

            // B + cutter：对每条 NON_MANIFOLD 折线的每个悬空端点。
            // 记录端点 global 顶点 + cutLine（local 新顶点下标），merge 后转 global 并把端点拼到最前。
            struct PendingCut
            {
                int endpointGlobal;
                std::vector<int> cutLineLocal;
            };
            std::vector<PendingCut> pending;
            JasMeshAddCutLines cutter;
            MeshCutByMark::CutPlaneManager cutPlaneManager; // 复用 isOnMarkDiffEdge
            for (const auto &polyline : polylines)
            {
                if (polyline.type != CUT_EDGE_NON_MANIFOLD)
                {
                    continue;
                }
                if (!cutPlaneManager.isOnMarkDiffEdge(polyline.startFaceIdx, polyline.startEdgeIdx, mesh))
                {
                    auto cutInput = buildCutInput(polyline, true, localMesh, mesh);
                    std::vector<int> cutLine;
                    cutter.AddCutLines(&localMesh.mesh, cutInput.normal, cutInput.line, cutLine);
                    if (!cutLine.empty())
                    {
                        pending.push_back({polyline.vertexIndices.front(), cutLine});
                    }
                }
                if (!cutPlaneManager.isOnMarkDiffEdge(polyline.endFaceIdx, polyline.endEdgeIdx, mesh))
                {
                    auto cutInput = buildCutInput(polyline, false, localMesh, mesh);
                    std::vector<int> cutLine;
                    cutter.AddCutLines(&localMesh.mesh, cutInput.normal, cutInput.line, cutLine);
                    if (!cutLine.empty())
                    {
                        pending.push_back({polyline.vertexIndices.back(), cutLine});
                    }
                }
            }

            // C：merge 回主网格（新顶点此时 append 进 mesh，得到 vertLocalToGlobal）
            MergeResult merge = mergeBack(mesh, localMesh, targetMark);

            // cutLines：端点 global + 新顶点 global，拼成完整切割路径（spec D.1）
            std::vector<std::vector<int>> cutLines;
            for (const auto &pendingCut : pending)
            {
                std::vector<int> globalLine;
                globalLine.push_back(pendingCut.endpointGlobal); // 端点（原顶点，global）
                for (int localVertexIndex : pendingCut.cutLineLocal)
                {
                    if (localVertexIndex >= 0 && localVertexIndex < (int)merge.vertLocalToGlobal.size())
                    {
                        globalLine.push_back(merge.vertLocalToGlobal[localVertexIndex]);
                    }
                }
                if (globalLine.size() >= 2)
                {
                    cutLines.push_back(globalLine);
                }
            }

            // E：外部加点（在重算 FF 之前）
            propagateExternal(mesh, localMesh, merge);

            // F1：resize m_newMark + 重算 FF/normal
            finalizeGrow(regionMarker, mesh);

            // D：标分割边（cutLines 已是 global 顶点）
            markCutEdges(mesh, cutLines);

            // F2：重建 curFaces
            rebuildCurFaces(curFaces, mesh, merge);
        }
    };

    inline void LocalMeshCutManager::extractLocalMesh(
        CMeshOD *mesh,
        const std::vector<int> &curFaces,
        LocalMesh &localMesh)
    {
        // Extract a local copy of the region's faces plus boundary-seam info
        // 重置输出结构，避免上次调用残留。
        localMesh.mesh.Clear();
        localMesh.localToGlobalVert.clear();
        localMesh.localFaceToGlobal.clear();
        localMesh.seamExternal.clear();
        localMesh.Nv0 = 0;
        localMesh.localFaceToGlobal = curFaces; // local face i <-> global curFaces[i]

        // 1) 收集去重顶点，建映射
        std::map<int, int> globalToLocalVertex;
        for (int globalFaceIndex : curFaces)
        {
            for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++)
            {
                int globalVertexIndex = mesh->face[globalFaceIndex].V(vertexIndex)->Index();
                if (globalToLocalVertex.find(globalVertexIndex) == globalToLocalVertex.end())
                {
                    int localVertexIndex = (int)localMesh.localToGlobalVert.size();
                    globalToLocalVertex[globalVertexIndex] = localVertexIndex;
                    localMesh.localToGlobalVert.push_back(globalVertexIndex);
                }
            }
        }

        // 2) AddVertices + 拷坐标
        int localVertexCount = (int)localMesh.localToGlobalVert.size();
        vcg::tri::Allocator<CMeshOD>::AddVertices(localMesh.mesh, localVertexCount);
        for (int localVertexIndex = 0; localVertexIndex < localVertexCount; localVertexIndex++)
        {
            localMesh.mesh.vert[localVertexIndex].P() = mesh->vert[localMesh.localToGlobalVert[localVertexIndex]].P();
        }

        // 3) AddFace（顶点引用重映射）
        //    IMark 拷贝需两端 OCF mark 均已 Enable，否则访问空 MV 为 UB；
        //    此处采用 VCG 自带 ImportData 的守卫习惯（component_ocf.h MarkOcf::ImportData）。
        if (!localMesh.mesh.face.IsMarkEnabled())
        {
            localMesh.mesh.face.EnableMark();
        }
        if (!localMesh.mesh.face.IsFFAdjacencyEnabled())
        {
            localMesh.mesh.face.EnableFFAdjacency();
        }
        if (!localMesh.mesh.face.IsVFAdjacencyEnabled())
        {
            localMesh.mesh.face.EnableVFAdjacency();
        }
        if (!localMesh.mesh.vert.IsVFAdjacencyEnabled())
        {
            localMesh.mesh.vert.EnableVFAdjacency();
        }
        if (!localMesh.mesh.vert.IsMarkEnabled())
        {
            localMesh.mesh.vert.EnableMark();
        }
        for (int globalFaceIndex : curFaces)
        {
            int localVertexA = globalToLocalVertex[mesh->face[globalFaceIndex].V(0)->Index()];
            int localVertexB = globalToLocalVertex[mesh->face[globalFaceIndex].V(1)->Index()];
            int localVertexC = globalToLocalVertex[mesh->face[globalFaceIndex].V(2)->Index()];
            vcg::tri::Allocator<CMeshOD>::AddFace(
                localMesh.mesh, &localMesh.mesh.vert[localVertexA], &localMesh.mesh.vert[localVertexB], &localMesh.mesh.vert[localVertexC]);
            if (localMesh.mesh.face.IsMarkEnabled() && mesh->face.IsMarkEnabled())
            {
                localMesh.mesh.face.back().IMark() = mesh->face[globalFaceIndex].IMark();
            }
            localMesh.mesh.face.back().N() = mesh->face[globalFaceIndex].N();
        }

        localMesh.Nv0 = (int)localMesh.mesh.vert.size();

        // 4) 抓边界缝：curFaces 边在原 mesh 里 FFp 指向 curFaces 外部的，记外部面
        std::set<int> inCurrentFaces(curFaces.begin(), curFaces.end());
        for (int globalFaceIndex : curFaces)
        {
            for (int edgeIndex = 0; edgeIndex < 3; edgeIndex++)
            {
                CFaceOD *adjacentFace = mesh->face[globalFaceIndex].FFp(edgeIndex);
                if (adjacentFace == nullptr)
                {
                    continue;
                }
                int adjacentFaceIndex = static_cast<int>(adjacentFace - &mesh->face[0]);
                if (adjacentFaceIndex < 0 || adjacentFaceIndex == globalFaceIndex)
                {
                    continue;
                }
                if (inCurrentFaces.count(adjacentFaceIndex))
                {
                    continue; // 内部边，非缝
                }
                // 这是缝边：记录 local 顶点对 -> 外部面
                int globalVertexA = mesh->face[globalFaceIndex].V(edgeIndex)->Index();
                int globalVertexB = mesh->face[globalFaceIndex].V((edgeIndex + 1) % 3)->Index();
                int localVertexA = globalToLocalVertex[globalVertexA], localVertexB = globalToLocalVertex[globalVertexB];
                auto key = std::minmax(localVertexA, localVertexB);
                localMesh.seamExternal[{key.first, key.second}] = adjacentFaceIndex;
            }
        }

        return;
    }

} // namespace MeshCutByMark

#endif // LOCAL_MESH_CUT_H
