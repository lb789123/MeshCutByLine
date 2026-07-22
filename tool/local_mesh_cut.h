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

namespace MeshCutByMark {

class LocalMeshCutManager {
public:
    // 局部 mesh + 各种映射
    struct LocalMesh {
        CMeshOD mesh;
        int Nv0 = 0;                                  // 原顶点数（新顶点 local 下标 >= Nv0）
        std::vector<int> localToGlobalVert;           // localVertIdx -> globalVertIdx（仅 < Nv0）
        std::vector<int> localFaceToGlobal;           // localFaceIdx -> globalFaceIdx（仅原始面）
        // 边界缝：local 原始边的两个 local 顶点 -> 外部邻接面 global idx
        std::map<std::pair<int,int>, int> seamExternal;
    };

    // 步骤 A：从 m_pMesh 的 curFaces 提取局部 mesh
    LocalMesh extractLocalMesh(CMeshOD* mesh, const std::vector<int>& curFaces);

    struct CutInput {
        std::vector<vcg::Point3d> line;
        vcg::Point3d normal;
    };

    // 步骤 B：构造延长段 line + 区域 normal
    CutInput buildCutInput(const Polyline& polyline, bool isStart,
                           const LocalMesh& lm, CMeshOD* mesh) {
        CutInput ci;
        const auto& vi = polyline.vertexIndices;
        int endpointIdx = isStart ? vi.front() : vi.back();

        vcg::Point3d P = mesh->vert[endpointIdx].P();
        vcg::Point3d D;
        if (isStart) {
            D = mesh->vert[vi[0]].P() - mesh->vert[vi[1]].P();
        } else {
            int n = (int)vi.size();
            D = mesh->vert[vi[n-1]].P() - mesh->vert[vi[n-2]].P();
        }
        D.Normalize();

        // L = localMesh 包围盒对角线
        vcg::Box3d box;
        for (int i = 0; i < (int)lm.mesh.vert.size(); i++) box.Add(lm.mesh.vert[i].P());
        double L = box.Diag();
        if (L < 1e-9) L = 1.0;

        ci.line.push_back(P);
        ci.line.push_back(P + D * L);

        // normal = 区域法向（取第一个 curFaces 面法向）
        ci.normal = mesh->face[lm.localFaceToGlobal[0]].N();
        if (ci.normal.Norm() < 1e-9) ci.normal = vcg::Point3d(0, 0, 1);
        return ci;
    }

    // 步骤 D：把 cutLines 上的边标成边界（FFp 自指）
    void markCutEdges(CMeshOD* mesh, const std::vector<std::vector<int>>& cutLines) {
        // 收集所有要标记的 global 顶点对
        std::set<std::pair<int,int>> edges;
        for (const auto& cl : cutLines) {
            for (size_t i = 0; i + 1 < cl.size(); i++) {
                edges.insert(std::minmax(cl[i], cl[i+1]));
            }
        }
        if (edges.empty()) return;

        // 遍历面边，命中则两侧 FFp 自指
        for (int f = 0; f < (int)mesh->face.size(); f++) {
            if (mesh->face[f].IsD()) continue;
            for (int e = 0; e < 3; e++) {
                int a = mesh->face[f].V(e)->Index();
                int b = mesh->face[f].V((e+1)%3)->Index();
                if (edges.count(std::minmax(a,b))) {
                    CFaceOD* adj = mesh->face[f].FFp(e);
                    mesh->face[f].FFp(e) = &mesh->face[f];
                    mesh->face[f].FFi(e) = e;
                    if (adj != nullptr && adj != &mesh->face[f]) {
                        // 断开对面的同一条边
                        for (int e2 = 0; e2 < 3; e2++) {
                            if (adj->FFp(e2) == &mesh->face[f]) {
                                adj->FFp(e2) = adj;
                                adj->FFi(e2) = e2;
                            }
                        }
                    }
                }
            }
        }
    }

    // 步骤 C 的返回：merge 回主网格后给上层用的映射
    struct MergeResult {
        std::vector<int> newFaceGlobals;       // append 进 mesh 的新面 global 下标
        std::vector<int> splitOriginGlobals;   // 被 SetD 的原始面 global 下标
        std::vector<int> vertLocalToGlobal;    // localVertIdx -> globalVertIdx（含新顶点）
    };

    // 判断 local 面是否引用新顶点（local 下标 >= Nv0）
    static bool faceHasNewVert(const LocalMesh& lm, int localFaceIdx) {
        const CFaceOD& f = lm.mesh.face[localFaceIdx];
        for (int j = 0; j < 3; j++) {
            int lv = static_cast<int>(f.V(j) - &lm.mesh.vert[0]);
            if (lv >= lm.Nv0) return true;
        }
        return false;
    }

    // 反推新面来自哪个原始面（global）：新面质心落在哪个 curFaces 三角形内，就是被分裂的那个。
    // 比边查表稳：对"完整边是共享内部边"的切片也能正确归属（质心唯一落在一个原三角内）。
    static int deriveOriginGlobal(const LocalMesh& lm, int localFaceIdx, CMeshOD* mesh) {
        const CFaceOD& f = lm.mesh.face[localFaceIdx];
        vcg::Point3d centroid = (f.V(0)->P() + f.V(1)->P() + f.V(2)->P()) / 3.0;
        for (int gf : lm.localFaceToGlobal) {
            if (gf < 0 || gf >= (int)mesh->face.size()) continue;
            if (mesh->face[gf].IsD()) continue;  // already handled by a sibling piece
            const CFaceOD& of = mesh->face[gf];
            if (pointInTriangle(centroid, of.V(0)->P(), of.V(1)->P(), of.V(2)->P()))
                return gf;
        }
        return -1;
    }

    // p 与 c 是否在直线 ab 同侧（含容差）
    static bool sameSide(const vcg::Point3d& p, const vcg::Point3d& a,
                         const vcg::Point3d& b, const vcg::Point3d& c) {
        vcg::Point3d cp1 = (b - a) ^ (p - a);
        vcg::Point3d cp2 = (b - a) ^ (c - a);
        return (cp1 * cp2) >= -1e-9;
    }
    // p 是否在三角形 abc 内（共面，用同侧法）
    static bool pointInTriangle(const vcg::Point3d& p, const vcg::Point3d& a,
                                const vcg::Point3d& b, const vcg::Point3d& c) {
        return sameSide(p, a, b, c) && sameSide(p, b, c, a) && sameSide(p, c, a, b);
    }

    // 步骤 C：把 cutter 切过的 local mesh merge 回主网格。
    // 新顶点/新面 append 进 mesh；被分裂的原始面 SetD()。
    MergeResult mergeBack(CMeshOD* mesh, LocalMesh& lm, int targetMark) {
        MergeResult res;

        // 1) append 所有新顶点（local >= Nv0）到 *mesh*，一次性批量加（避免多次 realloc）
        int numNew = static_cast<int>(lm.mesh.vert.size()) - lm.Nv0;
        int firstNewG = static_cast<int>(mesh->vert.size());
        if (numNew > 0) vcg::tri::Allocator<CMeshOD>::AddVertices(*mesh, numNew);
        res.vertLocalToGlobal = lm.localToGlobalVert;  // < Nv0 部分
        for (int k = 0; k < numNew; k++) {
            mesh->vert[firstNewG + k].P() = lm.mesh.vert[lm.Nv0 + k].P();
            res.vertLocalToGlobal.push_back(firstNewG + k);
        }

        // 2) 遍历 local 面：新面（含新顶点）append，未动原始面跳过
        for (int i = 0; i < (int)lm.mesh.face.size(); i++) {
            if (lm.mesh.face[i].IsD()) continue;
            if (!faceHasNewVert(lm, i)) continue;  // 未动原始面，保持

            int ga = res.vertLocalToGlobal[static_cast<int>(lm.mesh.face[i].V(0) - &lm.mesh.vert[0])];
            int gb = res.vertLocalToGlobal[static_cast<int>(lm.mesh.face[i].V(1) - &lm.mesh.vert[0])];
            int gc = res.vertLocalToGlobal[static_cast<int>(lm.mesh.face[i].V(2) - &lm.mesh.vert[0])];
            vcg::tri::Allocator<CMeshOD>::AddFace(
                *mesh, &mesh->vert[ga], &mesh->vert[gb], &mesh->vert[gc]);
            int newG = static_cast<int>(mesh->face.size()) - 1;
            mesh->face[newG].IMark() = targetMark;
            res.newFaceGlobals.push_back(newG);

            // 反推来源 global -> SetD
            int splitG = deriveOriginGlobal(lm, i, mesh);
            if (splitG >= 0 && splitG < (int)mesh->face.size() && !mesh->face[splitG].IsD()) {
                mesh->face[splitG].SetD();
                res.splitOriginGlobals.push_back(splitG);
            }
        }
        return res;
    }

    // 步骤 E：缝边上的新顶点 -> 把外部邻接面在加点处一分为二
    // 注意：MergeResult 必须已声明（本方法定义在 MergeResult 之后）
    void propagateExternal(CMeshOD* mesh, const LocalMesh& lm, const MergeResult& merge) {
        // 对每个新顶点（local >= Nv0）：判断是否落在某条缝边上
        for (int lv = lm.Nv0; lv < (int)lm.mesh.vert.size(); lv++) {
            vcg::Point3d p = lm.mesh.vert[lv].P();
            // 找它落在哪条缝边（local 顶点对）上
            for (const auto& kv : lm.seamExternal) {
                int la = kv.first.first, lb = kv.first.second;
                int extG = kv.second;
                if (mesh->face[extG].IsD()) continue;
                vcg::Point3d pa = lm.mesh.vert[la].P();
                vcg::Point3d pb = lm.mesh.vert[lb].P();
                if (!pointOnSegment(p, pa, pb)) continue;

                // 新顶点 global
                int gv = (lv < (int)merge.vertLocalToGlobal.size()) ? merge.vertLocalToGlobal[lv] : -1;
                if (gv < 0) continue;

                // 外部面的三个顶点，找出缝边两端的 global 下标
                int ga = lm.localToGlobalVert.size() > (size_t)la ? lm.localToGlobalVert[la] : -1;
                int gb = lm.localToGlobalVert.size() > (size_t)lb ? lm.localToGlobalVert[lb] : -1;
                splitExternalFace(mesh, extG, ga, gb, gv);
                break;  // 该新顶点已处理
            }
        }
    }

    // p 是否落在线段 a-b 上：投影参数 t in [0,1] 且距离 < tolerance。
    // 注意：VCG Point3d 的点乘是 operator*（无 .Dot 方法）。
    static bool pointOnSegment(const vcg::Point3d& p, const vcg::Point3d& a, const vcg::Point3d& b) {
        vcg::Point3d ab = b - a, ap = p - a;
        double t = (ap * ab) / (ab * ab);
        if (t < -1e-9 || t > 1 + 1e-9) return false;
        vcg::Point3d proj = a + ab * t;
        return (proj - p).Norm() < 1e-7;
    }

    // 把外部面 extG 沿 (ga, gb) 边在 gv 处一分为二。用下标访问，不跨 AddFace 持引用。
    static void splitExternalFace(CMeshOD* mesh, int extG, int ga, int gb, int gv) {
        // 先读出所需信息（AddFace 可能 realloc mesh->face，使引用失效）
        int idx[3] = {
            mesh->face[extG].V(0)->Index(),
            mesh->face[extG].V(1)->Index(),
            mesh->face[extG].V(2)->Index() };
        int gc = -1;
        for (int j = 0; j < 3; j++) if (idx[j] != ga && idx[j] != gb) gc = idx[j];
        int imark = mesh->face[extG].IMark();
        if (gc < 0) return;

        // 新面 A: (ga, gv, gc)
        vcg::tri::Allocator<CMeshOD>::AddFace(*mesh, &mesh->vert[ga], &mesh->vert[gv], &mesh->vert[gc]);
        mesh->face.back().IMark() = imark;
        // 新面 B: (gv, gb, gc)
        vcg::tri::Allocator<CMeshOD>::AddFace(*mesh, &mesh->vert[gv], &mesh->vert[gb], &mesh->vert[gc]);
        mesh->face.back().IMark() = imark;
        // 原 extG 标记删除（按下标，安全）
        mesh->face[extG].SetD();
    }

    // 步骤 F：resize m_newMark、重建 curFaces、重算 FF。在 D 之前调用 FF，D 之后不改拓扑。
    // 注意调用顺序：cutRegion 里先 finalizeTopology（重算 FF）→ 再 markCutEdges。
    void finalizeGrow(RegionMarker& regionMarker, CMeshOD* mesh) {
        regionMarker.growNewMark(mesh->face.size());
        vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*mesh);
        vcg::tri::UpdateNormal<CMeshOD>::PerFace(*mesh);
    }

    // 重建 curFaces：移除 SetD 原始面，加入新面
    static void rebuildCurFaces(std::vector<int>& curFaces, CMeshOD* mesh,
                                const MergeResult& merge) {
        std::vector<int> out;
        for (int gf : curFaces) {
            if (gf < (int)mesh->face.size() && !mesh->face[gf].IsD()) out.push_back(gf);
        }
        for (int nf : merge.newFaceGlobals) out.push_back(nf);
        curFaces = out;
    }

    // 总装：对一个区域跑 A→B→cutter→C→E→F(FF)→D→F(curFaces)
    void cutRegion(CMeshOD* mesh,
                   std::vector<int>& curFaces,
                   const std::vector<Polyline>& polylines,
                   int targetMark,
                   RegionMarker& regionMarker)
    {
        LocalMesh lm = extractLocalMesh(mesh, curFaces);

        // B + cutter：对每条 NON_MANIFOLD 折线的每个悬空端点。
        // 记录端点 global 顶点 + cutLine（local 新顶点下标），merge 后转 global 并把端点拼到最前。
        struct PendingCut { int endpointGlobal; std::vector<int> cutLineLocal; };
        std::vector<PendingCut> pending;
        JasMeshAddCutLines cutter;
        MeshCutByMark::CutPlaneManager cpm;  // 复用 isOnMarkDiffEdge
        for (const auto& pl : polylines) {
            if (pl.type != CUT_EDGE_NON_MANIFOLD) continue;
            if (!cpm.isOnMarkDiffEdge(pl.startFaceIdx, pl.startEdgeIdx, mesh)) {
                auto ci = buildCutInput(pl, true, lm, mesh);
                std::vector<int> cl;
                cutter.AddCutLines(&lm.mesh, ci.normal, ci.line, cl);
                if (!cl.empty()) pending.push_back({pl.vertexIndices.front(), cl});
            }
            if (!cpm.isOnMarkDiffEdge(pl.endFaceIdx, pl.endEdgeIdx, mesh)) {
                auto ci = buildCutInput(pl, false, lm, mesh);
                std::vector<int> cl;
                cutter.AddCutLines(&lm.mesh, ci.normal, ci.line, cl);
                if (!cl.empty()) pending.push_back({pl.vertexIndices.back(), cl});
            }
        }

        // C：merge 回主网格（新顶点此时 append 进 mesh，得到 vertLocalToGlobal）
        MergeResult merge = mergeBack(mesh, lm, targetMark);

        // cutLines：端点 global + 新顶点 global，拼成完整切割路径（spec D.1）
        std::vector<std::vector<int>> cutLines;
        for (const auto& pc : pending) {
            std::vector<int> g;
            g.push_back(pc.endpointGlobal);  // 端点（原顶点，global）
            for (int lv : pc.cutLineLocal) {
                if (lv >= 0 && lv < (int)merge.vertLocalToGlobal.size())
                    g.push_back(merge.vertLocalToGlobal[lv]);
            }
            if (g.size() >= 2) cutLines.push_back(g);
        }

        // E：外部加点（在重算 FF 之前）
        propagateExternal(mesh, lm, merge);

        // F1：resize m_newMark + 重算 FF/normal
        finalizeGrow(regionMarker, mesh);

        // D：标分割边（cutLines 已是 global 顶点）
        markCutEdges(mesh, cutLines);

        // F2：重建 curFaces
        rebuildCurFaces(curFaces, mesh, merge);
    }
};

inline LocalMeshCutManager::LocalMesh LocalMeshCutManager::extractLocalMesh(
    CMeshOD* mesh, const std::vector<int>& curFaces)
{
    LocalMesh lm;
    lm.localToGlobalVert.clear();
    lm.localFaceToGlobal = curFaces;  // local face i <-> global curFaces[i]

    // 1) 收集去重顶点，建映射
    std::map<int,int> globalToLocal;
    for (int gf : curFaces) {
        for (int j = 0; j < 3; j++) {
            int gv = mesh->face[gf].V(j)->Index();
            if (globalToLocal.find(gv) == globalToLocal.end()) {
                int li = (int)lm.localToGlobalVert.size();
                globalToLocal[gv] = li;
                lm.localToGlobalVert.push_back(gv);
            }
        }
    }

    // 2) AddVertices + 拷坐标
    int nv = (int)lm.localToGlobalVert.size();
    vcg::tri::Allocator<CMeshOD>::AddVertices(lm.mesh, nv);
    for (int li = 0; li < nv; li++) {
        lm.mesh.vert[li].P() = mesh->vert[lm.localToGlobalVert[li]].P();
    }

    // 3) AddFace（顶点引用重映射）
    //    IMark 拷贝需两端 OCF mark 均已 Enable，否则访问空 MV 为 UB；
    //    此处采用 VCG 自带 ImportData 的守卫习惯（component_ocf.h MarkOcf::ImportData）。
    for (int gf : curFaces) {
        int la = globalToLocal[mesh->face[gf].V(0)->Index()];
        int lb = globalToLocal[mesh->face[gf].V(1)->Index()];
        int lc = globalToLocal[mesh->face[gf].V(2)->Index()];
        vcg::tri::Allocator<CMeshOD>::AddFace(
            lm.mesh, &lm.mesh.vert[la], &lm.mesh.vert[lb], &lm.mesh.vert[lc]);
        if (lm.mesh.face.IsMarkEnabled() && mesh->face.IsMarkEnabled()) {
            lm.mesh.face.back().IMark() = mesh->face[gf].IMark();
        }
        lm.mesh.face.back().N() = mesh->face[gf].N();
    }

    lm.Nv0 = (int)lm.mesh.vert.size();

    // 4) 抓边界缝：curFaces 边在原 mesh 里 FFp 指向 curFaces 外部的，记外部面
    std::set<int> inCur(curFaces.begin(), curFaces.end());
    for (int gf : curFaces) {
        for (int j = 0; j < 3; j++) {
            CFaceOD* adj = mesh->face[gf].FFp(j);
            if (adj == nullptr) continue;
            int adjIdx = static_cast<int>(adj - &mesh->face[0]);
            if (adjIdx < 0 || adjIdx == gf) continue;
            if (inCur.count(adjIdx)) continue;  // 内部边，非缝
            // 这是缝边：记录 local 顶点对 -> 外部面
            int ga = mesh->face[gf].V(j)->Index();
            int gb = mesh->face[gf].V((j+1)%3)->Index();
            int la = globalToLocal[ga], lb = globalToLocal[gb];
            auto key = std::minmax(la, lb);
            lm.seamExternal[{key.first, key.second}] = adjIdx;
        }
    }

    return lm;
}

} // namespace MeshCutByMark

#endif // LOCAL_MESH_CUT_H
