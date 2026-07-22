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
        // 原始面的 global 顶点对 -> 该 curFaces 面 global idx（用未改动的 m_pMesh 建表，反推来源用）
        std::map<std::pair<int,int>, int> globalEdgeToCurFace;
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

    // （后续 Task 实现）
    // markCutEdges(...) / propagateExternal(...) / cutRegion(...)

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

    // 反推新面来自哪个原始面（直接返回 global 面下标）：
    // 取新面里 < Nv0 的原顶点 -> global，用 globalEdgeToCurFace 查它们构成的边属于哪个 curFaces 面。
    // 用建表时的未改动 m_pMesh 拓扑，不受 cutter 原地改写 local 面的影响。
    static int deriveOriginGlobal(const LocalMesh& lm, int localFaceIdx) {
        const CFaceOD& f = lm.mesh.face[localFaceIdx];
        int origs[3], no = 0;
        for (int j = 0; j < 3; j++) {
            int lv = static_cast<int>(f.V(j) - &lm.mesh.vert[0]);
            if (lv < lm.Nv0 && lv < (int)lm.localToGlobalVert.size())
                origs[no++] = lm.localToGlobalVert[lv];
        }
        for (int a = 0; a < no; a++)
            for (int b = a + 1; b < no; b++) {
                auto it = lm.globalEdgeToCurFace.find(std::minmax(origs[a], origs[b]));
                if (it != lm.globalEdgeToCurFace.end()) return it->second;
            }
        return -1;
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
            int splitG = deriveOriginGlobal(lm, i);
            if (splitG >= 0 && splitG < (int)mesh->face.size() && !mesh->face[splitG].IsD()) {
                mesh->face[splitG].SetD();
                res.splitOriginGlobals.push_back(splitG);
            }
        }
        return res;
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

    // 4) 建 globalEdgeToCurFace：用未改动的 mesh 原始面建表（反推来源用）
    for (int gf : curFaces) {
        for (int j = 0; j < 3; j++) {
            int a = mesh->face[gf].V(j)->Index();
            int b = mesh->face[gf].V((j+1)%3)->Index();
            lm.globalEdgeToCurFace[std::minmax(a, b)] = gf;
        }
    }

    // 5) 抓边界缝：curFaces 边在原 mesh 里 FFp 指向 curFaces 外部的，记外部面
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
