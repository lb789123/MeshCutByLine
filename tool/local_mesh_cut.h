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
    // void mergeBack(...) / markCutEdges(...) / propagateExternal(...) / cutRegion(...)
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
