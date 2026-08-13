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
#include "local_cut_result.h"
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
		void extractLocalMesh(CMeshOD* mesh, const std::vector<int>& curFaces,
			LocalMesh& localMesh);

		struct CutInput
		{
			std::vector<vcg::Point3d> line;
			vcg::Point3d normal;
		};

		// 步骤 B：构造切割 line = 首端延长段(可选) + 折线本体 + 尾端延长段(可选)，以及区域 normal
		CutInput buildCutInput(
			const Polyline& polyline,
			bool extendStart,
			bool extendEnd,
			const LocalMesh& lm,
			CMeshOD* mesh)
		{
			// Build the full cut path: optional start extension + polyline + optional end extension
			CutInput cutInput;
			const auto& vertexIndices = polyline.vertexIndices;
			if (vertexIndices.size() < 2)
			{
				return cutInput; // 退化折线：cutter 直接 no-op
			}

			// L = localMesh 包围盒对角线，保证延长段切穿区域
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

			// 首端延长：方向沿首段折线方向继续向外
			if (extendStart)
			{
				vcg::Point3d startDirection =
					mesh->vert[vertexIndices[0]].P() - mesh->vert[vertexIndices[1]].P();
				startDirection.Normalize();
				vcg::Point3d startPoint = mesh->vert[vertexIndices.front()].P();
				cutInput.line.push_back(startPoint + startDirection * diagonalLength);
			}

			// 折线本体
			for (int vertexIndex : vertexIndices)
			{
				cutInput.line.push_back(mesh->vert[vertexIndex].P());
			}

			// 尾端延长：方向沿末段折线方向继续向外
			if (extendEnd)
			{
				int vertexCount = (int)vertexIndices.size();
				vcg::Point3d endDirection =
					mesh->vert[vertexIndices[vertexCount - 1]].P() - mesh->vert[vertexIndices[vertexCount - 2]].P();
				endDirection.Normalize();
				vcg::Point3d endPoint = mesh->vert[vertexIndices.back()].P();
				cutInput.line.push_back(endPoint + endDirection * diagonalLength);
			}

			// normal = 区域法向（取第一个 curFaces 面法向）
			cutInput.normal = mesh->face[lm.localFaceToGlobal[0]].N();
			if (cutInput.normal.Norm() < 1e-9)
			{
				cutInput.normal = vcg::Point3d(0, 0, 1);
			}
			return cutInput;
		}

		// 步骤 D：把 cutLines 上的边标成边界（FFp 自指）
		void markCutEdges(CMeshOD* mesh, const std::vector<std::vector<int>>& cutLines)
		{
			// Make both sides of each cut line edge point to itself in FF adjacency

			// 收集所有要标记的 global 顶点对
			std::set<std::pair<int, int>> edgeSet;
			for (const auto& cutLine : cutLines)
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
						CFaceOD* adjacentFace = mesh->face[faceIndex].FFp(edgeIndex);
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
		static bool faceHasNewVert(const LocalMesh& lm, int localFaceIdx)
		{
			// Return true when any vertex of the face is a newly added local vertex
			const CFaceOD& face = lm.mesh.face[localFaceIdx];
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
		MergeResult mergeBack(CMeshOD* mesh, LocalMesh& lm, int targetMark)
		{
			// Append new vertices and rewrite/append faces touched by the cutter
			MergeResult result;

			// 1) append 所有新顶点（local >= Nv0）到 *mesh*，一次性批量加（避免多次 realloc）
			int newVertexCount = static_cast<int>(lm.mesh.vert.size()) - lm.Nv0;
			result.vertLocalToGlobal = lm.localToGlobalVert; // < Nv0 部分
			for (int newVertexIndex = 0; newVertexIndex < newVertexCount; newVertexIndex++)
			{
				// 已被 AddCutLines 清理（折叠后无面引用）的顶点不 append，占位 -1 保持映射对齐
				if (lm.mesh.vert[lm.Nv0 + newVertexIndex].IsD())
				{
					result.vertLocalToGlobal.push_back(-1);
					continue;
				}
				vcg::tri::Allocator<CMeshOD>::AddVertices(*mesh, 1);
				mesh->vert.back().P() = lm.mesh.vert[lm.Nv0 + newVertexIndex].P();
				result.vertLocalToGlobal.push_back(static_cast<int>(mesh->vert.size()) - 1);
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
		void propagateExternal(CMeshOD* mesh, const LocalMesh& lm, const MergeResult& merge,
			LocalCutResult* result = nullptr, bool stitch = true)
		{
			// 收集每条缝边内部的新顶点（沿边参数 t 排序），同一缝边多个交点
			// 必须一次分割；并维护“缝边 -> 外部邻接面”的可变映射，分割后把
			// 其他指向同一外部面的缝边重定向到包含它的子面，保证后续顶点能
			// 继续细分（否则外部面只被切一刀，其余位置留下裂缝）。
			std::map<std::pair<int, int>, std::vector<SeamCutPoint>> seamPoints;
			for (int localVertexIndex = lm.Nv0; localVertexIndex < (int)lm.mesh.vert.size(); localVertexIndex++)
			{
				if (lm.mesh.vert[localVertexIndex].IsD())
				{
					continue; // 已被清理的孤立顶点不参与外部加点
				}
				vcg::Point3d vertexPoint = lm.mesh.vert[localVertexIndex].P();
				for (const auto& seamEntry : lm.seamExternal)
				{
					int localVertexA = seamEntry.first.first, localVertexB = seamEntry.first.second;
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
					// 新顶点与缝边端点重合：外部面已经共享该端点，再分割只会
					// 产生退化面/重合顶点（非流形点），直接跳过，保持纯分割语义。
					if (globalVertexIndex == globalVertexA ||
						globalVertexIndex == globalVertexB ||
						(vertexPoint - segmentPointA).Norm() < 1e-9 ||
						(vertexPoint - segmentPointB).Norm() < 1e-9)
					{
						continue;
					}
					vcg::Point3d segmentVectorAB = segmentPointB - segmentPointA;
					double projectionParameter =
						(vertexPoint - segmentPointA) * segmentVectorAB / segmentVectorAB.SquaredNorm();
					SeamCutPoint cutPoint;
					cutPoint.localVertexIndex = localVertexIndex;
					cutPoint.globalVertexIndex = globalVertexIndex;
					cutPoint.t = projectionParameter;
					cutPoint.point = vertexPoint;
					seamPoints[{ localVertexA, localVertexB }].push_back(cutPoint);
					break; // 该新顶点已处理
				}
			}

			// 可变的外部面映射：分割后其他缝边重定向到包含它的子面
			std::map<std::pair<int, int>, int> seamFace = lm.seamExternal;
			for (auto& entry : seamPoints)
			{
				auto& points = entry.second;
				if (points.empty())
				{
					continue;
				}
				std::sort(points.begin(), points.end(),
					[](const SeamCutPoint& lhs, const SeamCutPoint& rhs)
					{
						return lhs.t < rhs.t;
					});
				std::vector<int> splitVertices;
				for (const auto& point : points)
				{
					if (splitVertices.empty() || splitVertices.back() != point.globalVertexIndex)
					{
						splitVertices.push_back(point.globalVertexIndex);
					}
				}
				if (splitVertices.empty())
				{
					continue;
				}
				const std::pair<int, int> seamKey = entry.first;
				int externalFaceIndex = seamFace[seamKey];
				if (externalFaceIndex < 0 || externalFaceIndex >= (int)mesh->face.size() ||
					mesh->face[externalFaceIndex].IsD())
				{
					continue; // 外部面缺失：防御
				}
				int globalVertexA = lm.localToGlobalVert.size() > (size_t)seamKey.first
					? lm.localToGlobalVert[seamKey.first] : -1;
				int globalVertexB = lm.localToGlobalVert.size() > (size_t)seamKey.second
					? lm.localToGlobalVert[seamKey.second] : -1;

				if (result != nullptr)
				{
					SeamCutLine seamLine;
					seamLine.globalVertexA = std::min(globalVertexA, globalVertexB);
					seamLine.globalVertexB = std::max(globalVertexA, globalVertexB);
					seamLine.externalFaceIndex = externalFaceIndex;
					seamLine.points = points; // 已按 t 排序
					result->seams.push_back(std::move(seamLine));
				}

				if (!stitch)
				{
					continue;
				}

				std::vector<int> newSubFaces;
				splitExternalFaceMulti(mesh, externalFaceIndex, globalVertexA, globalVertexB,
					splitVertices, newSubFaces);
				if (newSubFaces.empty())
				{
					continue;
				}

				// 更新其他缝边对该外部面的引用：重定向到包含该缝边的子面
				for (auto& otherEntry : seamFace)
				{
					if (otherEntry.second != externalFaceIndex)
					{
						continue;
					}
					if (otherEntry.first == seamKey)
					{
						otherEntry.second = -1; // 本缝边已处理
						continue;
					}
					int otherVertexA = lm.localToGlobalVert.size() > (size_t)otherEntry.first.first
						? lm.localToGlobalVert[otherEntry.first.first] : -1;
					int otherVertexB = lm.localToGlobalVert.size() > (size_t)otherEntry.first.second
						? lm.localToGlobalVert[otherEntry.first.second] : -1;
					for (int subFaceIndex : newSubFaces)
					{
						if (faceHasEdge(mesh, subFaceIndex, otherVertexA, otherVertexB))
						{
							otherEntry.second = subFaceIndex;
							break;
						}
					}
				}
			}
		}

		// p 是否落在线段 a-b 上：投影参数 t in [0,1] 且距离 < tolerance。
		// 注意：VCG Point3d 的点乘是 operator*（无 .Dot 方法）。
		static bool pointOnSegment(const vcg::Point3d& p, const vcg::Point3d& a, const vcg::Point3d& b)
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
				mesh->face[extG].V(2)->Index() };
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
			vcg::Point3d windingVectorAB = mesh->vert[windingVertexB].P() - mesh->vert[windingVertexA].P();
			for (int splitVertex : splitVertices)
			{
				vcg::Point3d windingVectorAP = mesh->vert[splitVertex].P() - mesh->vert[windingVertexA].P();
				double projectionParameter =
					windingVectorAP * windingVectorAB / windingVectorAB.SquaredNorm();
				orderedVertices.push_back({ projectionParameter, splitVertex });
			}
			std::sort(orderedVertices.begin(), orderedVertices.end());

			int previousVertex = windingVertexA;
			for (size_t splitIndex = 0; splitIndex <= orderedVertices.size(); splitIndex++)
			{
				int currentVertex =
					(splitIndex < orderedVertices.size()) ? orderedVertices[splitIndex].second : windingVertexB;
				if (previousVertex == currentVertex)
				{
					continue; // 防御：相邻分割点重合
				}
				vcg::tri::Allocator<CMeshOD>::AddFace(*mesh,
					&mesh->vert[previousVertex], &mesh->vert[currentVertex], &mesh->vert[thirdVertexIndex]);
				mesh->face.back().IMark() = originalMark;
				newSubFaces.push_back(static_cast<int>(mesh->face.size()) - 1);
				previousVertex = currentVertex;
			}
			// 原 extG 标记删除（按下标，安全）
			mesh->face[extG].SetD();
		}

		// 步骤 F1：resize m_newMark + 重算 FF/normal。在同步 local 区域标记之前调用。
		// 注意调用顺序：cutRegion 里先 finalizeTopology（重算 FF）→ 再 propagateLocalRegionMarks。
		void finalizeGrow(RegionMarker& regionMarker, CMeshOD* mesh)
		{
			// Grow mark storage and recompute face-face adjacency and normals
			regionMarker.growNewMark(mesh->face.size());
			vcg::tri::UpdateTopology<CMeshOD>::FaceFace(*mesh);
			vcg::tri::UpdateNormal<CMeshOD>::PerFace(*mesh);
		}

		// 重建 curFaces：移除 SetD 原始面，加入新面
		static void rebuildCurFaces(
			std::vector<int>& curFaces,
			CMeshOD* mesh,
			const MergeResult& merge)
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

		// 把 AddCutLines 在 local mesh 上产生的区域标记（IMark）同步为全局 new-mark。
		// 每个 local 区域标记首次出现时分配一个递增的全局 new-mark，同区域所有面共享。
		void propagateLocalRegionMarks(
			RegionMarker& regionMarker,
			CMeshOD* mesh,
			const LocalMesh& localMesh,
			const MergeResult& merge,
			int& newMarkCounter)
		{
			// local 区域标记 -> 全局 new-mark 的映射，避免与其他区域冲突
			std::map<int, int> localMarkToGlobalMark;
			const int numOriginFaces = static_cast<int>(localMesh.localFaceToGlobal.size());
			for (int localFaceIndex = 0; localFaceIndex < (int)localMesh.mesh.face.size(); localFaceIndex++)
			{
				const CFaceOD& localFace = localMesh.mesh.face[localFaceIndex];
				if (localFace.IsD())
				{
					continue;
				}

				// 原始面槽位按 localFaceToGlobal 映射；附加分片按 mergeBack 追加顺序映射
				int globalFaceIndex = -1;
				if (localFaceIndex < numOriginFaces)
				{
					globalFaceIndex = localMesh.localFaceToGlobal[localFaceIndex];
				}
				else
				{
					const int appendedIndex = localFaceIndex - numOriginFaces;
					if (appendedIndex >= 0 && appendedIndex < (int)merge.newFaceGlobals.size())
					{
						globalFaceIndex = merge.newFaceGlobals[appendedIndex];
					}
				}
				if (globalFaceIndex < 0 || globalFaceIndex >= (int)mesh->face.size() || mesh->face[globalFaceIndex].IsD())
				{
					continue;
				}

				const int localMark = localFace.IMark();
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
		}

		// 总装：对一个区域跑 A→B→cutter→C→E→F1→D(new)→F2(curFaces)
		void cutRegion(
			CMeshOD* mesh,
			std::vector<int>& curFaces,
			const std::vector<Polyline>& polylines,
			int targetMark,
			RegionMarker& regionMarker,
			int& newMarkCounter,
			LocalCutResult* localResult = nullptr,
			bool stitchSeams = true)
		{
			// Cut one region: extract, cut dangling NON_MANIFOLD ends, merge back and finalize topology
			LocalMesh localMesh;
			extractLocalMesh(mesh, curFaces, localMesh);

			// B + cutter：先收集所有需要延长的悬空端点（首端/尾端/两端），再统一执行切割。
			// AddCutLines 会完成切割并按“切割边不可跨越 + 已有标记”在 local mesh 上
			// 重新标记区域，因此这里不再需要 cutLine 输出。
			JasMeshAddCutLines cutter;

			// 当前区域的边界顶点：非 NON_MANIFOLD 折线（MARK_DIFF/BOUNDARY 及合并折线）
			// 的顶点，即落在区域边界上的顶点
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

			// 收集所有 NON_MANIFOLD 折线的延长切割输入，批量在 CGAL 精确网格会话中
			// 一次构建、多刀累积、最后统一写回（避免每条折线重复 VCG<->CGAL 往返）。
			std::vector<vcg::Point3d> normals;
			std::vector<std::vector<vcg::Point3d>> lines;
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
				auto cutInput = buildCutInput(polyline, extendStart, extendEnd, localMesh, mesh);
				normals.push_back(cutInput.normal);
				lines.push_back(std::move(cutInput.line));
			}
			std::vector<std::vector<int>> cutLines;
			cutter.AddCutLinesBatch(&localMesh.mesh, normals, lines, cutLines);

			// C：merge 回主网格（新顶点此时 append 进 mesh，得到 vertLocalToGlobal）
			MergeResult merge = mergeBack(mesh, localMesh, targetMark);

			// E：外部加点（在重算 FF 之前）
			propagateExternal(mesh, localMesh, merge, localResult, stitchSeams);

			// F1：resize m_newMark + 重算 FF/normal
			finalizeGrow(regionMarker, mesh);

			// D（新）：AddCutLines 已把切割边视为不可跨越并在 local mesh 上完成区域
			// 拆分与重新标记；这里把 local 区域标记同步为全局 new-mark。
			// 取代旧的 markCutEdges：不再用 FF 自指把切割边标成边界。
			propagateLocalRegionMarks(regionMarker, mesh, localMesh, merge, newMarkCounter);

			// F2：重建 curFaces
			rebuildCurFaces(curFaces, mesh, merge);
		}
	};

	inline void LocalMeshCutManager::extractLocalMesh(
		CMeshOD* mesh,
		const std::vector<int>& curFaces,
		LocalMesh& localMesh)
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
				CFaceOD* adjacentFace = mesh->face[globalFaceIndex].FFp(edgeIndex);
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
