/**
 * @brief MeshCutByMark - 将网格按 mark 分割成简单多边形
 *
 * 核心功能：
 * 1. 按 mark 分组三角形
 * 2. 找到切割边（mark 不同、非流形、边界）
 * 3. 将切割边连接成折线
 * 4. 从端点延长切割
 * 5. 通过新标记机制得到简单多边形
 *
 * 输入：带有 mark 属性的三角形网格
 * 输出：简单多边形区域列表
 */
#ifndef JASMESHMARKANDSPLIT_H
#define JASMESHMARKANDSPLIT_H

#include <tool/cmesh.h>
#include <vector>
#include "tool/edge_info.h"
#include "tool/polyline.h"
#include "tool/cut_plane.h"
#include "tool/region_marker.h"
class JasMeshMarkAndSplit
{
public:

	struct splitReg
	{
		int mark;                    // 原始平面标记
		int newMark;                 // 新标记（简单多边形 ID）
		std::vector<int> inTris;     // 包含的三角形索引
		vcg::Point3d normal;         // 法向量
		std::vector<int> boundlines; // 边界边的顶点索引序列
	};

	JasMeshMarkAndSplit();
	~JasMeshMarkAndSplit();

	//设置mesh 并且已经根据mark标记了三角形归属的每个平面
	void SetMainMesh(CMeshO* pMesh) { m_pMesh = pMesh; }

	//构建边信息（在SetMainMesh之后调用）
	void BuildEdgeInfo() { m_edgeInfoManager.buildEdgeInfo(m_pMesh); }

	//返回分割后的多个三维多边形
	void SplitMeshByMarkAndEdge(std::vector<splitReg> &retRegs);

	//查找给定面集合中的切割边
	std::vector<MeshCutByMark::CutEdge> findCutEdges(const std::vector<int>& curFaces);

	//提取区域的边界边序列
	std::vector<std::vector<int>> extractBoundaryEdges(const std::vector<int>& regionFaces);
private:
	CMeshO* m_pMesh = nullptr;
	MeshCutByMark::EdgeInfoManager m_edgeInfoManager; // 边信息管理器
	MeshCutByMark::PolylineManager m_polylineManager; // 折线管理器
	MeshCutByMark::CutPlaneManager m_cutPlaneManager; // 切割平面管理器
	MeshCutByMark::RegionMarker m_regionMarker;       // 区域标记管理器
	int m_newMarkCounter = 0;                          // 新区域标记计数器
	std::vector<vcg::Point3i> m_edgeMarks;//对应是否是分割边
};

#endif // JASMESHMARKANDSPLIT_H
