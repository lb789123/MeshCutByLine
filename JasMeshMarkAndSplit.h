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
		int mark, polyInd;//归属平面标记 和平面索引
		std::vector<int> inTris;//包含的三角形
		vcg::Point3d normal;//法向量
		std::vector<int> boundlines;//多边形顶点索引
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
