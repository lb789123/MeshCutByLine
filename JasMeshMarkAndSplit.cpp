#include "JasMeshMarkAndSplit.h"

JasMeshMarkAndSplit::JasMeshMarkAndSplit()
{
}

JasMeshMarkAndSplit::~JasMeshMarkAndSplit()
{
}

std::vector<MeshCutByMark::CutEdge> JasMeshMarkAndSplit::findCutEdges(const std::vector<int>& curFaces) {
    std::vector<MeshCutByMark::CutEdge> cutEdges;

    for (int faceIdx : curFaces) {
        for (int j = 0; j < 3; j++) {
            int v0 = m_pMesh->face[faceIdx].V(j)->Index();
            int v1 = m_pMesh->face[faceIdx].V((j+1)%3)->Index();

            if (v0 > v1) std::swap(v0, v1);

            MeshCutByMark::CutEdgeType type = m_edgeInfoManager.getEdgeType(v0, v1);

            if (type != MeshCutByMark::CUT_EDGE_NONE) {
                MeshCutByMark::CutEdge edge;
                edge.v0 = v0;
                edge.v1 = v1;
                edge.faceIdx = faceIdx;
                edge.edgeIdx = j;
                edge.type = type;
                cutEdges.push_back(edge);
            }
        }
    }

    return cutEdges;
}

void JasMeshMarkAndSplit::SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs)
{
	//先构建所有mesh的边，并且标记分割边：边左右mark不同、非流形边。

	//从每个面开始查找不被分割边切断的区域
	std::vector<int> fflag(m_pMesh->face.size(), 0);

	for (int i = 0; i < fflag.size(); ++i)
	{
		if (fflag[i])continue;

		//从i通过非递归调用得到 和i在一个面上的所有三角形
		std::vector<int> curFaces;

		//curFaces中找到分割边
		std::vector<std::pair<int,int>> cutLines;

		//分割边链接成对应的线

		//每个线延长到curFaces box范围 切割所有三角形。

		//三角形更新 并找到多个
		std::vector<splitReg> iLocalRegs;

		if (iLocalRegs.size())
			retRegs.insert(retRegs.begin(), iLocalRegs.begin(), iLocalRegs.end());
	}
}
