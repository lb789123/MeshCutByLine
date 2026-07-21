#include "JasMeshMarkAndSplit.h"

JasMeshMarkAndSplit::JasMeshMarkAndSplit()
{
}

JasMeshMarkAndSplit::~JasMeshMarkAndSplit()
{
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
