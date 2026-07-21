#include <tool/cmesh.h>
#include <vector>
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

	//返回分割后的多个三维多边形
	void SplitMeshByMarkAndEdge(std::vector<splitReg> &retRegs);
private:
	CMeshO* m_pMesh;
	std::vector<vcg::Point3i> m_edgeMarks;//对应是否是分割边
};
