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
#ifndef JASMESHMARKANDCUTSPLIT_H
#define JASMESHMARKANDCUTSPLIT_H

#include <tool/cmesh.h>
#include <vector>
#include <string>
#include <map>
#include "tool/edge_info.h"
#include "tool/polyline.h"
#include "tool/cut_plane.h"
#include "tool/region_marker.h"
#include "tool/local_mesh_cut.h"

class JasMeshMarkAndCutSplit
{
public:
    struct splitReg
    {
        int mark;                    // 原始平面标记
        int newMark;                 // 新标记（简单多边形 ID）
        std::vector<int> inTris;     // 包含的三角形索引
        vcg::Point3d normal;         // 法向量
        std::vector<int> boundlines; // 边界边的顶点索引序列
        std::vector<std::vector<int>> boundaries; // 全部边界环（第 0 圈外圈，其余为洞）
    };

    // 构造函数
    JasMeshMarkAndCutSplit();

    // 析构函数
    ~JasMeshMarkAndCutSplit();

    // 设置 mesh，并且已经根据 mark 标记了三角形归属的每个平面
    void SetMainMesh(CMeshOD* pMesh)
    {
        m_pMesh = pMesh;
    }

    // 构建边信息（在 SetMainMesh 之后调用）
    void BuildEdgeInfo()
    {
        m_edgeInfoManager.buildEdgeInfo(m_pMesh);
    }

    // 设置调试输出开关
    void SetDebug(bool enable)
    {
        m_debug = enable;
    }

    // 设置调试输出目录
    void SetDebugOutputDir(const std::string& dir)
    {
        m_debugOutputDir = dir;
    }

    // 返回分割后的多个三维多边形
    void SplitMeshByMarkAndEdge(std::vector<splitReg>& retRegs);

    // 查找给定面集合中的切割边
    std::vector<MeshCutByMark::CutEdge> findCutEdges(const std::vector<int>& curFaces);

    // 提取区域的边界边序列
    std::vector<std::vector<int>> extractBoundaryEdges(const std::vector<int>& regionFaces);

private:
    // 确保调试输出目录存在
    void debugEnsureDir();

    // 将折线写入 OBJ 调试文件
    void debugWritePolylines(int iterIdx, const std::vector<MeshCutByMark::Polyline>& polylines);

    // 将面集合写入 OFF 调试文件
    void debugWriteFacesOFF(int iterIdx, const char* suffix, const std::vector<int>& faceIndices);

    // 将多个子区域写入 OFF 调试文件
    void debugWriteSubRegionsOFF(int iterIdx, const std::vector<std::vector<int>>& subRegions);

    // 将最终多边形写入 OBJ 调试文件
    void debugWritePolygonsOBJ(const std::map<int, std::vector<int>>& markToFaces);

    // 保存带随机颜色的网格 OBJ 调试文件
    void debugSaveColoredMesh(const std::vector<splitReg>& regs);

    CMeshOD* m_pMesh = nullptr;
    MeshCutByMark::EdgeInfoManager m_edgeInfoManager; // 边信息管理器
    MeshCutByMark::PolylineManager m_polylineManager; // 折线管理器
    MeshCutByMark::RegionMarker m_regionMarker;       // 区域标记管理器
    MeshCutByMark::LocalMeshCutManager m_localMeshCut; // 局部 mesh 切割管线
    int m_newMarkCounter = 0;                          // 新区域标记计数器
    std::vector<vcg::Point3i> m_edgeMarks;             // 对应是否是分割边

    bool m_debug = false;                              // 调试输出开关
    std::string m_debugOutputDir = "debug_output/";    // 调试输出目录
    int m_debugIterCounter = 0;                        // 主循环迭代计数器
};

#endif // JASMESHMARKANDCUTSPLIT_H
