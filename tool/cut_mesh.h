#include "cmesh.h"
#include <vector>
#include <string>
#include <map>

class JasMeshAddCutLines
{

public:
    JasMeshAddCutLines();
    ~JasMeshAddCutLines();

    void AddCutLines(CMeshOD *pMesh,vcg::Point3d &normal,std::vector<vcg::Point3d> &line,std::vector<int> &cutLine);
};

inline JasMeshAddCutLines::JasMeshAddCutLines(/* args */)
{
}

inline JasMeshAddCutLines::~JasMeshAddCutLines()
{
}
