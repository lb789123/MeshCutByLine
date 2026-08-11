cmake_minimum_required(VERSION 3.15)
project(MeshCutByLine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix MSVC encoding issues with Chinese characters in comments
if(MSVC)
    add_compile_options(/utf-8)
endif()

# vcglib include path
include_directories(${CMAKE_SOURCE_DIR}/vcglib)
# Eigen include path (used by vcglib)
include_directories(${CMAKE_SOURCE_DIR}/vcglib/eigenlib)
# project root for tool/ headers
include_directories(${CMAKE_SOURCE_DIR})

# 外部局部切割库（CGAL corefine）——cgalLocalMeshCut submodule
set(CGLMCUT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(external/cgalLocalMeshCut)
include_directories(${CMAKE_SOURCE_DIR}/external/cgalLocalMeshCut)

# test executable
add_executable(test_mesh_cut
    tests/test_mesh_cut.cpp
    JasMeshMarkAndCutSplit.cpp
    tool/cmesh.cpp
)
target_link_libraries(test_mesh_cut PRIVATE cglmcut)
