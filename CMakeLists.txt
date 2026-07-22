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

# test executable
add_executable(test_mesh_cut
    tests/test_mesh_cut.cpp
    JasMeshMarkAndCutSplit.cpp
    tool/cmesh.cpp
)
