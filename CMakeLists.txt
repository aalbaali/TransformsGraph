cmake_minimum_required(VERSION 3.10)
project(TransformGraph)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17")

# Add header-only library
add_library(TransformGraph INTERFACE)
target_include_directories(TransformGraph INTERFACE include)

# Find Eigen
find_package(Eigen3 REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC TransformGraph Eigen3::Eigen)
target_include_directories(main INTERFACE ${EIGEN3_INCLUDE_DIR})
