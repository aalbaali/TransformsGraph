cmake_minimum_required(VERSION 3.16)
project(TransformsGraph)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11")

# Add header-only library
add_library(TransformsGraph INTERFACE)
target_include_directories(TransformsGraph INTERFACE include)

# Find Eigen
find_package(Eigen3 REQUIRED)

add_executable(main main.cpp)
target_link_libraries(main PUBLIC TransformsGraph Eigen3::Eigen)
target_include_directories(main INTERFACE ${EIGEN3_INCLUDE_DIR})

# Add option to build examples
option(BUILD_EXAMPLES "Build examples" OFF)
if(BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

enable_testing()
include(CTest)
# Add option to build tests
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()
