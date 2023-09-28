cmake_minimum_required(VERSION 3.16)
project(TransformsGraph VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-std=c++11")

# Add header-only library
add_library(TransformsGraph INTERFACE)
add_library(TransformsGraph::TransformsGraph ALIAS TransformsGraph)
target_include_directories(TransformsGraph INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
    $<INSTALL_INTERFACE:include>
)


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

################################################
# Installation
################################################
# Export package for use from the build tree
set(TRANSFORMS_GRAPH_CMAKE_EXPORT_DIR lib/cmake/TransformsGraph)

install(TARGETS TransformsGraph
        EXPORT  TransformsGraphTargets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)
install(EXPORT TransformsGraphTargets 
  NAMESPACE TransformsGraph::
  DESTINATION ${TRANSFORMS_GRAPH_CMAKE_EXPORT_DIR}
)

# Install the include directory
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
      DESTINATION include)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"cmake/TransformsGraphConfig.cmake.in" "${CMAKE_CURRENT_BINARY_DIR}/TransformsGraphConfig.cmake"
	INSTALL_DESTINATION "lib/cmake/TransformsGraph"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/TransformsGraphConfig.cmake" DESTINATION "lib/cmake/TransformsGraph")

