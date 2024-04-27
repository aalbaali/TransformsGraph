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
set(CMAKE_EXPORT_DIR lib/cmake/TransformsGraph)
set(TARGETS_NAME "TransformsGraph")
set(TARGETS_NAMESPACE "TransformsGraph")
set(TARGETS_CONFIG_FILENAME "${TARGETS_NAME}Config.cmake.in")

install(
  TARGETS "${TARGETS_NAME}"
  EXPORT  "${TARGETS_NAME}Targets"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
)
install(EXPORT "${TARGETS_NAME}Targets" 
  NAMESPACE "${TARGETS_NAMESPACE}::"
  DESTINATION ${CMAKE_EXPORT_DIR}
)

# Install the include directory
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/
  DESTINATION include
)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${TARGETS_CONFIG_FILENAME}" "${CMAKE_CURRENT_BINARY_DIR}/${TARGETS_NAME}Config.cmake"
  INSTALL_DESTINATION "${CMAKE_EXPORT_DIR}"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${TARGETS_NAME}Config.cmake" DESTINATION "${CMAKE_EXPORT_DIR}")

# Export without installing
export(
  EXPORT TransformsGraphTargets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGETS_NAME}Targets.cmake"
  NAMESPACE "${TARGETS_NAMESPACE}::"
)
