enable_testing()
include(CTest)

find_package(GTest REQUIRED)

add_executable(test_transforms_graph  test_transforms_graph.cpp)
target_link_libraries(test_transforms_graph TransformsGraph ${GTEST_LIBRARIES} pthread)
add_test(test_transforms_graph "../tests/test_transforms_graph")
