find_package(Eigen3)
find_package(Sophus 1.22.10)

add_executable(minimal_graph_example minimal_graph_example.cpp)
target_link_libraries(minimal_graph_example TransformsGraph)

if (Eigen3_FOUND)
  message(STATUS "Eigen3 found, building eigen_pose_example")

  add_executable(eigen_pose_example eigen_pose_example.cpp)
  target_link_libraries(eigen_pose_example PUBLIC TransformsGraph Eigen3::Eigen)

  if (Sophus_FOUND)
    message(STATUS "Sophus found, building sophus_pose_example")

    add_executable(sophus_pose_example sophus_pose_example.cpp)
    target_link_libraries(sophus_pose_example PUBLIC TransformsGraph Sophus::Sophus Eigen3::Eigen)
  else()
    message(STATUS "Sophus not found, not building sophus_pose_example")
  endif()

else()
  message(STATUS "Eigen3 not found, not building eigen_pose_example")
endif()

