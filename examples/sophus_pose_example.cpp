/**
 * @file sophus_pose_example.cpp
 * @brief Run an example using Sophus SE2
 * @author Amro Al-Baali
 * @date 2023-05-23
 */
#include <Eigen/Dense>
#include <iostream>
#include <sophus/se2.hpp>

#include "transforms_graph/transforms_graph.h"

// This class satisfies the requirements on the Transform class (i.e., it has `*` overloaded, and
// has an `inverse()` function)
using Transform = Sophus::SE2d;

// Overriding the `<<` operator is necessary
std::ostream& operator<<(std::ostream& os, Transform transform) {
  os << "x: " << transform.translation().x() << ", y: " << transform.translation().y()
     << ", theta: " << transform.so2().log();
  return os;
}

enum class Frame { A, B, C, D, E, F };

// Overriding the `<<` operator necessary for using the `GetMermaidGraph(true)` function
std::ostream& operator<<(std::ostream& os, const Frame& frame) {
  return os << static_cast<char>('A' + (int)frame);
}

int main(int argc, char* argv[]) {
  // Construct a graph that consists of two unconnected subgraphs
  tg::TransformsGraph<Transform, Frame> transforms;
  transforms.AddTransform(Frame::A, Frame::B, Transform(M_PI_4, {0, 0}));
  transforms.AddTransform(Frame::A, Frame::C, Transform(M_PI_2, {0, 0}));
  transforms.AddTransform(Frame::B, Frame::D, Transform(0, {1, 2}));

  // Add Frame::E and Frame::F such that they are in a subgraph that is not connected to the rest of
  // the graph
  transforms.AddTransform(Frame::E, Frame::F, Transform(-M_PI_4, {3, 4}));

  // Resolve a vector in a different frame
  const auto T_a_b = transforms.GetTransform(Frame::A, Frame::B);
  const Eigen::Vector2d v_b{1, 0};
  const Eigen::Vector2d v_a = T_a_b * v_b;
  std::cout << "[" << v_a.transpose() << "]_a = [" << v_b.transpose() << "]_b" << std::endl;

  std::unordered_map<Frame, std::string> frame_names = {{Frame::A, "Map"},
                                                        {Frame::B, "Odom"},
                                                        {Frame::C, "Baselink"},
                                                        {Frame::D, "Camera"},
                                                        {Frame::E, "Front_lidar"},
                                                        {Frame::F, "Rear_lidar"}};
  const auto get_frame_names = [&frame_names](Frame frame) { return frame_names[frame]; };
  // Visualize the graph using mermaid graph. Copy the output and run on
  // https://mermaid-js.github.io/mermaid-live-editor
  // You should see the two unconnected subgraphs
  std::cout << transforms.GetMermaidGraph(get_frame_names, false) << std::endl;

  // There are some ways to check if a transform exists between two frames
  std::cout << "Does a->b exist? " << transforms.HasTransform(Frame::A, Frame::B) << std::endl;
  std::cout << "Does a->d exist? " << transforms.HasTransform(Frame::A, Frame::D) << std::endl;
  std::cout << "Does a->f exist? " << transforms.HasTransform(Frame::A, Frame::F) << std::endl;

  // Now let's connect the two subgraphs
  transforms.AddTransform(Frame::C, Frame::F, Transform(0, {1, 0}));

  // Visualize the graph with edges
  std::cout << transforms.GetMermaidGraph(true) << std::endl;
  std::cout << "Does a->f exist? " << transforms.HasTransform(Frame::A, Frame::F) << std::endl;

  // One can visualize the path between two frames. This should match what you see on the graph
  std::cout << "Path from a->e: " << transforms.GetTransformChainString(Frame::A, Frame::E, true)
            << std::endl;

  // Get the transform from a->e. This should match the summation (because we defined the `*`
  // operator as a summation) from the path above
  std::cout << "Transform from a->e: " << transforms.GetTransform(Frame::A, Frame::E) << std::endl;

  // Adding a transform to an already existing path (i.e., transform) should throw an error
  // This is because the graph is acyclic and adding a transform to an existing path would
  // create a cycle
  try {
    transforms.AddTransform(Frame::A, Frame::F, Transform(0, {0, 1}));
  } catch (const std::runtime_error& e) {
    std::cout << "Caught exception: " << e.what() << std::endl;
  }

  // Removing a frame from the graph should delete all the transforms that are connected to it
  transforms.RemoveFrame(Frame::A);
  std::cout << "Graph after removing frame Frame::A\n"
            << transforms.GetMermaidGraph(true) << std::endl;

  // Removing a raw transform from the graph removes the transform but not the frames
  transforms.RemoveRawTransform(Frame::F, Frame::C);
  std::cout << "Graph after removing 'f->c' transform\n"
            << transforms.GetMermaidGraph(true) << std::endl;
}
