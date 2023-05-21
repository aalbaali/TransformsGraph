#include <algorithm>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "transform_graph/transform_graph.h"

// Fake pose class
class Pose {
 public:
  static Pose Identity() { return Pose(0); }

  Pose() = default;
  Pose(double x) : x_(x) {}
  double x() const { return x_; }
  void set_x(double x) { x_ = x; }

  Pose Inverse() const { return Pose(-x_); }

 private:
  double x_;
};

Pose operator*(const Pose& p1, const Pose& p2) { return Pose(p1.x() + p2.x()); }
Pose& operator*=(Pose& p1, const Pose& p2) { return p1 = p1 * p2; }
///////////////////////////////////////////////////////////////////////////////////////

// constexpr int MAX_FRAMES = 100;

// using Frame = int;
// using Frame = char;
enum class Frame { MAP, ODOM, BASE_LINK, CAMERA, FRONT_LIDAR };
std::ostream& operator<<(std::ostream& os, const Frame& frame) {
  return os << static_cast<int>(frame);
}

///////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  // Example of constructing a graph

  // Mermaid graph (visible on Markdown files or on
  // https://mermaid-js.github.io/mermaid-live-editor/)
  //```mermaid
  // graph LR;
  //  a --> c
  //  a --> b
  //  b --> d
  //```
  tg::TransformGraph<Pose, Frame> transforms;
  // transforms.AddTransform('a', 'b', 1);
  // transforms.AddTransform('a', 'c', 2);
  // transforms.AddTransform('b', 'd', 3);
  // transforms.AddTransform('e', 'f', 4);

  std::unordered_map<Frame, std::string> frame_names = {{Frame::MAP, "Map"},
                                                        {Frame::ODOM, "Odom"},
                                                        {Frame::BASE_LINK, "Baselink"},
                                                        {Frame::CAMERA, "Camera"},
                                                        {Frame::FRONT_LIDAR, "Front_lidar"}};

  transforms.AddTransform(Frame::MAP, Frame::ODOM, 1);
  transforms.AddTransform(Frame::BASE_LINK, Frame::CAMERA, 2);
  transforms.AddTransform(Frame::BASE_LINK, Frame::FRONT_LIDAR, 3);

  std::cout << "TF before linking baselink and odom:\n"
            //<< transforms.GetMermaidGraph(
            //       [&frame_names](Frame frame) { return frame_names[frame]; })
            << transforms.GetMermaidGraph() << std::endl;

  // Add transform between already-existing frames
  transforms.AddTransform(Frame::ODOM, Frame::BASE_LINK, 4);

  transforms.SetGraphSearchCallback(tg::BFS<Frame>);

  Frame start = Frame::ODOM;
  Frame end = Frame::CAMERA;

  // Example of using DFS
  std::vector<Frame> path = tg::DFS(transforms.GetGraph(), start, end);
  // std::vector<Frame> path = tg::BFS(transforms.GetGraph(), start, end);

  // Print the path
  std::cout << "Path: ";
  for (const auto& frame : path) {
    std::cout << frame << " ";
  }

  // if (!transforms.DoesTransformExist(start, end)) {
  //   std::cout << "Transform doesn't exist" << std::endl;
  //   return -1;
  // }

  // std::cout << transforms.GetTransformChain(start, end) << std::endl;
  // std::cout << transforms.GetTransform(start, end).x() << std::endl;

  //// std::cout << transforms.GetMermaidGraph(frame_names) << std::endl;
}
