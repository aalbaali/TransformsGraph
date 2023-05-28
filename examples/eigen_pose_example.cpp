/**
 * @file eigen_pose_example.cpp
 * @brief Run an example using Eigen Affine2d
 * @author Amro Al-Baali
 * @date 2023-05-23
 */
#include <algorithm>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Include Eigen geometry
#include <Eigen/Geometry>

#include "transforms_graph/transforms_graph.h"

class Pose {
 public:
  Pose() { pose_ = Eigen::Affine2d::Identity(); }
  Pose(const Eigen::Affine2d& pose) : pose_(pose) {}
  Pose(Eigen::Affine2d&& pose) : pose_(std::move(pose)) {}
  Pose(const Eigen::Vector2d& translation, double heading) {
    pose_.translation() = translation;
    pose_.linear() = Eigen::Rotation2Dd(heading).toRotationMatrix();
  }

  Pose inverse() const {
    Pose p;
    p.pose_ = std::move(pose_.inverse());
    return p;
  }
  Eigen::Affine2d Affine() const { return pose_; }
  Eigen::Matrix3d matrix() const { return pose_.matrix(); }

 private:
  Eigen::Affine2d pose_;
};

Pose operator*(const Pose& pose1, const Pose& pose2) { return pose1.Affine() * pose2.Affine(); }
std::ostream& operator<<(std::ostream& os, const Pose& pose) {
  os << pose.matrix();
  return os;
}

// using Frame = int;
// using Frame = char;
enum class Frame { MAP, ODOM, BASE_LINK, CAMERA, FRONT_LIDAR };
std::ostream& operator<<(std::ostream& os, const Frame& frame) {
  return os << static_cast<int>(frame);
}

int main(int argc, char* argv[]) {

  tg::TransformsGraph<Pose, Frame> transforms;
  transforms.AddTransform(Frame::MAP, Frame::ODOM, Pose({1, 2}, 0.0));
  transforms.AddTransform(Frame::BASE_LINK, Frame::CAMERA, Pose({0.5, 0.0}, 0.0));
  transforms.AddTransform(Frame::BASE_LINK, Frame::FRONT_LIDAR, Pose({2, 1}, M_PI_4));

  std::unordered_map<Frame, std::string> frame_names = {{Frame::MAP, "Map"},
                                                        {Frame::ODOM, "Odom"},
                                                        {Frame::BASE_LINK, "Baselink"},
                                                        {Frame::CAMERA, "Camera"},
                                                        {Frame::FRONT_LIDAR, "Front_lidar"}};
  const auto get_frame_names = [&frame_names](Frame frame) { return frame_names[frame]; };
  std::cout << "TF before linking baselink and odom:\n"
            << transforms.GetMermaidGraph(get_frame_names)
            //<< transforms.GetMermaidGraph()
            << std::endl;

  // Add transform between already-existing frames
  transforms.AddTransform(Frame::ODOM, Frame::BASE_LINK, Pose({-1, 3}, M_PI_2));

  // Set BFS as the graph search algorithm
  // transforms.SetGraphSearchCallback(tg::BFS<char>);

  Frame start = Frame::ODOM;
  Frame end = Frame::CAMERA;

  std::cout << transforms.GetTransformChainString(start, end, true) << std::endl;
  std::cout << transforms.GetTransform(start, end).matrix() << std::endl;

  std::cout << transforms.GetMermaidGraph(get_frame_names) << std::endl;
}
