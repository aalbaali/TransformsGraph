#include <algorithm>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Include Eigen geometry
#include <Eigen/Geometry>

#include "transform_graph/abstract_pose.h"
#include "transform_graph/transform_graph.h"

//// Fake pose class
// class Pose : public tg::AbstractPose<Pose> {
//  public:
//   Pose() = default;
//   Pose(double x) : x_(x) {}
//   double x() const { return x_; }
//   void set_x(double x) { x_ = x; }

//  Pose inverse() const override { return Pose(-x_); }

// private:
//  double x_;
//};

class Pose : public tg::AbstractPose<Pose> {
 public:
  Pose() { pose_ = Eigen::Affine2d::Identity(); }
  Pose(const Eigen::Affine2d& pose) : pose_(pose) {}
  Pose(Eigen::Affine2d&& pose) : pose_(std::move(pose)) {}
  Pose(const Eigen::Vector2d& translation, double heading) {
    pose_.translation() = translation;
    pose_.linear() = Eigen::Rotation2Dd(heading).toRotationMatrix();
  }

  Pose inverse() const override {
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

// Pose operator*(const Pose& p1, const Pose& p2) { return Pose(p1.x() + p2.x()); }
// Pose& operator*=(Pose& p1, const Pose& p2) { return p1 = p1 * p2; }

// using Frame = int;
// using Frame = char;
enum class Frame { MAP, ODOM, BASE_LINK, CAMERA, FRONT_LIDAR };
std::ostream& operator<<(std::ostream& os, const Frame& frame) {
  return os << static_cast<int>(frame);
}

int main(int argc, char* argv[]) {
  // Eigen::Affine2d p2;
  // p1.translation() = Eigen::Vector2d(1, 2);
  // p1.linear() = Eigen::Rotation2Dd(0.0).toRotationMatrix();
  // p2.translation() = Eigen::Vector2d(3, 1);
  // p2.linear() = Eigen::Rotation2Dd(M_PI_4).toRotationMatrix();

  // std::cout << p1.matrix() << std::endl;
  // std::cout << p2.matrix() << std::endl;
  // std::cout << (p1 * p2).matrix() << std::endl;

  //// Invert p1 matrix
  // std::cout << p2.inverse().matrix() << std::endl;

  // Example of constructing a graph
  // tg::TransformGraph<Pose, Frame> transforms;
  // transforms.AddTransform('a', 'b', 1);
  // transforms.AddTransform('a', 'c', 2);
  // transforms.AddTransform('b', 'd', 3);
  // transforms.AddTransform('e', 'f', 4);

  std::unordered_map<Frame, std::string> frame_names = {{Frame::MAP, "Map"},
                                                        {Frame::ODOM, "Odom"},
                                                        {Frame::BASE_LINK, "Baselink"},
                                                        {Frame::CAMERA, "Camera"},
                                                        {Frame::FRONT_LIDAR, "Front_lidar"}};

  tg::TransformGraph<Pose, Frame> transforms;
  transforms.AddTransform(Frame::MAP, Frame::ODOM, Pose({1, 2}, 0.0));
  transforms.AddTransform(Frame::BASE_LINK, Frame::CAMERA, Pose({0.5, 0.0}, 0.0));
  transforms.AddTransform(Frame::BASE_LINK, Frame::FRONT_LIDAR, Pose({2, 1}, M_PI_4));

  const auto get_frame_names = [&frame_names](Frame frame) { return frame_names[frame]; };
  std::cout << "TF before linking baselink and odom:\n"
            << transforms.GetMermaidGraph(get_frame_names)
            //<< transforms.GetMermaidGraph()
            << std::endl;

  // Add transform between already-existing frames
  transforms.AddTransform(Frame::ODOM, Frame::BASE_LINK, Pose({-1, 3}, M_PI_2));

  // transforms.SetGraphSearchCallback(tg::BFS<char>);

  Frame start = Frame::ODOM;
  Frame end = Frame::CAMERA;

  ////// Example of using DFS
  //// std::vector<Frame> path = tg::DFS(transforms.GetGraph(), start, end);
  ////// std::vector<Frame> path = tg::BFS(transforms.GetGraph(), start, end);

  //// Print the path
  // std::cout << "Path: ";
  // for (const auto& frame : path) {
  //   std::cout << frame << " ";
  // }

  //// if (!transforms.DoesTransformExist(start, end)) {
  ////   std::cout << "Transform doesn't exist" << std::endl;
  ////   return -1;
  //// }

  std::cout << transforms.GetTransformChain(start, end, true) << std::endl;
  std::cout << transforms.GetTransform(start, end).matrix() << std::endl;

  std::cout << transforms.GetMermaidGraph(get_frame_names) << std::endl;
}
