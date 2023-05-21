#include <algorithm>
#include <iostream>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// Fake pose class
class Pose {
 public:
  Pose() = default;
  Pose(double x) : x_(x) {}
  double x() const { return x_; }
  void set_x(double x) { x_ = x; }

  Pose Inverse() const { return Pose(-x_); }

 private:
  double x_;
};

constexpr int MAX_FRAMES = 100;

// using Frame = int;
using Frame = char;
using TransformId = int;
using Transform = Pose;

// Graph of frames and transforms
using Transforms = std::unordered_map<TransformId, Transform>;
using AdjacentFrames = std::unordered_map<Frame, std::unordered_set<Frame>>;

/**
 * @brief Depth-first-search to find a path between two frames
 *
 * @param[in] graph Transform forest
 * @param[in] start Starting frame
 * @param[in] end Ending frame
 *
 * @return
 */
std::vector<Frame> DFS(const AdjacentFrames& graph, Frame start, Frame end) {
  std::vector<Frame> path;
  std::unordered_set<Frame> visited;
  std::unordered_map<Frame, Frame> parent;
  std::vector<Frame> stack;

  bool found_solution = false;
  stack.push_back(start);
  while (!stack.empty()) {
    Frame current = stack.back();
    stack.pop_back();

    // Found the end
    if (current == end) {
      found_solution = true;
      break;
    }
    if (visited.count(current)) continue;
    visited.insert(current);

    for (const auto& neighbour : graph.at(current)) {
      if (visited.count(neighbour)) continue;
      stack.push_back(neighbour);
      parent[neighbour] = current;
    }
  }

  if (!found_solution) return {};

  // Found the end
  Frame node = end;
  while (node != start) {
    path.push_back(node);
    node = parent[node];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * @brief Compute a unique ID for a transform between two frames
 *
 * @param[in] parent Parent frame
 * @param[in] child Child frame
 *
 * @return Unique ID for the transform
 */
TransformId ComputeParentToChildId(Frame parent, Frame child) {
  return parent * MAX_FRAMES + child;
}

bool ShouldInvertFrames(Frame parent, Frame child) { return parent > child; }

/**
 * @brief Compute a unique ID for a transform between two frames. The ID is independent of the order
 *
 * @param[in] parent
 * @param[in] child
 *
 * @return
 */
TransformId ComputeTransformId(Frame parent, Frame child) {
  if (ShouldInvertFrames(parent, child)) {
    return ComputeParentToChildId(child, parent);
  } else {
    return ComputeParentToChildId(parent, child);
  }
}

class TransformForest {
 public:
  /**
   * @brief Check if a frame is in the forest
   *
   * @param[in] frame Frame to check
   *
   * @return True if it's in the forest
   */
  bool IsFrameInForest(Frame frame) const {
    if (adjacent_frames_.count(frame) > 0) return true;

    // Otherwise, will need to check each frame neighbours
    for (const auto& [_, neighbours] : adjacent_frames_) {
      if (neighbours.count(frame) > 0) return true;
    }
    return false;
  }

  /**
   * @brief Get the path between two frames. Returns empty vector if no path exists
   *
   * @param[in] parent Frame to start from
   * @param[in] child Frame to end at
   *
   * @return Vector of paths from the parent to the child
   */
  std::vector<Frame> GetPath(Frame parent, Frame child) const {
    if (!IsFrameInForest(parent)) return {};
    if (!IsFrameInForest(child)) return {};

    return DFS(adjacent_frames_, parent, child);
  }

  /**
   * @brief Check if a valid transform exists in the forest
   *
   * @param[in] parent Frame to start from
   * @param[in] child Frame to end at
   *
   * @return
   */
  bool DoesTransformExist(Frame parent, Frame child) const {
    if (!IsFrameInForest(parent)) return false;
    if (!IsFrameInForest(child)) return false;
    return DFS(adjacent_frames_, parent, child).size() > 0;
  }

  /**
   * @brief Get the transform between two frames. Throws an exception if no transform exists
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return
   */
  // Transform GetTransform(Frame parent, Frame child) const {}
  std::string GetTransformChain(Frame parent, Frame child) const {
    const auto paths = GetPath(parent, child);
    if (paths.size() == 0) {
      throw std::runtime_error("No transform exists between the two frames");
    }

    std::stringstream ss;
    std::stringstream ss2;
    Frame prev = parent;
    for (const auto& frame : paths) {
      if (frame == prev) continue;
      if (ShouldInvertFrames(prev, frame)) {
        ss << "T_" << frame << "_" << prev << "^-1";
        ss2 << transforms_.at(ComputeTransformId(prev, frame)).Inverse().x() << " -> ";
      } else {
        ss << "T_" << prev << "_" << frame;
        ss2 << transforms_.at(ComputeTransformId(prev, frame)).x() << " -> ";
      }
      ss << " -> ";

      // Warn user if ID doesn't exist
      if (transforms_.count(ComputeTransformId(prev, frame)) == 0) {
        std::cout << "Warning: Transform ID doesn't exist" << std::endl;
      }

      prev = frame;
    }
    return ss.str() + "\n" + ss2.str();
  }

  /**
   * @brief Add a transform to the forest. If a valid transform already exists in the forest and the
   * override is set to true, then it overrides the transform. Otherwise, it throws an exception.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   * @param[in] pose Transform from parent to child. That is, for a displacement `r_child` resolved
   * in the `child` frame, it can be resolved in the parent frame using `r_parent = pose * r_child`
   * @param[in] override If true, overrides the transform if it already exists
   */
  void AddTransform(Frame parent, Frame child, const Pose& pose, bool override = false) {
    // Handle the case where the transform already exists in the forest
    if (DoesTransformExist(parent, child)) {
      if (!override) {
        throw std::runtime_error("Transform already exists between the two frames");
      }

      const auto transform_id = ComputeTransformId(parent, child);
      transforms_[transform_id] = ShouldInvertFrames(parent, child) ? pose.Inverse() : pose;
      return;
    }

    // Add frame to the forest if it doesn't already exist
    if (!IsFrameInForest(parent)) AddFrame(parent);
    if (!IsFrameInForest(child)) AddFrame(child);

    adjacent_frames_[parent].insert(child);
    adjacent_frames_[child].insert(parent);

    const auto transform_id = ComputeTransformId(parent, child);
    transforms_[transform_id] = ShouldInvertFrames(parent, child) ? pose.Inverse() : pose;
  }

  /**
   * @brief Adds frame into the forest, if it doesn't already exist.
   *
   * @param[in] frame Frame to add to the forest
   */
  void AddFrame(Frame frame) noexcept {
    if (IsFrameInForest(frame)) return;
    adjacent_frames_.emplace(frame, std::unordered_set<Frame>());
  }

  const AdjacentFrames& GetForest() const noexcept { return adjacent_frames_; }

 private:
  /** Acyclic graph where the vertices are the frames and the edges are transforms between the two
   * frames */
  AdjacentFrames adjacent_frames_;

  /** Map of unique raw transforms between frames. The transform is stored as `T_a_b`, where `a <
   * b`. That is, if `T_a_b` exists, then `T_b_a` should not exist in the map. */
  Transforms transforms_;
};

int main(int argc, char* argv[]) {
  //// Example of constructing a graph
  // AdjacentFrames graph;
  // graph.emplace(0, std::unordered_set<Frame>{1, 2});
  // graph.emplace(1, std::unordered_set<Frame>{0});
  // graph.emplace(2, std::unordered_set<Frame>{0});

  //// Second frame
  // graph.at(1).insert(3);
  // graph.emplace(3, std::unordered_set<Frame>{1});
  TransformForest transforms;
  // transforms.AddTransform(0, 1, 1);
  // transforms.AddTransform(0, 2, 2);
  // transforms.AddTransform(1, 3, 3);
  transforms.AddTransform('a', 'b', 1);
  transforms.AddTransform('a', 'c', 2);
  transforms.AddTransform('b', 'd', 3);

  Frame start = 'c';
  Frame end = 'd';

  //// Example of using DFS
  // std::vector<Frame> path = DFS(transforms.GetForest(), start, end);

  //// Print the path
  // std::cout << "Path: ";
  // for (const auto& frame : path) {
  //   std::cout << frame << " ";
  // }

  std::cout << transforms.GetTransformChain(start, end) << std::endl;
}
