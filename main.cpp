#include <algorithm>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <vector>

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

std::vector<Frame> BFS(const AdjacentFrames& graph, Frame start, Frame end) {
  std::vector<Frame> path;
  std::unordered_set<Frame> visited;
  std::unordered_map<Frame, Frame> parent;
  std::queue<Frame> queue;

  bool found_solution = false;
  queue.push(start);
  visited.insert(start);

  while (!queue.empty()) {
    Frame current = queue.front();
    queue.pop();

    // Found the end
    if (current == end) {
      found_solution = true;
      break;
    }

    for (const auto& neighbour : graph.at(current)) {
      if (visited.count(neighbour)) continue;
      visited.insert(neighbour);
      queue.push(neighbour);
      parent[neighbour] = current;
    }
  }

  if (!found_solution) return {};

  // Reconstruct the path
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
  using GraphSearchCallback =
      std::function<std::vector<Frame>(const AdjacentFrames&, Frame, Frame)>;

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

    return graph_search_callback_(adjacent_frames_, parent, child);
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
    return graph_search_callback_(adjacent_frames_, parent, child).size() > 0;
  }

  /**
   * @brief Get the transform between two frames. Throws an exception if no transform exists
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return
   */
  Transform GetTransform(Frame parent, Frame child) const {
    const auto path = GetPath(parent, child);
    if (path.size() == 0) {
      throw std::runtime_error("No transform exists between the two frames");
    }
    Pose T_parent_child = Pose::Identity();

    Frame prev = parent;
    for (const auto& frame : path) {
      if (frame == prev) continue;
      const auto transform_id = ComputeTransformId(prev, frame);
      auto T_prev_curr = transforms_.at(transform_id);
      if (ShouldInvertFrames(prev, frame)) {
        T_prev_curr = T_prev_curr.Inverse();
      }

      T_parent_child = T_parent_child * T_prev_curr;

      prev = frame;
    }

    return T_parent_child;
  }

  /**
   * @brief Get a string showing the transform chain between two frames
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return
   */
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
   * @brief Get a mermiad graph of the forest
   *
   * @return String of the mermaid graph. It can be viewed on
   * https://mermaid-js.github.io/mermaid-live-editor or on markdown files by wrapping it in
   * ```mermaid and ```
   */
  std::string GetMermaidGraph() const {
    std::stringstream ss;
    ss << "graph TD" << std::endl;
    for (const auto& [frame, neighbours] : adjacent_frames_) {
      ss << "  " << frame << std::endl;
      for (const auto& neighbour : neighbours) {
        ss << "  " << frame << " --> " << neighbour << std::endl;
      }
    }
    return ss.str();
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

  void SetGraphSearchCallback(GraphSearchCallback callback) noexcept {
    graph_search_callback_ = callback;
  }

 private:
  /** Acyclic graph where the vertices are the frames and the edges are transforms between the two
   * frames */
  AdjacentFrames adjacent_frames_;

  /** Map of unique raw transforms between frames. The transform is stored as `T_a_b`, where `a <
   * b`. That is, if `T_a_b` exists, then `T_b_a` should not exist in the map. */
  Transforms transforms_;

  /** Function to call to search for a path between two frames in the forest */
  GraphSearchCallback graph_search_callback_ = DFS;
};

int main(int argc, char* argv[]) {
  // Example of constructing a graph

  // Mermaid graph (visible on Markdown files or on
  // https://mermaid-js.github.io/mermaid-live-editor/)
  //```mermaid
  // graph LR;
  //  a --> b
  //  a --> c
  //  b --> d
  //```
  TransformForest transforms;
  transforms.AddTransform('a', 'b', 1);
  transforms.AddTransform('a', 'c', 2);
  transforms.AddTransform('b', 'd', 3);
  transforms.AddTransform('e', 'f', 4);

  // Add transform between already-existing frames
  transforms.AddTransform('a', 'e', 5);

  transforms.SetGraphSearchCallback(BFS);

  Frame start = 'd';
  Frame end = 'f';

  //// Example of using DFS
  ////std::vector<Frame> path = DFS(transforms.GetForest(), start, end);
  // std::vector<Frame> path = BFS(transforms.GetForest(), start, end);

  //// Print the path
  // std::cout << "Path: ";
  // for (const auto& frame : path) {
  //   std::cout << frame << " ";
  // }

  if (!transforms.DoesTransformExist(start, end)) {
    std::cout << "Transform doesn't exist" << std::endl;
    return -1;
  }

  std::cout << transforms.GetTransformChain(start, end) << std::endl;
  std::cout << transforms.GetTransform(start, end).x() << std::endl;

  std::cout << transforms.GetMermaidGraph() << std::endl;
}
