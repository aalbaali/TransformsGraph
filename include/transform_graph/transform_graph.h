/** 
* @file transform_graph.h
* @brief Core of the transform graph
* @author Amro Al-Baali
* @date 2023-05-21
*/

#ifndef TRANSFORM_GRAPH_TRANSFORM_GRAPH_H_
#define TRANSFORM_GRAPH_TRANSFORM_GRAPH_H_

#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "graph_search.h"

namespace tg {
// TODO:
// - Templetize the frame type
// - Templetize the transform type
// - Move the ComputeTransformId into the TransformGraph class
template <typename Transform, typename Frame = char>
class TransformGraph {
 public:
  using TransformId = int;
  // Graph of frames and transforms
  using Transforms = std::unordered_map<TransformId, Transform>;
  using AdjacentFrames = std::unordered_map<Frame, std::unordered_set<Frame>>;

  TransformGraph(int max_frames = 100) : max_frames_(max_frames) {}

  /**
   * @brief Get maximum number allowed in the transform graph
   *
   * @return
   */
  int max_frames() const { return max_frames_; }

  // TEMPORARY: Adding a private section here to avoid declaring the functions used in the public
  // functions
 private:
  /**
   * @brief Compute a unique ID for a transform between two frames
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return Unique ID for the transform
   */
  TransformId ComputeParentToChildId(Frame parent, Frame child) const {
    return static_cast<TransformId>(parent) * max_frames_ + static_cast<TransformId>(child);
  }

  bool ShouldInvertFrames(Frame parent, Frame child) const { return parent > child; }

  /**
   * @brief Compute a unique ID for a transform between two frames. The ID is independent of the
   * order
   *
   * @param[in] parent
   * @param[in] child
   *
   * @return Unique ID for the transform
   */
  TransformId ComputeTransformId(Frame parent, Frame child) const {
    if (ShouldInvertFrames(parent, child)) {
      return ComputeParentToChildId(child, parent);
    } else {
      return ComputeParentToChildId(parent, child);
    }
  }

 public:
  using GraphSearchCallback =
      std::function<std::vector<Frame>(const AdjacentFrames&, Frame, Frame)>;

  /**
   * @brief Check if a frame is in the graph
   *
   * @param[in] frame Frame to check
   *
   * @return True if it's in the graph
   */
  bool IsFrameInGraph(Frame frame) const {
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
    if (!IsFrameInGraph(parent)) return {};
    if (!IsFrameInGraph(child)) return {};

    return graph_search_callback_(adjacent_frames_, parent, child);
  }

  /**
   * @brief Check if a valid transform exists in the graph
   *
   * @param[in] parent Frame to start from
   * @param[in] child Frame to end at
   *
   * @return
   */
  bool DoesTransformExist(Frame parent, Frame child) const {
    if (!IsFrameInGraph(parent)) return false;
    if (!IsFrameInGraph(child)) return false;
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
    Transform T_parent_child = Transform::Identity();

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

    std::stringstream ss_frames;
    std::stringstream ss_transforms;
    Frame prev = parent;
    for (const auto& frame : paths) {
      if (frame == prev) continue;
      if (ShouldInvertFrames(prev, frame)) {
        ss_frames << "T_" << frame << "_" << prev << "^-1";
        ss_transforms << transforms_.at(ComputeTransformId(prev, frame)).Inverse().x() << " -> ";
      } else {
        ss_frames << "T_" << prev << "_" << frame;
        ss_transforms << transforms_.at(ComputeTransformId(prev, frame)).x() << " -> ";
      }
      ss_frames << " -> ";

      prev = frame;
    }
    return ss_frames.str() + "\n" + ss_transforms.str();
  }

  /**
   * @brief Get a mermiad graph of the graph
   *
   * @return String of the mermaid graph. It can be viewed on
   * https://mermaid-js.github.io/mermaid-live-editor or on markdown files by wrapping it in
   * ```mermaid and ```
   */
  // std::string GetMermaidGraph() const {
  //   std::stringstream ss;
  //   ss << "graph TD" << std::endl;
  //   for (const auto& [frame, neighbours] : adjacent_frames_) {
  //     ss << "  " << frame << std::endl;
  //     for (const auto& neighbour : neighbours) {
  //       ss << "  " << frame << " --> " << neighbour << std::endl;
  //     }
  //   }
  //   return ss.str();
  // }

  // std::string GetMermaidGraph(const std::unordered_map<Frame, std::string>& frame_names) const {
  //   std::stringstream ss;
  //   ss << "graph TD" << std::endl;
  //   for (const auto& [frame, neighbours] : adjacent_frames_) {
  //     ss << "  " << frame_names.at(frame) << std::endl;
  //     for (const auto& neighbour : neighbours) {
  //       ss << "  " << frame_names.at(frame) << " --> " << frame_names.at(neighbour) << std::endl;
  //     }
  //   }
  //   return ss.str();
  // }

  std::string GetMermaidGraph(const std::function<std::string(Frame)>& get_frame_name) const {
    std::stringstream ss;
    ss << "graph TD" << std::endl;
    for (const auto& [frame, neighbours] : adjacent_frames_) {
      ss << "  " << get_frame_name(frame) << std::endl;
      for (const auto& neighbour : neighbours) {
        ss << "  " << get_frame_name(frame) << " --> " << get_frame_name(neighbour) << std::endl;
      }
    }
    return ss.str();
  }

  std::string GetMermaidGraph() const {
    const auto get_frame_name = [](Frame frame) {
      std::stringstream ss;
      ss << frame;
      return ss.str();
    };
    return GetMermaidGraph(get_frame_name);
  }

  /**
   * @brief Add a transform to the graph. If a valid transform already exists in the graph and the
   * override is set to true, then it overrides the transform. Otherwise, it throws an exception.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   * @param[in] pose Transform from parent to child. That is, for a displacement `r_child` resolved
   * in the `child` frame, it can be resolved in the parent frame using `r_parent = pose * r_child`
   * @param[in] override If true, overrides the transform if it already exists
   */
  void AddTransform(Frame parent, Frame child, const Transform& pose, bool override = false) {
    // Handle the case where the transform already exists in the graph
    if (DoesTransformExist(parent, child)) {
      if (!override) {
        throw std::runtime_error("Transform already exists between the two frames");
      }

      const auto transform_id = ComputeTransformId(parent, child);
      transforms_[transform_id] = ShouldInvertFrames(parent, child) ? pose.Inverse() : pose;
      return;
    }

    // Add frame to the graph if it doesn't already exist
    if (!IsFrameInGraph(parent)) AddFrame(parent);
    if (!IsFrameInGraph(child)) AddFrame(child);

    adjacent_frames_[parent].insert(child);
    adjacent_frames_[child].insert(parent);

    const auto transform_id = ComputeTransformId(parent, child);
    transforms_[transform_id] = ShouldInvertFrames(parent, child) ? pose.Inverse() : pose;
  }

  /**
   * @brief Adds frame into the graph, if it doesn't already exist.
   *
   * @param[in] frame Frame to add to the graph
   */
  void AddFrame(Frame frame) noexcept {
    if (IsFrameInGraph(frame)) return;
    adjacent_frames_.emplace(frame, std::unordered_set<Frame>());
  }

  const AdjacentFrames& GetGraph() const noexcept { return adjacent_frames_; }

  void SetGraphSearchCallback(GraphSearchCallback callback) noexcept {
    graph_search_callback_ = callback;
  }

 private:
  /** Maximum number of frames expected to be in the graph */
  int max_frames_ = 100;

  /** Acyclic graph where the vertices are the frames and the edges are transforms between the two
   * frames */
  AdjacentFrames adjacent_frames_;

  /** Map of unique raw transforms between frames. The transform is stored as `T_a_b`, where `a <
   * b`. That is, if `T_a_b` exists, then `T_b_a` should not exist in the map. */
  Transforms transforms_;

  /** Function to call to search for a path between two frames in the graph */
  // GraphSearchCallback graph_search_callback_ = DFS;
  GraphSearchCallback graph_search_callback_ = DFS<Frame>;
};

}  // namespace tg
#endif  // TRANSFORM_GRAPH_TRANSFORM_GRAPH_H_
