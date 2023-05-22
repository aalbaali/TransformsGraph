/**
 * @file transforms_graph.h
 * @brief Core of the transform graph
 * @author Amro Al-Baali
 * @date 2023-05-21
 */

// TODO: there needs to be a distinction between raw transforms and transforms that can be chained
// That is, if the graph is `a->b->c`, then `a->b` is a "raw transform", whereas `a->c` is a
// "chained transform" but not a "raw transform"
// This will make it easier to modify the graph

#ifndef TRANSFORMS_GRAPH_TRANSFORMS_GRAPH_H_
#define TRANSFORMS_GRAPH_TRANSFORMS_GRAPH_H_

#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "graph_search.h"

namespace tg {

// TODO: Should rename the class to 'TransformsGraph' (add 's' to 'Transform' to avoid confusion
// with an action ("Transform the graph"))
/**
 * @brief Transform graph class
 */
template <typename Transform, typename Frame = char>
class TransformsGraph {
 public:
  using TransformId = int;
  // Graph of frames and transforms
  using Transforms = std::unordered_map<TransformId, Transform>;
  using AdjacentFrames = std::unordered_map<Frame, std::unordered_set<Frame>>;

  /**
   * @brief Construct a new Transform Graph object
   *
   * @details The maximum number of frames cannot be changed after construction
   *
   * @param[in] max_frames Maximum number of frames allowed in the graph
   */
  TransformsGraph(int max_frames = 100) : max_frames_(max_frames) {}

  /**
   * @brief Get maximum number allowed in the transform graph
   *
   * @return Maximum number of frames allowed in the graph
   */
  int max_frames() const { return max_frames_; }

  // Adding a private section here to avoid declaring the functions used in the public
  // functions
  // private:
 public:
  /** Type of function that searches for a path between two frames in the graph */
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
  std::vector<Frame> GetTransformChain(Frame parent, Frame child) const {
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
  // TODO: Rename to 'HasTransform'?
  bool DoesTransformExist(Frame parent, Frame child) const {
    if (!IsFrameInGraph(parent)) return false;
    if (!IsFrameInGraph(child)) return false;
    return graph_search_callback_(adjacent_frames_, parent, child).size() > 0;
  }

  // TODO:
  // GetRawTransform(Frame parent, Frame child) const;
  // DoesRawTransformExist(Frame parent, Frame child) const;

  /**
   * @brief Get the transform between two frames. Throws an exception if no transform exists
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return
   */
  Transform GetTransform(Frame parent, Frame child) const {
    const auto path = GetTransformChain(parent, child);
    if (path.size() == 0) {
      throw std::runtime_error("No transform exists between the two frames");
    }

    // Transform starts as identity
    Transform T_parent_child;

    Frame prev = parent;
    for (const auto& frame : path) {
      if (frame == prev) continue;
      const auto transform_id = ComputeTransformId(prev, frame);
      auto T_prev_curr = transforms_.at(transform_id);
      if (ShouldInvertFrames(prev, frame)) {
        T_prev_curr = T_prev_curr.inverse();
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
  std::string GetTransformChainString(Frame parent, Frame child,
                                      bool show_transforms = false) const {
    const auto paths = GetTransformChain(parent, child);
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
        if (show_transforms) {
          ss_transforms << transforms_.at(ComputeTransformId(prev, frame)).inverse() << " -> ";
        }
      } else {
        ss_frames << "T_" << prev << "_" << frame;
        if (show_transforms) {
          ss_transforms << transforms_.at(ComputeTransformId(prev, frame)) << " -> ";
        }
      }
      ss_frames << " -> ";
      prev = frame;
    }
    if (show_transforms) {
      return "Chain: " + ss_frames.str() + "\nTransforms: " + ss_transforms.str();
    }
    return ss_frames.str();
  }

  /**
   * @brief Get a mermaid graph of the frames and transforms that can be viewed on Markdown files or
   * online (e.g., https://mermaid-js.github.io/mermaid-live-editor)
   *
   * @param[in] get_frame_name Function that returns a string (i.e., frame name) for a given frame
   * @param[in] show_edges If true, will show the transform between frames
   *
   * @return String of a mermaid graph. To insert in a Markdown file, then wrap with '```mermaid'
   * and '```'
   */
  std::string GetMermaidGraph(const std::function<std::string(Frame)>& get_frame_name,
                              bool show_edges = false) const {
    std::stringstream ss;
    ss << "graph TD" << std::endl;
    for (const auto& [frame, neighbours] : adjacent_frames_) {
      ss << "  " << get_frame_name(frame) << std::endl;
      for (const auto& neighbour : neighbours) {
        const auto transform_id = ComputeParentToChildId(frame, neighbour);
        if (!transforms_.count(transform_id)) continue;
        ss << "  " << get_frame_name(frame) << " --> ";
        if (show_edges) {
          ss << "| " << transforms_.at(transform_id) << " | ";
        }
        ss << get_frame_name(neighbour) << std::endl;
      }
    }
    return ss.str();
  }

  /**
   * @brief Get a mermaid graph of the frames and transforms using the default frame names (or
   * numbers)
   *
   * @param[in] show_edges If true, will show the transform between frames
   *
   * @return String of a mermaid graph. To insert in a Markdown file, then wrap with '```mermaid'
   * and '```'
   */
  std::string GetMermaidGraph(bool show_edges = false) const {
    const auto get_frame_name = [](Frame frame) {
      std::stringstream ss;
      ss << frame;
      return ss.str();
    };
    return GetMermaidGraph(get_frame_name, show_edges);
  }

  /**
   * @brief Add a transform to the graph. If a valid transform already exists in the graph and the
   * override is set to true, then it overrides the transform. Otherwise, it throws an exception.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   * @param[in] pose Transform from parent to child. That is, for a displacement `r_child` resolved
   * in the `child` frame, it can be resolved in the parent frame using `r_parent = pose * r_child`
   */
  void AddTransform(Frame parent, Frame child, const Transform& pose) {
    // Handle the case where the transform already exists in the graph
    if (DoesTransformExist(parent, child)) {
      // TODO: How to handle 'overriding' a transform that already exists in the graph?
      // Note that this is not a simple case of replacing the transform, since the transform
      // might be used in other transforms. For example, if we have the following transforms:
      //  'a'->'b'->'c'->'d', and we add the transform 'a'->'d', then what does "overriding" the
      //  transform really mean?
      //  To do this correctly, we need to remove one of the paths between 'a' and 'd' to break
      // the cycle, and then we can add 'a'->'d'. However, this is not trivial to do, so we
      // simply throw an exception for now.
      //
      // This could be done by overriding the transform if it explicitly exists in the frames (i.e.,
      // if there's a transform `T_parent_child` or `T_child_parent`)

      throw std::runtime_error("Transform already exists between the two frames");
    }

    // Add frame to the graph if it doesn't already exist
    if (!IsFrameInGraph(parent)) AddFrame(parent);
    if (!IsFrameInGraph(child)) AddFrame(child);

    adjacent_frames_[parent].insert(child);
    adjacent_frames_[child].insert(parent);

    InsertTransform(parent, child, pose);
  }

  void UpdateTransform(Frame parent, Frame child, const Transform& pose) {
    if (!DoesTransformExist(parent, child)) {
      throw std::runtime_error("Transform does not exist between the two frames");
    }
    InsertTransform(parent, child, pose);
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
  void InsertTransform(Frame parent, Frame child, const Transform& pose) {
    const auto transform_id = ComputeTransformId(parent, child);
    transforms_[transform_id] = ShouldInvertFrames(parent, child) ? pose.inverse() : pose;
  }

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

  /**
   * @brief Check if the transform should be inverted (internally) when stored in the transform
   * graph
   *
   * @details This is used to avoid storing the same transform twice. The transform is stored as
   * `T_a_b`, where `a` < `b`.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return True if the transform should be inverted (i.e., stored internally as `T_child_parent`)
   */
  bool ShouldInvertFrames(Frame parent, Frame child) const { return parent > child; }

  /**
   * @brief Compute a unique ID for a transform between two frames. The ID is independent of the
   * order of the frames
   *
   * @details The transform ID is stored as "aabb", where "aa" < "bb" (i.e., the frames are sorted)
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return Unique ID for the transform, which is in the form "aabb", where "aa" < "bb"
   */
  TransformId ComputeTransformId(Frame parent, Frame child) const {
    if (ShouldInvertFrames(parent, child)) {
      return ComputeParentToChildId(child, parent);
    } else {
      return ComputeParentToChildId(parent, child);
    }
  }

  /** Maximum number of frames expected to be in the graph */
  int max_frames_ = 100;

  /** Acyclic graph where the vertices are the frames and the edges are transforms between the two
   * frames */
  AdjacentFrames adjacent_frames_;

  /** Map of unique raw transforms between frames. The transform is stored as `T_a_b`, where `a <
   * b`. That is, if `T_a_b` exists, then `T_b_a` should not exist in the map. */
  Transforms transforms_;

  /** Function to call to search for a path between two frames in the graph */
  GraphSearchCallback graph_search_callback_ = DFS<Frame>;
};

}  // namespace tg
#endif // TRANSFORMS_GRAPH_TRANSFORMS_GRAPH_H_
