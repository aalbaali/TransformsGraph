/**
 * @file transforms_graph.h
 * @brief Core of the transform graph
 * @author Amro Al-Baali
 * @date 2023-05-21
 */

#ifndef TRANSFORMS_GRAPH_TRANSFORMS_GRAPH_H_
#define TRANSFORMS_GRAPH_TRANSFORMS_GRAPH_H_

#include <functional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "transforms_graph/graph_search.h"

namespace tg {

/**
 * @brief Transform graph class
 * @tparam Transform Transform/pose type. Should have `*`, `<<`, and `inverse()` defined
 * @tparam Frame Frame type (e.g. char, int, etc.). Should have `<<` defined.
 * @tparam Inv Function object, where the instance takes an argument of type Transform and returns
 * an inverse Transform.
 */
template <typename Transform, typename Frame = char,
          typename Inv = std::function<Transform(Transform)>>
class TransformsGraph {
 public:
  /** Id/key used to store transforms in the Transforms map */
  using TransformId = int;

  /** Raw transforms */
  using Transforms = std::unordered_map<TransformId, Transform>;

  /** Inverse function object */
  using TransformInverse = Inv;

  /** Adjacency matrix of adjacent frames */
  using AdjacentFrames = std::unordered_map<Frame, std::unordered_set<Frame>>;

  /** Callable that searches for a path between two frames in the graph */
  using GraphSearchCallback =
      std::function<std::vector<Frame>(const AdjacentFrames&, Frame, Frame)>;

  /**
   * @brief Construct a new Transform Graph object
   *
   * @details The maximum number of frames cannot be changed after construction
   *
   * @param[in] max_frames Maximum number of frames allowed in the graph
   * @param[in] transform_inverse Function object to invert a transform. Should take a
   *            transform as an argument and return its inverse.
   */
  TransformsGraph(int max_frames = 100,
                  TransformInverse transform_inverse = std::bind(&Transform::inverse,
                                                                 std::placeholders::_1))
      : max_frames_(max_frames), transform_inverse_(transform_inverse) {}

  /**
   * @brief Get maximum number allowed in the transform graph
   *
   * @return Maximum number of frames allowed in the graph
   */
  int GetMaxFrames() const { return max_frames_; }

  /**
   * @brief Check if a frame is in the graph
   *
   * @param[in] frame Frame to check
   *
   * @return True if it's in the graph
   */
  bool HasFrame(Frame frame) const { return adjacent_frames_.count(frame) > 0; }

  /**
   * @brief Get the path between two frames. Returns empty vector if no path exists
   *
   * @param[in] parent Frame to start from
   * @param[in] child Frame to end at
   *
   * @return Vector of paths from the parent to the child
   */
  std::vector<Frame> GetTransformChain(Frame parent, Frame child) const {
    if (!HasFrame(parent)) return {};
    if (!HasFrame(child)) return {};

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
  bool HasTransform(Frame parent, Frame child) const {
    if (!HasFrame(parent)) return false;
    if (!HasFrame(child)) return false;
    return graph_search_callback_(adjacent_frames_, parent, child).size() > 0;
  }

  /**
   * @brief Checks if a raw transform exists in the graph, irrespective of order.
   *
   * @details A raw transform is a transform that is directly stored in the graph. For example,
   * given a graph `a->b->c`, then `a->b` is a raw transform, whereas `a->c` is a chained transform,
   * but not a raw transform. Note that `b->a` is also a raw transform that is valid in this case.
   *
   * @param[in] parent
   * @param[in] child
   *
   * @return True if the raw transform exists
   */
  bool HasRawTransform(Frame parent, Frame child) const {
    const auto transform_id = ComputeTransformId(parent, child);
    return raw_transforms_.count(transform_id) > 0;
  }

  /**
   * @brief Get the raw transform between two frames (irrespective of order). Throws an exception if
   * no transform exists.
   *
   * @details A raw transform is a transform that is directly stored in the graph. For example,
   * given a graph `a->b->c`, then `a->b` is a raw transform, whereas `a->c` is a chained transform,
   * but not a raw transform. Note that `b->a` is also a raw transform that is valid in this case.
   *
   * @param[in] parent
   * @param[in] child
   *
   * @return
   */
  Transform GetRawTransform(Frame parent, Frame child) const {
    if (!HasRawTransform(parent, child)) {
      throw std::runtime_error("No raw transform exists between the two frames");
    }
    const auto transform_id = ComputeTransformId(parent, child);
    auto transform = raw_transforms_.at(transform_id);

    return ShouldInvertFrames(parent, child) ? transform_inverse_(transform) : transform;
  }

  /**
   * @brief Get the chained transform between two frames. Throws an exception if no transform exists
   *
   * @details A chained transform is a transform between two frames that are not necessarily
   * neighbours in the graph. For example, given a graph `a->b->c`, then `a->c` is a chained
   * transform but not a raw transform.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return Chained transform between the two frames
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
      auto T_prev_curr = raw_transforms_.at(transform_id);
      if (ShouldInvertFrames(prev, frame)) {
        T_prev_curr = transform_inverse_(T_prev_curr);
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
   * @return Chain of transforms between the two frames (e.g., 'b->c' is given by "T_a_b^-1 ->
   * T_a_c"). If no transform exists, an empty string is returned.
   */
  std::string GetTransformChainString(Frame parent, Frame child,
                                      bool show_transforms = false) const {
    const auto paths = GetTransformChain(parent, child);
    if (paths.size() == 0) {
      return "";
    }

    std::stringstream ss_frames;
    std::stringstream ss_transforms;
    Frame prev = parent;
    for (const auto& frame : paths) {
      if (frame == prev) continue;
      if (ShouldInvertFrames(prev, frame)) {
        ss_frames << "T_" << frame << "_" << prev << "^-1";
      } else {
        ss_frames << "T_" << prev << "_" << frame;
      }

      if (show_transforms) {
        // The inversion, if necessary, is taken care of
        ss_transforms << GetRawTransform(prev, frame) << " -> ";
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
    for (const auto& kv : adjacent_frames_) {
      const auto frame = kv.first;
      const auto neighbours = kv.second;
      ss << "  " << get_frame_name(frame) << std::endl;
      for (const auto& neighbour : neighbours) {
        // To keep the direction of the transforms (as they are stored in the graph), ignore the
        // inverse transforms (e.g., if `T_a_b` is stored in the graph, then transforms_['ab']
        // exists, whereas transforms_['ba'] doesn't exist in the graph)
        const auto transform_id = ComputeParentToChildId(frame, neighbour);
        if (!raw_transforms_.count(transform_id)) continue;

        // Add frame name and direction
        ss << "  " << get_frame_name(frame) << " --> ";

        // Add edges to the mermaid graph, if necessary
        if (show_edges) {
          ss << "| " << raw_transforms_.at(transform_id) << " | ";
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
    // The default frame name is whatever is returned by the Frame class
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
  void AddTransform(Frame parent, Frame child, Transform&& pose, bool should_override = false) {
    // Handle the case where the transform exists. The only situation in which the transform is
    // overridden is if the override flag is set tot true AND the transform to be updated is a raw
    // transform
    if (HasTransform(parent, child) && (!should_override || !HasRawTransform(child, parent))) {
      // Note that handling overriding generic transforms is not a simple case of replacing the
      // transform, since the transform might be used in other transforms. For example, if we have
      // the following transforms: 'a'->'b'->'c'->'d', and we add the transform 'a'->'d', then what
      // does "overriding" the transform really mean?
      // To do this correctly, we need to remove one of the paths between 'a' and 'd' to break
      // the cycle, and then we can add 'a'->'d'. However, this is not trivial to do when the
      // transform to be updated is not the raw transform (since there are many possible transforms
      // that can be updated/deleted)
      throw std::runtime_error(
          "Transform already exists between the two frames and overriding is not applicable");
    }

    // Add frame to the graph if it doesn't already exist
    if (!HasFrame(parent)) AddFrame(parent);
    if (!HasFrame(child)) AddFrame(child);

    adjacent_frames_[parent].insert(child);
    adjacent_frames_[child].insert(parent);

    // Insert/update raw transform into the graph
    AddRawTransform(parent, child, std::move(pose));
  }

  /**
   * @brief Adds frame into the graph, if it doesn't already exist. Throws an exception if the
   * maximum number of frames has been reached
   *
   * @param[in] frame Frame to add to the graph
   */
  void AddFrame(Frame frame) {
    if (HasFrame(frame)) return;
    if (GetAllFrames().size() == max_frames_) {
      std::stringstream ss;
      ss << "Maximum number of frames (" << max_frames_ << ") reached. Not adding frame";
      throw std::runtime_error(ss.str().c_str());
    }
    adjacent_frames_.emplace(frame, std::unordered_set<Frame>());
  }

  /**
   * @brief Get the graph of frames and their neighbours
   *
   * @return AdjacentFrames Graph of frames and their neighbours
   */
  const AdjacentFrames& GetGraph() const noexcept { return adjacent_frames_; }

  /**
   * @brief Set the function that is called when searching the graph for a path between two frames
   *
   * @param[in] callback
   */
  void SetGraphSearchCallback(GraphSearchCallback callback) noexcept {
    graph_search_callback_ = callback;
  }

  /**
   * @brief Update or insert a raw transform into the graph. Throws an exception if the frames are
   * not in the graph.
   *
   * @details Note that the function takes care of the order. That is, `T_a_b` and `T_b_a` are
   * treated as one.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   * @param[in] pose Transform from parent to child. That is, for a displacement `r_child` resolved
   */
  void UpdateRawTransform(Frame parent, Frame child, const Transform& pose) {
    if (!HasRawTransform(parent, child)) {
      throw std::runtime_error("Transform does not exist in the graph");
    }
    const auto transform_id = ComputeTransformId(parent, child);
    raw_transforms_[transform_id] =
        ShouldInvertFrames(parent, child) ? transform_inverse_(pose) : pose;
  }

  /**
   * @brief Remove a transform from the graph. If the transform does not exist, then it throws an
   * exception. Note that the frames are not deleted from the graph, even if they are no longer
   * connected to other frames.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   */
  void RemoveRawTransform(Frame parent, Frame child) {
    if (!HasRawTransform(parent, child)) {
      throw std::runtime_error("Transform does not exist in the graph");
    }

    // Remove raw transform
    const auto transform_id = ComputeTransformId(parent, child);
    raw_transforms_.erase(transform_id);

    // Remove edges from the graph
    adjacent_frames_[parent].erase(child);
    adjacent_frames_[child].erase(parent);
  }

  /**
   * @brief Return a vector of all frames in the graph
   *
   * @return Vector of unordered frames in the graph
   */
  std::vector<Frame> GetAllFrames() const {
    std::vector<Frame> frames;
    frames.reserve(adjacent_frames_.size());
    for (const auto& kv : adjacent_frames_) {
      frames.push_back(kv.first);
    }
    return frames;
  }

  /**
   * @brief Get frames adjacent to a given frame. Throws an exception if the frame does not exist
   *
   * @param[in] frame Frame to get neighbours of
   *
   * @return Vector of frames adjacent to the given frame. If frame doesn't have neighbours, then
   * it's an empty vector.
   */
  std::vector<Frame> GetAdjacentFrames(Frame frame) const {
    if (!HasFrame(frame)) {
      throw std::runtime_error("Frame does not exist in the graph");
    }
    return std::vector<Frame>(adjacent_frames_.at(frame).begin(), adjacent_frames_.at(frame).end());
  }

  /**
   * @brief Return a map of all the raw transforms in the graph
   *
   * @return Map of transforms, where the keys are computed using `ComputeTransformId`
   */
  Transforms GetAllRawTransforms() const { return raw_transforms_; }

  /**
   * @brief Remove a frame from the graph. If the frame does not exist, then it throws an
   * exception
   *
   * @param[in] frame to delete
   */
  void RemoveFrame(Frame frame) {
    if (!HasFrame(frame)) {
      throw std::runtime_error("Frame does not exist in the graph");
    }

    // Remove all transforms from/to this frame
    const auto adjacent_frames = GetAdjacentFrames(frame);
    for (const auto& neighbour : adjacent_frames) {
      RemoveRawTransform(frame, neighbour);
    }

    // Remove the frame from the graph
    adjacent_frames_.erase(frame);
  }

 private:
  /**
   * @brief Add a raw transform into the graph. Throws an exception if the transform already exists
   * in the graph.
   *
   * @details This function is used internally and is not expected to be exposed to the user. The
   * user should instead use the `AddTransform` function.
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   * @param[in] pose T_parent_child
   */
  void AddRawTransform(Frame parent, Frame child, const Transform& pose) {
    if (HasRawTransform(parent, child)) {
      throw std::runtime_error("Raw transform already exists");
    }
    // Add frames if they're not already in the graph
    if (!HasFrame(parent)) AddFrame(parent);
    if (!HasFrame(child)) AddFrame(child);

    const auto transform_id = ComputeTransformId(parent, child);
    raw_transforms_[transform_id] =
        ShouldInvertFrames(parent, child) ? transform_inverse_(pose) : pose;
  }

  /**
   * @brief Compute a unique ID for a transform between two frames. The order matters (i.e., the ID
   * for `T_a_b` will be different than `T_b_a`). Check `ComputeTransformId` for an order-agnostic
   * ID.
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
   * @details The transform ID is computed as "aabb", where "aa" < "bb" (i.e., the frames are
   * sorted)
   *
   * @param[in] parent Parent frame
   * @param[in] child Child frame
   *
   * @return Unique ID for the transform, which is in the form "aabb", where "aa" < "bb"
   */
  TransformId ComputeTransformId(Frame parent, Frame child) const {
    return ShouldInvertFrames(parent, child) ? ComputeParentToChildId(child, parent)
                                             : ComputeParentToChildId(parent, child);
  }

  /** Maximum number of frames expected to be in the graph */
  int max_frames_ = 100;

  /** Function object to invert transform */
  TransformInverse transform_inverse_;

  /** Acyclic graph where the vertices are the frames and the edges are transforms between the two
   * frames */
  AdjacentFrames adjacent_frames_;

  /** Map of raw transforms between frames. The transform is stored as `T_a_b`, where `a < b`. That
   * is, if `T_a_b` exists, then `T_b_a` should not exist in the map.
   */
  Transforms raw_transforms_;

  /** Function to call when searching for a path between two frames in the graph */
  GraphSearchCallback graph_search_callback_ = DFS<Frame>;
};

}  // namespace tg
#endif  // TRANSFORMS_GRAPH_TRANSFORMS_GRAPH_H_
