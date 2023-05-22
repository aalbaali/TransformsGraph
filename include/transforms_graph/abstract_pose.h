/**
 * @file abstract_pose.h
 * @brief This file is to define the abstract pose class that is to be extended by the user
 * @author Amro Al-Baali
 * @date 2023-05-21
 */

#ifndef transforms_graph_ABSTRACT_POSE_H_
#define transforms_graph_ABSTRACT_POSE_H_

namespace tg {

template<typename T>
class AbstractPose {
 public:
  virtual T inverse() const = 0;
};

}  // namespace tg

#endif  // transforms_graph_ABSTRACT_POSE_H_
