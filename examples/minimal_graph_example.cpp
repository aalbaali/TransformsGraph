/**
 * @file minimal_graph_example.cpp
 * @brief A minimal example of using the transform graph using displacements as transforms
 * @author Amro Al-Baali
 * @date 2023-05-21
 */
#include <iostream>

#include "transforms_graph/transforms_graph.h"

/**
 * @brief A simple class to represent a displacement
 *
 * @details This class serves as a very basic, 1-D, alternative to a proper Pose class. The
 * important things to define for this class are the `inverse()`, `identity()`, `<<`, and `*`
 * functions/operators
 */
class Displacement {
 public:
  Displacement() { d_ = 0; }
  Displacement(double d) : d_(d) {}
  double x() const { return d_; }

 private:
  double d_;
};

// Overloading the `<<` operator is important when visualizing the graph
std::ostream& operator<<(std::ostream& os, const Displacement& d) {
  os << d.x();
  return os;
}

// Overloading the `*` operator is important as that's what "chains" the frames together
Displacement operator*(const Displacement& lhs, const Displacement& rhs) {
  return Displacement(lhs.x() + rhs.x());
}

int main(int argc, char* argv[]) {
  // Let frames be defined using 'char' type
  using Frame = char;
  using Transform = Displacement;

  // Construct a graph that consists of two unconnected subgraphs
  auto displacement_inverse = [](const Transform& t) -> Transform { return -t.x(); };
  tg::TransformsGraph<Transform, Frame> transforms(100, displacement_inverse);
  transforms.InsertTransform('a', 'b', 1);
  transforms.InsertTransform('a', 'c', 2);
  transforms.InsertTransform('b', 'd', 3);

  // Add 'e' and 'f' such that they are in a subgraph that is not connected to the rest of the graph
  transforms.InsertTransform('e', 'f', 4);

  // Visualize the graph using mermaid graph. Copy the output and run on
  // https://mermaid-js.github.io/mermaid-live-editor
  // You should see the two unconnected subgraphs
  std::cout << transforms.GetMermaidGraph() << std::endl;

  // There are some ways to check if a transform exists between two frames
  std::cout << "Does a->b exist? " << transforms.HasTransform('a', 'b') << std::endl;
  std::cout << "Does a->d exist? " << transforms.HasTransform('a', 'd') << std::endl;
  std::cout << "Does a->f exist? " << transforms.HasTransform('a', 'f') << std::endl;

  // Now let's connect the two subgraphs
  transforms.InsertTransform('c', 'f', 5);

  // Visualize the graph with edges
  std::cout << transforms.GetMermaidGraph(true) << std::endl;
  std::cout << "Does a->f exist? " << transforms.HasTransform('a', 'f') << std::endl;

  // One can visualize the path between two frames. This should match what you see on the graph
  std::cout << "Path from a->e: " << transforms.GetTransformChainString('a', 'e', true)
            << std::endl;

  // Get the transform from a->e. This should match the summation (because we defined the `*`
  // operator as a summation) from the path above
  std::cout << "Transform from a->e: " << transforms.GetTransform('a', 'e') << std::endl;

  // Adding a transform to an already existing path (i.e., transform) should throw an error
  // This is because the graph is acyclic and adding a transform to an existing path would
  // create a cycle
  try {
    transforms.InsertTransform('a', 'f', 7);
  } catch (const std::runtime_error& e) {
    std::cout << "Caught exception: " << e.what() << std::endl;
  }

  // Removing a frame from the graph should delete all the transforms that are connected to it
  transforms.RemoveFrame('a');
  std::cout << "Graph after removing frame 'a'\n" << transforms.GetMermaidGraph(true) << std::endl;

  // Removing a raw transform from the graph removes the transform but not the frames
  transforms.RemoveRawTransform('f', 'c');
  std::cout << "Graph after removing 'f->c' transform\n"
            << transforms.GetMermaidGraph(true) << std::endl;
}
