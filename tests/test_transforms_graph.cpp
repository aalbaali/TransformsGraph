#include <gtest/gtest.h>

#include <iostream>

#include "transforms_graph/transforms_graph.h"

class Displacement {
 public:
  static Displacement identity() { return Displacement(0); }
  Displacement() { d_ = 0; }
  Displacement(double d) : d_(d) {}
  double x() const { return d_; }
  Displacement inverse() const { return Displacement(-d_); }

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

using Frame = char;
using Transform = Displacement;
using TransformsGraph = tg::TransformsGraph<Displacement, Frame>;

// Construct a simple graph
TransformsGraph ConstructTwoSubgraphs() {
  // Graph:
  //        -> b -> d
  //      /
  //    a
  //      \
  //        -> c
  //
  ///   e -> f
  TransformsGraph transforms;
  // First connected subgraph
  transforms.AddTransform('a', 'b', 1);
  transforms.AddTransform('a', 'c', 2);
  transforms.AddTransform('b', 'd', 3);

  // Second connected subgraph
  transforms.AddTransform('e', 'f', 3);
  return transforms;
}

TEST(TransformsGraph, EmptyGraph) {
  TransformsGraph transforms;
  EXPECT_EQ(transforms.GetAllFrames().size(), 0);
  EXPECT_EQ(transforms.GetAllRawTransforms().size(), 0);

  // Default max frames should be 100
  EXPECT_EQ(transforms.GetMaxFrames(), 100);

  // Create a new graph with more max frames, and check that it's correct
  TransformsGraph transforms2(10);
  EXPECT_EQ(transforms2.GetMaxFrames(), 10);
}

TEST(TransformsGraph, IdentityOperations) {
  const auto transforms = ConstructTwoSubgraphs();
  EXPECT_TRUE(transforms.HasTransform('a', 'a'));
  EXPECT_DOUBLE_EQ(transforms.GetTransform('a', 'a').x(), Displacement::identity().x());
}

TEST(TransformsGraph, RawTransforms) {
  const auto transforms = ConstructTwoSubgraphs();

  // a<->b raw transform should exist
  EXPECT_TRUE(transforms.HasRawTransform('a', 'b'));
  EXPECT_TRUE(transforms.HasRawTransform('b', 'a'));

  // b<->d raw transform should exist
  EXPECT_TRUE(transforms.HasRawTransform('b', 'd'));
  EXPECT_TRUE(transforms.HasRawTransform('d', 'b'));

  // a<->d RAW transform should not exist
  EXPECT_FALSE(transforms.HasRawTransform('a', 'd'));
  EXPECT_FALSE(transforms.HasRawTransform('d', 'a'));

  // e<->f raw transform should exist
  EXPECT_TRUE(transforms.HasRawTransform('e', 'f'));
  EXPECT_TRUE(transforms.HasRawTransform('f', 'e'));

  // a<->e raw transform should not exist
  EXPECT_FALSE(transforms.HasRawTransform('a', 'e'));
  EXPECT_FALSE(transforms.HasRawTransform('e', 'a'));
}

TEST(TransformsGraph, ChainedTransforms) {
  const auto transforms = ConstructTwoSubgraphs();
  // a<->b chained transform should exist
  EXPECT_TRUE(transforms.HasTransform('a', 'b'));
  EXPECT_TRUE(transforms.HasTransform('b', 'a'));

  // b<->d chained transform should exist
  EXPECT_TRUE(transforms.HasTransform('b', 'd'));
  EXPECT_TRUE(transforms.HasTransform('d', 'b'));

  // a<->d chained transform should exist
  EXPECT_TRUE(transforms.HasTransform('a', 'd'));
  EXPECT_TRUE(transforms.HasTransform('d', 'a'));

  // e<->f chained transform should exist
  EXPECT_TRUE(transforms.HasTransform('e', 'f'));
  EXPECT_TRUE(transforms.HasTransform('f', 'f'));

  // a<->e chained transform should not exist
  EXPECT_FALSE(transforms.HasTransform('a', 'e'));
  EXPECT_FALSE(transforms.HasTransform('e', 'a'));
}

TEST(TransformsGraph, AddingTransform) {
  // Add a transform that connects the two subgraphs
  auto transforms = ConstructTwoSubgraphs();
  transforms.AddTransform('d', 'e', 4);

  // a<->e chained transform should exist
  EXPECT_TRUE(transforms.HasTransform('a', 'e'));
  EXPECT_TRUE(transforms.HasTransform('e', 'a'));
}

TEST(TransformsGraph, ExceedingMaxFrames) {
  TransformsGraph transforms(3);
  transforms.AddFrame('a');
  transforms.AddFrame('b');
  transforms.AddFrame('c');

  // No transforms are added
  EXPECT_EQ(transforms.GetAllRawTransforms().size(), 0);

  EXPECT_EQ(transforms.GetAllFrames().size(), 3);
  EXPECT_ANY_THROW(transforms.AddFrame('d'));
  EXPECT_ANY_THROW(transforms.AddTransform('a', 'd', 5));

  // Adding a transform to an already-existing transforms shouldn't throw an exception
  EXPECT_NO_THROW(transforms.AddTransform('a', 'b', 1));
  EXPECT_EQ(transforms.GetAllRawTransforms().size(), 1);
}

// Tests to add:
// - TransformsGraph(int max_frames = 100)
// - MaxFrames() const
// - HasFrame(Frame frame) const
// - GetTransformChain(Frame parent, Frame child) const
// - HasTransform(Frame parent, Frame child) const
// - HasRawTransform(Frame parent, Frame child) const
// - GetRawTransform(Frame parent, Frame child) const
// - GetTransform(Frame parent, Frame child) const
// - GetTransformChainString(Frame parent, Frame child, bool show_transforms = false) const
// - GetMermaidGraph(const std::function<std::string(Frame)>& get_frame_name, bool show_edges =
// false) const
// - GetMermaidGraph(bool show_edges = false) const
// - AddTransform(Frame parent, Frame child, const Transform& pose, bool should_override = false)
// - UpdateRawTransform(Frame parent, Frame child, const Transform& pose)
// - RemoveRawTransform(Frame parent, Frame child)
// - GetAllFrames() const
// - GetAdjacentFrames(Frame frame) const
// - GetAllRawTransforms() const
// - RemoveFrame(Frame frame)

// Chained transform
// Inverse chained transform
// Frames

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
