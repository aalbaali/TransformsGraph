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
  transforms.AddTransform('e', 'f', 4);
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

TEST(TransformsGraph, TransformChain) {
  auto transforms = ConstructTwoSubgraphs();

  // Get all the transform chain
  const auto frames_chain = transforms.GetTransformChain('c', 'd');
  EXPECT_EQ(frames_chain.size(), 4);

  EXPECT_EQ(frames_chain[0], 'c');
  EXPECT_EQ(frames_chain[1], 'a');
  EXPECT_EQ(frames_chain[2], 'b');
  EXPECT_EQ(frames_chain[3], 'd');

  // Get string chain
  const std::string frames_chain_string = transforms.GetTransformChainString('c', 'd');
  const std::string expected_str = "T_a_c^-1 -> T_a_b -> T_b_d -> ";
  EXPECT_STREQ(frames_chain_string.c_str(), expected_str.c_str());

  // If no chain exists
  transforms.AddFrame('e');
  const auto frames_chain_2 = transforms.GetTransformChain('c', 'e');
  EXPECT_EQ(frames_chain_2.size(), 0);

  const std::string frames_chain_string_2 = transforms.GetTransformChainString('c', 'e');
  EXPECT_STREQ(frames_chain_string_2.c_str(), "");
}

TEST(TransformsGraph, UpdatingRawTransform) {
  auto transforms = ConstructTwoSubgraphs();

  EXPECT_DOUBLE_EQ(transforms.GetTransform('a', 'b').x(), 1);
  EXPECT_NO_THROW(transforms.UpdateRawTransform('a', 'b', 7));
  EXPECT_DOUBLE_EQ(transforms.GetTransform('a', 'b').x(), 7);

  // Expect throw when updating a non-existing transform
  EXPECT_ANY_THROW(transforms.UpdateRawTransform('a', 'e', 7));
}

TEST(TransformsGraph, GetAdjacentFrames) {
  const auto transforms = ConstructTwoSubgraphs();

  EXPECT_EQ(transforms.GetAdjacentFrames('a').size(), 2);
  EXPECT_EQ(transforms.GetAdjacentFrames('b').size(), 2);
  EXPECT_EQ(transforms.GetAdjacentFrames('c').size(), 1);
  EXPECT_EQ(transforms.GetAdjacentFrames('d').size(), 1);
}

TEST(TransformsGraph, RemoveRawTransform) {
  auto transforms = ConstructTwoSubgraphs();

  // Exception should be thrown if removing a non-raw transform, even if it's chained
  EXPECT_TRUE(transforms.HasTransform('a', 'd'));
  EXPECT_FALSE(transforms.HasRawTransform('a', 'd'));
  EXPECT_ANY_THROW(transforms.RemoveRawTransform('a', 'd'));
  EXPECT_ANY_THROW(transforms.RemoveRawTransform('y', 'z'));

  // Remove a transform that will split the graph into two subgraphs
  EXPECT_NO_THROW(transforms.RemoveRawTransform('a', 'b'));

  // There should no longer be a chained transform between a and d
  EXPECT_FALSE(transforms.HasTransform('a', 'd'));
  EXPECT_EQ(transforms.GetAdjacentFrames('a').size(), 1);
  EXPECT_EQ(transforms.GetAdjacentFrames('b').size(), 1);
}

TEST(TransformsGraph, RemoveFrame) {
  auto transforms = ConstructTwoSubgraphs();

  // Exception should be thrown if removing a non-existent frame
  EXPECT_ANY_THROW(transforms.RemoveFrame('y'));

  // Removing frame 'a' should divide the first subgraph into two subgraphs
  EXPECT_TRUE(transforms.HasFrame('a'));
  EXPECT_NO_THROW(transforms.RemoveFrame('a'));
  EXPECT_FALSE(transforms.HasFrame('a'));
  EXPECT_FALSE(transforms.HasRawTransform('a', 'b'));
  EXPECT_FALSE(transforms.HasRawTransform('a', 'c'));
  EXPECT_FALSE(transforms.HasTransform('a', 'b'));
  EXPECT_FALSE(transforms.HasTransform('a', 'c'));
  EXPECT_FALSE(transforms.HasTransform('c', 'd'));
  EXPECT_TRUE(transforms.HasTransform('b', 'd'));
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
