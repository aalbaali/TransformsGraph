/**
 * @file test_transforms_graph.cpp
 * @brief Unit tests for the TransformsGraph class using 'Frame = char' and 'Transform =
 * Displacement'
 * @author Amro Al-Baali
 * @date 2023-05-22
 */
#include <gtest/gtest.h>

#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "transforms_graph/graph_search.h"
#include "transforms_graph/transforms_graph.h"

using Node = int;
using Nodes = std::vector<Node>;

class UnorderedGraph : public ::testing::Test {
 public:
  using Graph = std::unordered_map<Node, std::unordered_set<Node>>;

 protected:
  void SetUp() override {
    // clang-format off
    // Graph:
    //        -> b -> d
    //      /
    //    a
    //      \
    //        -> c
    //
    ///   e -> f
    graph_ = {{'a', {'b', 'c'}},
              {'b', {'a', 'd'}},
              {'c', {'a'}},
              {'d', {'b'}},
              {'e', {'f'}},
              {'f', {'e'}}};
    // clang-format on
  }

  std::function<Nodes(const Graph&, Node, Node)> graph_search_alg_;
  Graph graph_;
};

class OrderedGraph : public ::testing::Test {
 public:
  using Graph = std::map<Node, std::set<Node>>;

 protected:
  void SetUp() override {
    // clang-format off
    // Graph:
    //        -> b -> d
    //      /
    //    a
    //      \
    //        -> c
    //
    ///   e -> f
    graph_ = {{'a', {'b', 'c'}},
              {'b', {'a', 'd'}},
              {'c', {'a'}},
              {'d', {'b'}},
              {'e', {'f'}},
              {'f', {'e'}}};
    // clang-format on
  }

  std::function<Nodes(const Graph&, Node, Node)> graph_search_alg_;
  Graph graph_;
};

inline void TestSearchAlgorithm(std::function<Nodes(Node, Node)> search_graph) {
  // a<->a
  const auto path_a_a = search_graph('a', 'a');
  EXPECT_EQ(path_a_a.size(), 1);
  EXPECT_EQ(path_a_a.front(), 'a');

  // a<->b
  const auto path_a_b = search_graph('a', 'b');
  const auto path_b_a = search_graph('b', 'a');
  EXPECT_EQ(path_a_b.size(), 2);
  EXPECT_EQ(path_b_a.size(), 2);
  EXPECT_EQ(path_a_b.front(), 'a');
  EXPECT_EQ(path_a_b.back(), 'b');
  EXPECT_EQ(path_b_a.front(), 'b');
  EXPECT_EQ(path_b_a.back(), 'a');

  // a<->d
  const auto path_a_d = search_graph('a', 'd');
  const auto path_d_a = search_graph('d', 'a');
  EXPECT_EQ(path_a_d.size(), 3);
  EXPECT_EQ(path_d_a.size(), 3);
  EXPECT_EQ(path_a_d[0], 'a');
  EXPECT_EQ(path_a_d[1], 'b');
  EXPECT_EQ(path_a_d[2], 'd');
  EXPECT_EQ(path_d_a[0], 'd');
  EXPECT_EQ(path_d_a[1], 'b');
  EXPECT_EQ(path_d_a[2], 'a');

  // b<->c
  const auto path_b_c = search_graph('b', 'c');
  const auto path_c_b = search_graph('c', 'b');
  EXPECT_EQ(path_b_c.size(), 3);
  EXPECT_EQ(path_c_b.size(), 3);
  EXPECT_EQ(path_b_c[0], 'b');
  EXPECT_EQ(path_b_c[1], 'a');
  EXPECT_EQ(path_b_c[2], 'c');
  EXPECT_EQ(path_c_b[0], 'c');
  EXPECT_EQ(path_c_b[1], 'a');
  EXPECT_EQ(path_c_b[2], 'b');

  // e<->f
  const auto path_e_f = search_graph('e', 'f');
  const auto path_f_e = search_graph('f', 'e');
  EXPECT_EQ(path_e_f.size(), 2);
  EXPECT_EQ(path_f_e.size(), 2);
  EXPECT_EQ(path_e_f.front(), 'e');
  EXPECT_EQ(path_e_f.back(), 'f');
  EXPECT_EQ(path_f_e.front(), 'f');
  EXPECT_EQ(path_f_e.back(), 'e');

  // a<->e
  const auto path_a_e = search_graph('a', 'e');
  const auto path_e_a = search_graph('e', 'a');
  EXPECT_EQ(path_a_e.size(), 0);
  EXPECT_EQ(path_e_a.size(), 0);
}

using GraphSearch = std::function<Nodes(Node, Node)>;

/**
 * @brief DFS on unordered map
 *
 * @param[in] UnorderedGraph
 * @param[in] DFS
 */
TEST_F(UnorderedGraph, DFS) {
  GraphSearch graph_search_alg_ =
      std::bind(tg::DFS<Node, Graph>, graph_, std::placeholders::_1, std::placeholders::_2);

  TestSearchAlgorithm(graph_search_alg_);
}

/**
 * @brief BFS on unordered map
 *
 * @param[in] UnorderedGraph
 * @param[in] BFS
 */
TEST_F(UnorderedGraph, BFS) {
  GraphSearch graph_search_alg_ =
      std::bind(tg::BFS<Node, Graph>, graph_, std::placeholders::_1, std::placeholders::_2);

  TestSearchAlgorithm(graph_search_alg_);
}

/**
 * @brief DFS on ordered map
 *
 * @param[in] UnorderedGraph
 * @param[in] DFS
 */
TEST_F(OrderedGraph, DFS) {
  GraphSearch graph_search_alg_ =
      std::bind(tg::DFS<Node, Graph>, graph_, std::placeholders::_1, std::placeholders::_2);

  TestSearchAlgorithm(graph_search_alg_);
}

/**
 * @brief BFS on ordered map
 *
 * @param[in] UnorderedGraph
 * @param[in] BFS
 */
TEST_F(OrderedGraph, BFS) {
  GraphSearch graph_search_alg_ =
      std::bind(tg::BFS<Node, Graph>, graph_, std::placeholders::_1, std::placeholders::_2);

  TestSearchAlgorithm(graph_search_alg_);
}
int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
