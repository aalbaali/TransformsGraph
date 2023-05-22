/**
 * @file graph_search.h
 * @brief Graph search algorithms
 * @author Amro Al-Baali
 * @date 2023-05-21
 */

#ifndef TRANSFORMS_GRAPH_GRAPH_SEARCH_H_
#define TRANSFORMS_GRAPH_GRAPH_SEARCH_H_

#include <algorithm>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace tg {
/**
 * @brief Depth-first-search to find a path between two frames
 *
 * @param[in] graph Transform graph
 * @param[in] start Starting frame
 * @param[in] end Ending frame
 *
 * @return
 */
template <typename Node, typename Graph = std::unordered_map<Node, std::unordered_set<Node>>>
std::vector<Node> DFS(const Graph& graph, Node start, Node end) {
  std::vector<Node> path;
  std::unordered_set<Node> visited;
  std::unordered_map<Node, Node> parent;
  std::vector<Node> stack;

  bool found_solution = false;
  stack.push_back(start);
  while (!stack.empty()) {
    Node current = stack.back();
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
  Node node = end;
  while (node != start) {
    path.push_back(node);
    node = parent[node];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

/**
 * @brief Breadth-first-search to find a path between two frames
 *
 * @param[in] graph Transform graph
 * @param[in] start Starting frame
 * @param[in] end Ending frame
 *
 * @return
 */
template <typename Node, typename Graph = std::unordered_map<Node, std::unordered_set<Node>>>
std::vector<Node> BFS(const Graph& graph, Node start, Node end) {
  std::vector<Node> path;
  std::unordered_set<Node> visited;
  std::unordered_map<Node, Node> parent;
  std::queue<Node> queue;

  bool found_solution = false;
  queue.push(start);
  visited.insert(start);

  while (!queue.empty()) {
    Node current = queue.front();
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
  Node node = end;
  while (node != start) {
    path.push_back(node);
    node = parent[node];
  }
  path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

}  // namespace tg

#endif // TRANSFORMS_GRAPH_GRAPH_SEARCH_H_
