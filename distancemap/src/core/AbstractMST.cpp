/*
 * AbstractMST.cpp
 *
 *  Created on: 22 Mar 2025
 *      Author: tam
 */

#include "AbstractMST.hpp"
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>
#include <vector>


#include "GridTypes.hpp"

#include "Debug.h"

namespace DistanceMap {

using namespace GridType;

AbstractMST::AbstractMST() {}

AbstractMST::~AbstractMST() {}

namespace {
// Union-Find (Disjoint Set) structure for Kruskal's algorithm.
struct UnionFind {
  std::vector<int> parent;
  UnionFind(int n) : parent(n) {
    for (int i = 0; i < n; i++)
      parent[i] = i;
  }
  int find(int x) {
    if (parent[x] != x)
      parent[x] = find(parent[x]);
    return parent[x];
  }
  void unite(int x, int y) { parent[find(x)] = find(y); }
};
} // namespace

// Dijkstra's algorithm: given a starting base node, compute the shortest path
// to all base nodes. Returns the distance vector, predecessor vector, and
// usedEdge vector (which stores the edge index and direction used).
void runDijkstra(int startIdx, const BaseGraph &graph, std::vector<int> &costs,
                 std::vector<int> &prevNodes,
                 std::vector<std::pair<int, bool>> &usedEdge) {
  int numNodes = static_cast<int>(graph.size());
  costs.assign(numNodes, std::numeric_limits<int>::max());
  prevNodes.assign(numNodes, -1);
  usedEdge.assign(numNodes, std::make_pair(-1, true));

  using CostNode = std::pair<int, int>; // (cost, base node)
  std::priority_queue<CostNode, std::vector<CostNode>, std::greater<CostNode>>
      pq;

  costs[startIdx] = 0;
  pq.push({0, startIdx});

  while (!pq.empty()) {
    auto [cost, nodeIdx] = pq.top();
    pq.pop();
    // if cost is already outdated, skip
    if (cost > costs[nodeIdx])
      continue;

    // check all the edges from this node
    for (const BaseGraphInfo &adj : graph[nodeIdx]) {
      int neighborIdx = adj.neighbor;
      int newCost = cost + adj.cost;
      if (newCost < costs[neighborIdx]) {
        costs[neighborIdx] = newCost;
        prevNodes[neighborIdx] = nodeIdx;
        usedEdge[neighborIdx] = {adj.edgeIndex, adj.forward};
        pq.push({newCost, neighborIdx});
      }
    }
  }
}

// Reconstruct the path (as a sequence of Points) between two base nodes given
// the Dijkstra predecessors.
Path reconstructPath(int startIdx, int targetIdx,
                     const std::vector<int> &prevNodes,
                     const std::vector<std::pair<int, bool>> &usedEdge,
                     const std::vector<Edge> &edges) {
  std::vector<std::pair<int, bool>> edgeSequence;
  int curIdx = targetIdx;
  while (curIdx != startIdx) {
    edgeSequence.push_back(usedEdge[curIdx]);
    curIdx = prevNodes[curIdx];
  }
  std::reverse(edgeSequence.begin(), edgeSequence.end());

  Path fullPath;
  bool firstSegment = true;
  for (auto [edgeIdx, forward] : edgeSequence) {
    const Path &edgePath = edges[edgeIdx].path;
    Path segment = edgePath; // Copy the segment.
    if (!forward)
      std::reverse(segment.begin(), segment.end());
    // Remove duplicate junction point if not the first segment.
    if (!firstSegment && !segment.empty())
      segment.erase(segment.begin());
    fullPath.insert(fullPath.end(), segment.begin(), segment.end());
    firstSegment = false;
  }
  return fullPath;
}
std::vector<int> reconstructBaseNodePath(int startIdx, int targetIdx,
                                         const std::vector<int> &prevNodes) {
  std::vector<int> path;
  int curIdx = targetIdx;
  while (curIdx != -1 && curIdx != startIdx) {
    path.push_back(curIdx);
    curIdx = prevNodes[curIdx];
  }

  if (curIdx == -1)
    return {}; // no path

  path.push_back(startIdx);
  std::reverse(path.begin(), path.end());
  return path;
}

// Compute candidate AbstractEdges between all pairs of AbstractNodes.
// For each abstract node pair (i, j) with i < j, run Dijkstra's from
// abstractNodes[i].baseCenterNode to determine the shortest path to
// abstractNodes[j].baseCenterNode.
void computeCandidateEdges(const BaseGraph &inBaseGraph,
                           const std::vector<Edge> &edges,
                           const std::vector<Point> &baseNodes,
                           const std::vector<AbstractNode> &abstractNodes,
                           std::vector<AbstractEdge> &candidates) {
  int numBaseNodes = static_cast<int>(baseNodes.size());
  BaseGraph gened;
  if (inBaseGraph.empty()) {
    gened = buildBaseGraph(edges, numBaseNodes);
  }
  const BaseGraph &baseGraph = (inBaseGraph.empty()) ? gened : inBaseGraph;

  int numAbstractNodes = static_cast<int>(abstractNodes.size());
  candidates.clear();

  // For each abstract node as a source:
  LOG_DEBUG("##MST: computeCandidateEdges abNodes: " << numAbstractNodes);
  for (int i = 0; i < numAbstractNodes; ++i) {
    int startBase = abstractNodes[i].baseCenterNode;
    std::vector<int> costs;
    std::vector<int> prevNodes;
    std::vector<std::pair<int, bool>> usedEdge;
    runDijkstra(startBase, baseGraph, costs, prevNodes, usedEdge);

    // For every other abstract node j > i, record the candidate edge.
    for (int j = i + 1; j < numAbstractNodes; ++j) {
      int targetBase = abstractNodes[j].baseCenterNode;
      if (costs[targetBase] == std::numeric_limits<int>::max())
        continue; // Not reachable.
      AbstractEdge ce;
      ce.from = i;
      ce.to = j;
      ce.path =
          reconstructPath(startBase, targetBase, prevNodes, usedEdge, edges);
      ce.nodePath = reconstructBaseNodePath(startBase, targetBase, prevNodes);
      LOG_DEBUG("NODEPATH:  " << i);
      LOG_DEBUG_FOR(auto n : ce.nodePath, "  " << n << " ("
                                               << baseNodes[n].first << ","
                                               << baseNodes[n].second << ")");
      LOG_DEBUG_FOR(const auto &n : ce.path,
                    " (" << n.first << "," << n.second << ")");

      candidates.push_back(ce);
    }
  }
  LOG_DEBUG("##=> " << candidates.size() << " candidates");
}

// Run Kruskal's algorithm on the candidate edges to build the MST over abstract
// nodes.
std::vector<AbstractEdge> buildMST(const std::vector<AbstractEdge> &candidates,
                                   int numAbstractNodes) {
  std::vector<AbstractEdge> mstEdges;
  // Start with all candidate edges and sort them by weight.
  std::vector<AbstractEdge> sortedEdges = candidates;
  std::sort(sortedEdges.begin(), sortedEdges.end(),
            [](const AbstractEdge &a, const AbstractEdge &b) {
              return a.path.size() < b.path.size();
            });

  // Use a union-find structure to detect cycles.
  UnionFind uf(numAbstractNodes);

  LOG_INFO("MST: Build MST from " << sortedEdges.size() << " candidates");
  for (const AbstractEdge &ce : sortedEdges) {
    int setA = uf.find(ce.from);
    int setB = uf.find(ce.to);
    if (setA != setB) {
      uf.unite(setA, setB);
      mstEdges.push_back(ce);
      // Stop if we have n-1 edges.
      if (mstEdges.size() == static_cast<size_t>(numAbstractNodes - 1))
        break;
    }
  }
  LOG_INFO("MST: DONE ADDed MST EDGEs: " << mstEdges.size());
  return mstEdges;
}

//
// Idex since can have multiple Edges with same From/To (but diff path), to keep
// the Edge which has the shorter path. But for 3200 Edges only 90 "duplicates"
// which does not seem a large enough saving
//
#if 0
std::vector<Edge> simplifyEdges(const std::vector<Edge> edges)
{
    std::vector<Edge> result;
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> shortestEdgeMap;

    // Build a map of (from,to) => (index,cost)
    // For multiple (from,to) only the smallest cost is stored
    for (int idx = 0; idx < static_cast<int>(edges.size()); ++idx) {
        const auto& edge = edges[idx];
        auto fromto = std::make_pair(edge.from, edge.to);
        int cost = static_cast<int>(edge.path.size());

        auto it = shortestEdgeMap.find(fromto);
        if (it == shortestEdgeMap.end() || cost < it->second.second) {
            shortestEdgeMap[fromto] = { idx, cost };
        }
    }
    // Get each Index from the map and store the Edge for it
    for (const auto& entry : shortestEdgeMap) {
        result.push_back(edges[entry.second.first]);
    }

    return result;
}
#endif

// The main function that computes the optimized set of AbstractEdges (MST)
// connecting all AbstractNodes.
std::vector<AbstractEdge> AbstractMST::generateMSTAbstractEdges(
    const BaseGraph &graph, const std::vector<Edge> &edges,
    const std::vector<Point> &baseNodes,
    const std::vector<AbstractNode> &abstractNodes) {
  std::vector<AbstractEdge> candidates;
  LOG_INFO("#MST: generateMSTAbstractEdges: for " << abstractNodes.size()
                                                  << " abstract nodes");
  // Idea was to remove dup F -> T keep the shortest path, but multiple paths is
  // allows for varied movement std::vector<Edge> simpleEdges =
  // simplifyEdges(edges);
  computeCandidateEdges(graph, edges, baseNodes, abstractNodes, candidates);
  return buildMST(candidates, static_cast<int>(abstractNodes.size()));
}
} /* namespace DistanceMap */
