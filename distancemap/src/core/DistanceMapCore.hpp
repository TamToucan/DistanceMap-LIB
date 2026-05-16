#pragma once

/**
 * @file DistanceMapCore.hpp
 * @brief Public entry point for the DistanceMap library.
 * @details Wraps a single GridToGraph::Graph built from a binary occupancy
 * grid. Construct, call initialize(grid), then read the resulting Graph via
 * getGraph(). All higher-level navigation (rooms, zones, flow fields, sparse
 * nav graph, routing) lives inside that Graph.
 */

#include <vector>

#include "DistanceMapDLL.hpp"
#include "GridToGraph.hpp"

namespace DistanceMap {

/**
 * @class DistanceMapCore
 * @brief Owns the full navigation Graph for one cave/level.
 */
class DISTANCEMAP_API DistanceMapCore {
public:
  DistanceMapCore() = default;
  ~DistanceMapCore() = default;

  /**
   * @brief Build the navigation graph from an occupancy grid.
   * @param grid Row-major [y][x] grid: 0 = empty/floor, 1 = solid/wall.
   *             Note: GridToGraph internally maps this so floor=PATH for the
   *             thinning step.
   */
  void initialize(const std::vector<std::vector<int>> &grid);

  /// Read-only access to the assembled navigation graph.
  const GridToGraph::Graph &getGraph() const { return m_graph; }

private:
  GridToGraph::Graph m_graph;
};
} // namespace DistanceMap
