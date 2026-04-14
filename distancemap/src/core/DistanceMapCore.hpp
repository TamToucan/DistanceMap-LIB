#pragma once

#include <vector>

#include "DistanceMapDLL.hpp"
#include "GridToGraph.hpp"
#include "WallDistanceGrid.hpp"

namespace DistanceMap {

class DISTANCEMAP_API DistanceMapCore {
public:
  DistanceMapCore() = default;
  ~DistanceMapCore() = default;

  // Grid is 0 = empty, 1 = solid
  void initialize(const std::vector<std::vector<int>> &grid);

  const GridToGraph::Graph &getGraph() const { return m_graph; }

private:
  GridToGraph::Graph m_graph;
  GridType::Grid wallDistGrid;
  DistanceMap::SightGrid sightGrid;
};
} // namespace DistanceMap
