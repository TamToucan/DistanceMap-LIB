#include "DistanceMapCore.hpp"

#include "Debug.h"
#include "GridToGraph.hpp"

namespace DistanceMap {

void DistanceMapCore::initialize(const std::vector<std::vector<int>>& grid) {
  LOG_DEBUG("## CREATE DistaceMapCore");
  for (const auto& row : grid) {
    for (int xy : row) {
      LOG_DEBUG_CONT((xy ? '#' : ' '));
    }
    LOG_DEBUG("");
  }
  LOG_DEBUG("");

  // Make a grid of distance to closest wall (0 = wall)
  LOG_INFO("## makeWallDistanceGrid");
  wallDistGrid = DistanceMap::makeWallDistanceGrid(grid);

  // Make a SightGrid which can return the distance to wall (N,S,E,W)
  LOG_INFO("## makeSightGrid");
  sightGrid = DistanceMap::makeSightGrid(grid);

  // Create a floor grid where WALLS = EMPTY, FLOOR = PATH
  GridType::Grid floorGrid = GridToGraph::gridToFloorGrid(grid);

  LOG_INFO("## makeGraph");
  m_graph = GridToGraph::makeGraph(floorGrid);
}

}  // namespace DistanceMap
