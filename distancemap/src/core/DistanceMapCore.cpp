#include "DistanceMapCore.hpp"

#include "Debug.h"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"

namespace DistanceMap {

void DistanceMapCore::initialize(const std::vector<std::vector<int>>& grid,
                                 const Router::Info& info) {
  SET_DEBUG("ALL");
  this->info = info;

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

  // Create a floor grid where
  //   WALLS = EMPTY
  //   FLOOR = PATH
  GridType::Grid floorGrid = GridToGraph::gridToFloorGrid(grid);

  LOG_INFO("## makeGraph");
  // Reset navigators before updating graph to avoid dangling references during update
  navGraph.reset();
  navFlow.reset();

  m_graph = GridToGraph::makeGraph(floorGrid);
  navGraph = std::make_unique<Routing::NavigationGraph>(m_graph, info);
  navFlow = std::make_unique<Routing::NavigationFlowGrid>(m_graph.infoGrid, info);
  pNavigator = navGraph.get();
}

float DistanceMapCore::getMove(Router::RouteCtx* ctx, GridType::Vec2 from, GridType::Vec2 to, int type) {
  return pNavigator->getMoveDirection(ctx, from, to, type);
}

GridType::Vec2 DistanceMapCore::getMove(GridType::Vec2 from, float ang, float distance) {
  // Resolve move with sliding
  return pNavigator->resolveMove(from, ang, distance);
}

}  // namespace DistanceMap
