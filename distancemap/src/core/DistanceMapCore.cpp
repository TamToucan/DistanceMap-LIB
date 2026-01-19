#include "DistanceMapCore.hpp"

#include <memory>

#include "Debug.h"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "NavigationAPI.hpp"
#include "NavigationFlowGrid.hpp"
#include "NavigationGraph.hpp"

namespace DistanceMap {

void DistanceMapCore::initialize(const std::vector<std::vector<int>>& grid, const Router::Info& info) {
  this->m_info = info;

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
  m_graph = GridToGraph::makeGraph(floorGrid);
}

std::unique_ptr<NavigationAPI> DistanceMapCore::makeNavigator(NavigatorType type) {
  switch (type) {
    case FLOW:
      return std::make_unique<Routing::NavigationFlowGrid>(m_graph.infoGrid, m_info);
    case GRAPH:
      return std::make_unique<Routing::NavigationGraph>(m_graph, m_info);
  }
  return nullptr;
}

float DistanceMapCore::getMoveAngle(const std::unique_ptr<NavigationAPI>& pNavigator,
                                    Router::RouteCtx* ctx, GridType::Vec2 from,
                                    GridType::Vec2 to, int type) {
  return pNavigator->getMoveDirection(ctx, from, to, type);
}

GridType::Vec2 DistanceMapCore::getMovePos(const std::unique_ptr<NavigationAPI>& pNavigator,
                                           GridType::Vec2 from, float ang, float distance) {
  // Resolve move with sliding
  return pNavigator->resolveMove(from, ang, distance);
}

}  // namespace DistanceMap
