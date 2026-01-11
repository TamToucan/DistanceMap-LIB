#ifndef NAVIGATION_GRAPH_HPP
#define NAVIGATION_GRAPH_HPP

#include "DistanceMapApi.h"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "Router.hpp"
#include "SparseNavGraph.hpp"
#include <tuple>
#include <vector>

namespace DistanceMap {
namespace Routing {

//
// DistanceMapNavigator uses a flow field approach:
//   - Computes distance map once when target changes (~1-3ms for 320x256 grid)
//   - Answers queries in O(1) time via lookup
//   - ~150 lines vs 370+ lines in this file
//   - Only requires infoGrid (no nodes/edges/zones)

class DISTANCEMAP_API NavigationGraph {
public:
  NavigationGraph();
  ~NavigationGraph() = default;

  void initialize(const GridToGraph::Graph &graphData,
                  const Router::Info &info);

  // Main entry point for movement
  float getMoveDirection(Router::RouteCtx *ctx, GridType::Vec2 from,
                         GridType::Vec2 to, int type);

private:
  // Data members from GridToGraph::Graph
  GridType::Grid m_infoGrid;
  GridToGraph::BaseGraph m_baseGraph;
  GridToGraph::PathCostMap m_pathCostMap;
  SparseNavGraph m_routingGraph;
  std::vector<GridType::Edge> m_baseEdges;
  std::vector<GridType::Point> m_baseNodes;
  std::vector<GridType::Point> m_deadEnds;
  std::vector<GridToGraph::AbstractLevel> m_abstractLevels;

  Router::Info m_info;

  // Helpers
  GridType::Point getNextMove(Router::RouteCtx *ctx, GridType::Point source,
                              GridType::Point target);
  std::tuple<int, int, bool> getClosestNode(const GridType::Point &pos);
  GridType::Point nextStep(const GridType::Point &from,
                           const GridType::Point &to);
  GridType::Point nextPoint(const GridType::Point &from,
                            const GridType::Point &dir);
  float computeAngle(double dx, double dy);
};

} // namespace Routing
} // namespace DistanceMap

#endif
