#ifndef NAVIGATION_GRAPH_HPP
#define NAVIGATION_GRAPH_HPP

#include <vector>

#include "DistanceMapDLL.hpp"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "NavigationAPI.hpp"
#include "Router.hpp"
#include "SparseNavGraph.hpp"

namespace DistanceMap {
namespace Routing {

//
// NavigationFlowGrid uses the GridToGraph::Grid data to navigate.
// The heart of it is to find the source/target abstract level zones
// and then
// - if same then use EDGE/NODE/XPND to move towards the target
// - if adjacent then move to the boundary of the source zone
// - otherwise it sort of moved towards the target
// The world flot/grid int can cause a diagonal to move vert/horz
// which if the target is clipping a wall will mean it moves into
// the wall.
// So validateMove/resolveMove area used to move the agent slide
// long the wall towards target

class DISTANCEMAP_API NavigationGraph : public NavigationAPI {
 public:
  NavigationGraph(const GridToGraph::Graph& graphData,
                  const Router::Info& info);
  ~NavigationGraph() = default;

  // Main entry point for movement
  float getMoveDirection(Router::RouteCtx* ctx, GridType::Vec2 from,
                         GridType::Vec2 to, int type) override;

 private:
  // Data members from GridToGraph::Graph
  const GridToGraph::BaseGraph& m_baseGraph;
  const GridToGraph::PathCostMap& m_pathCostMap;
  const SparseNavGraph& m_routingGraph;
  const std::vector<GridType::Edge>& m_baseEdges;
  const std::vector<GridType::Point>& m_baseNodes;
  const std::vector<GridType::Point>& m_deadEnds;
  const std::vector<GridToGraph::AbstractLevel>& m_abstractLevels;

  // Helpers
  struct ClosestNodeInfo {
    int nodeIdx;
    int edgeOrDeadEndIdx;
    bool isEdge;
    GridType::Point closestGraphPoint;
  };

  GridType::Point getNextMove(Router::RouteCtx* ctx, GridType::Point source,
                              GridType::Point target);
  ClosestNodeInfo getClosestNode(const GridType::Point& pos);

  GridType::Point nextStep(const GridType::Point& from,
                           const GridType::Point& to);
  GridType::Point nextPoint(const GridType::Point& from,
                            const GridType::Point& dir);
  float computeAngle(double dx, double dy);
};

}  // namespace Routing
}  // namespace DistanceMap

#endif
