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
// NavigationGraph uses the GridToGraph::Grid data to navigate.
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
  NavigationGraph(const GridToGraph::Graph &graphData,
                  const Router::Info &info);
  ~NavigationGraph() = default;

  // Main entry point for movement
  float getMoveDirection(Router::RouteCtx *ctx, GridType::Vec2 from,
                         GridType::Vec2 to, int type, float dt) override;

  float stuck(Router::RouteCtx *ctx, GridType::Vec2 from,
              GridType::Vec2 to, int type, float dt) override;

  const std::vector<GridToGraph::AbstractLevel> *
  getAbstractLevels() const override;

private:
  // Data members from GridToGraph::Graph
  const GridToGraph::BaseGraph &m_baseGraph;
  const GridToGraph::PathCostMap &m_pathCostMap;
  const SparseNavGraph &m_routingGraph;
  const std::vector<GridType::Edge> &m_baseEdges;
  const std::vector<GridType::Point> &m_baseNodes;
  const std::vector<GridType::Point> &m_deadEnds;
  const std::vector<GridToGraph::AbstractLevel> &m_abstractLevels;

  // Cell classification
  enum class CellKind { NODE, EDGE, DEND, XPND, OTHER };
  static CellKind classifyCell(int cell);

  // Closest graph point lookup (fixes stale edgeIdx after XPND walk)
  struct ClosestNodeInfo {
    int nodeIdx;
    int edgeOrDeadEndIdx; // edge index if isEdge, dead-end index if !isEdge
    bool isEdge;
    GridType::Point closestGraphPoint;
  };
  ClosestNodeInfo getClosestNode(const GridType::Point &pos) const;

  // Zone-level routing helpers
  struct ZoneInfo {
    int sourceZone    = -1;
    int targetZone    = -1;
    int ablvIdx       = -1;  // -1 = not found (use top level), >=0 = level index
    int relationship  = -2;  // -1 same zone, >=0 adjacent level, -2 distant
  };
  ZoneInfo findZoneRelationship(const GridType::Point &source,
                                const GridType::Point &target) const;

  // Route management: populate ctx->routeNodes if stale/empty; returns true on success
  bool ensureRoute(Router::RouteCtx *ctx,
                   const ZoneInfo &zones,
                   std::optional<int> srcNodeIdx,
                   std::optional<int> srcEdgeIdx,
                   int tgtNodeIdx) const;

  // Per-cell-type stepping (each returns the next grid cell to move into)
  GridType::Point stepFromNode   (Router::RouteCtx *ctx, int srcNode,
                                  GridType::Point source) const;
  GridType::Point stepFromEdge   (Router::RouteCtx *ctx, int edgeIdx,
                                  GridType::Point source) const;
  GridType::Point stepFromDeadEnd(int dendIdx, GridType::Point source) const;

  // Main per-frame step logic
  GridType::Point getNextMove(Router::RouteCtx *ctx, GridType::Point source,
                              GridType::Point target);

  float computeAngle(double dx, double dy) const;
};

} // namespace Routing
} // namespace DistanceMap

#endif
