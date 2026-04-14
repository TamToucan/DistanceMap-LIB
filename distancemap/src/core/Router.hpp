#pragma once

#include <vector>

namespace DistanceMap {
namespace Router {

/// Minimal A* route cache — used by SparseNavGraph for bidirectional A* bookkeeping.
/// Navigator-specific state (stuck detection, waypoint commitment, per-navigator
/// substates) lives in DistanceMap::NavCtx in the CuteLott repo.
struct RouteCtx {
  std::vector<int> routeNodes;

  int routeSrcNodeIdx  = -1;
  int routeTgtNodeIdx  = -1;
  int routeSrcNodeIdx2 = -1;
  int routeTgtEdgeIdx  = -1;
  int routeSrcEdgeIdx  = -1;
  int routeTgtNodeIdx2 = -1;
  int routeSrcEdgeIdx2 = -1;
  int routeTgtEdgeIdx2 = -1;

  enum class RouteType { None, NodeToNode, NodeToEdge, EdgeToNode, EdgeToEdge };
  RouteType lastRouteType = RouteType::None;
};

/// Grid dimension / cell-size configuration passed to navigator constructors.
struct Info {
  int mCaveWidth   = 2;
  int mCaveHeight  = 2;
  int mBorderWidth = 1;
  int mBorderHeight = 1;
  int mCellWidth   = 1;
  int mCellHeight  = 1;
  int mStartCellX  = 0;
  int mStartCellY  = 0;
};

} // namespace Router
} // namespace DistanceMap
