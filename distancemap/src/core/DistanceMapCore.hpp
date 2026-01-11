#pragma once
#include "DistanceMapApi.h"
#include "DistanceMapNavigator.hpp"
#include "GridTypes.hpp"
#include "NavigationGraph.hpp"
#include "Router.hpp"
#include "WallDistanceGrid.hpp"
#include <vector>

namespace DistanceMap {
class DISTANCEMAP_API DistanceMapCore {
public:
  DistanceMapCore() = default;
  ~DistanceMapCore() = default;

  // Grid is 0 = empty, 1 = solid
  void initialize(const std::vector<std::vector<int>> &grid,
                  const Router::Info &info);
  float getMove(Router::RouteCtx *ctx, GridType::Vec2 from, GridType::Vec2 to,
                int type);

private:
  Router::Info info;
  GridType::Grid wallDistGrid;
  DistanceMap::SightGrid sightGrid;
  Routing::NavigationGraph navGraph;
  Routing::DistanceMapNavigator distMapNav;
};
} // namespace DistanceMap
