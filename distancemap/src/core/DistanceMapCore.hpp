#include <memory>
#include <vector>

#include "DistanceMapDLL.hpp"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "NavigationAPI.hpp"
#include "Router.hpp"
#include "WallDistanceGrid.hpp"

namespace DistanceMap {

enum NavigatorType {
  FLOW,
  GRAPH,
};

class DISTANCEMAP_API DistanceMapCore {
public:
  DistanceMapCore() = default;
  ~DistanceMapCore() = default;

  // Grid is 0 = empty, 1 = solid
  void initialize(const std::vector<std::vector<int>> &grid,
                  const Router::Info &info);

  std::unique_ptr<NavigationAPI> makeNavigator(NavigatorType type);

  float getMoveAngle(const std::unique_ptr<NavigationAPI> &pNavigator,
                     Router::RouteCtx *ctx, GridType::Vec2 from,
                     GridType::Vec2 to, int type);
  GridType::Vec2 getMovePos(const std::unique_ptr<NavigationAPI> &pNavigator,
                            GridType::Vec2 from, float ang, float distance);

private:
  Router::Info m_info;
  GridToGraph::Graph m_graph;
  GridType::Grid wallDistGrid;
  DistanceMap::SightGrid sightGrid;
};
} // namespace DistanceMap
