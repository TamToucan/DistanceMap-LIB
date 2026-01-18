#include <memory>
#include <vector>

#include "DistanceMapDLL.hpp"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "NavigationFlowGrid.hpp"
#include "NavigationGraph.hpp"
#include "Router.hpp"
#include "WallDistanceGrid.hpp"

namespace DistanceMap {

class DISTANCEMAP_API DistanceMapCore {
 public:
  DistanceMapCore() = default;
  ~DistanceMapCore() = default;

  // Grid is 0 = empty, 1 = solid
  void initialize(const std::vector<std::vector<int>>& grid,
                  const Router::Info& info);
  float getMove(Router::RouteCtx* ctx, GridType::Vec2 from, GridType::Vec2 to, int type);
  GridType::Vec2 getMove(GridType::Vec2 from, float ang, float distance);

 private:
  Router::Info info;
  GridToGraph::Graph m_graph;
  GridType::Grid wallDistGrid;
  DistanceMap::SightGrid sightGrid;
  std::unique_ptr<Routing::NavigationGraph> navGraph;

  std::unique_ptr<Routing::NavigationFlowGrid> navFlow;
  NavigationAPI* pNavigator = nullptr;
};
}  // namespace DistanceMap
