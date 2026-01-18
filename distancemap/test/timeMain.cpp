#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "NavigationFlowGrid.hpp"
#include "NavigationGraph.hpp"
#include "Router.hpp"

using namespace DistanceMap;

std::pair<float, float> computeDirection(float angleDeg) {
  const double MYPI = 3.14159265358979323846;
  double radians = angleDeg * (MYPI / 180.0);
  return {std::cos(radians), std::sin(radians)};
}

bool testNavigator(const std::string& name,
                   std::function<float(DistanceMap::Router::RouteCtx*,
                                       DistanceMap::GridType::Vec2,
                                       DistanceMap::GridType::Vec2, int)>
                       getDirection,
                   const GridToGraph::Graph& graph,
                   const DistanceMap::Router::Info& info,
                   DistanceMap::GridType::Vec2 from,
                   DistanceMap::GridType::Vec2 to) {
  auto pathGrid = graph.infoGrid;
  DistanceMap::Router::RouteCtx* ctx = new DistanceMap::Router::RouteCtx();
  ctx->type = -1;

  int count = 2000;
  bool reached_target = false;
  GridType::Point toPnt = {static_cast<int>(to.x / (info.mCellWidth * DistanceMap::CELL_MULT)),
                           static_cast<int>(to.y / (info.mCellHeight * DistanceMap::CELL_MULT))};
  GridType::Point prevPnt = toPnt;

  std::cout << "\n=== Testing " << name << " ===" << std::endl;
  std::cout << "From: " << from.x << "," << from.y << " To: " << to.x << ","
            << to.y << std::endl;

  auto startTime = std::chrono::high_resolution_clock::now();

  do {
    GridType::Point fromPnt = {
        static_cast<int>(from.x / (info.mCellWidth * DistanceMap::CELL_MULT)),
        static_cast<int>(from.y / (info.mCellHeight * DistanceMap::CELL_MULT))};
    reached_target =
        (fromPnt.first == toPnt.first && fromPnt.second == toPnt.second);

#if 0
    if (prevPnt != fromPnt) {
      if (pathGrid[fromPnt.second][fromPnt.first] == 'x') {
        std::cerr << "ERROR: LOOPED BACK TO " << fromPnt.first << ","
                  << fromPnt.second << std::endl;
        break;
      }
    }
#endif
    prevPnt = fromPnt;

    if (pathGrid[fromPnt.second][fromPnt.first] & GridType::WALL) {
      std::cerr << "ERROR: WALL " << fromPnt.first << "," << fromPnt.second
                << std::endl;
      break;
    }
    pathGrid[fromPnt.second][fromPnt.first] = 'x';

    float ang = getDirection(ctx, from, to, 0);
    std::pair<float, float> mv = computeDirection(ang);
    from.x += mv.first * 13;
    from.y += mv.second * 13;

  } while (!reached_target && (--count > 0));

  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);

  delete ctx;

  if (reached_target) {
    std::cout << "✓ PATH FOUND in " << (2000 - count) << " steps" << std::endl;
    std::cout << "  Time: " << duration.count() << " μs" << std::endl;
  } else {
    std::cout << "✗ NO PATH (stopped after " << (2000 - count) << " steps)"
              << std::endl;
  }

  return reached_target;
}

int main(int argc, char** argv) {
  std::cout << "=== Distance Map Navigator Test ===" << std::endl;

  auto grid = GridToGraph::readGridFromFile("GRID.txt");
  auto floorGrid = GridToGraph::gridToFloorGrid(grid);
  auto graph = GridToGraph::makeGraph(floorGrid);

  DistanceMap::Router::Info info;
  info.mCaveHeight = 32;
  info.mCellWidth = 8;
  info.mCellHeight = 8;

  // Initialize both navigators
  Routing::NavigationGraph navGraph(graph, info);
  Routing::NavigationFlowGrid distMapNav(graph.infoGrid, info);

  // Test cases
  GridType::Vec2 from1(300, 250);
  GridType::Vec2 to1(1950, 1086);

  // Test original NavigationGraph
  LOG_INFO("## ======= DISTANCEMAP TEST =======");
  bool result1 = testNavigator(
      "NavigationGraph (Original)",
      [&navGraph](DistanceMap::Router::RouteCtx* ctx,
                  DistanceMap::GridType::Vec2 from,
                  DistanceMap::GridType::Vec2 to, int type) {
        return navGraph.getMoveDirection(ctx, from, to, type);
      },
      graph, info, from1, to1);

  // Test new NavigationFlowGrid
  LOG_INFO("## ======= NAVIGATOR TEST =======");
  bool result2 = testNavigator(
      "NavigationFlowGrid (New)",
      [&distMapNav](DistanceMap::Router::RouteCtx* ctx,
                    DistanceMap::GridType::Vec2 from,
                    DistanceMap::GridType::Vec2 to, int type) {
        return distMapNav.getMoveDirection(ctx, from, to, type);
      },
      graph, info, from1, to1);

  // Performance test: Multiple agents
  std::cout << "\n=== Performance Test: 100 Agents ===" << std::endl;

  std::vector<GridType::Vec2> agents;
  for (int i = 0; i < 100; ++i) {
    agents.push_back(GridType::Vec2(300 + i * 10, 250 + i * 5));
  }
  GridType::Vec2 target(1830, 986);

  // Test NavigationFlowGrid with multiple agents
  auto startTime = std::chrono::high_resolution_clock::now();
  for (int frame = 0; frame < 10; ++frame) {
    for (auto& agent : agents) {
      DistanceMap::Router::RouteCtx ctx;
      ctx.type = -1;
      distMapNav.getMoveDirection(&ctx, agent, target, 0);
    }
  }
  auto endTime = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      endTime - startTime);

  std::cout << "NavigationFlowGrid: 100 agents × 10 frames = "
            << duration.count() << " μs" << std::endl;
  std::cout << "  Average per query: " << (duration.count() / 2000.0) << " μs"
            << std::endl;

  // Summary
  std::cout << "\n=== Summary ===" << std::endl;
  if (result1 && result2) {
    std::cout << "✓ Both implementations found valid paths" << std::endl;
    return 0;
  } else {
    std::cout << "✗ One or both implementations failed" << std::endl;
    return 1;
  }
}
