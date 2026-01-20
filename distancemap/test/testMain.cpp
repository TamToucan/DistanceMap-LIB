#include <cmath>
#include <iostream>
#include <string>

#include "FlowField.hpp"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"
/*/
#include "NavigationGraph.hpp"
*/
#include "DistanceMapCore.hpp"
#include "Router.hpp"

using namespace DistanceMap;

struct FlowField::SubGrid subgrid;

void debugFlow(FlowField::SubGrid subGrid) {
  int cols = subGrid.width;
  int rows = subGrid.height;
  const auto& flowField = subGrid.getFlow(0);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      int idx = FlowField::SubGrid::indexFor(x, y, cols);
      uint16_t costDir = flowField[idx];
      int cost = costDir >> 8;
      int dir = costDir & 0xFF;
      std::cout << (dir < 10 ? "  " : (dir < 100 ? " " : "")) << dir << " ";
    }
    std::cout << std::endl;
  }
}

std::pair<float, float> computeDirection(float angleDeg) {
  const double MYPI = 3.14159265358979323846;
  double radians = angleDeg * (MYPI / 180.0);
  return {std::cos(radians), std::sin(radians)};
}

int main(int argc, char** argv) {
  SET_DEBUG("ALL");
  auto grid = GridToGraph::readGridFromFile("GRID.txt");

  /*
  auto floorGrid = GridToGraph::gridToFloorGrid(grid);
  auto graph = GridToGraph::makeGraph(floorGrid);
  */
  auto pathGrid(grid);
  DistanceMap::Router::Info info;
  info.mCaveHeight = 32;
  info.mCellWidth = 8;
  info.mCellHeight = 8;

  DistanceMap::DistanceMapCore core;
  core.initialize(grid, info);

  auto pNavigator = core.makeNavigator(DistanceMap::NavigatorType::FLOW);

  DistanceMap::GridType::Vec2 from(300, 250);
  DistanceMap::GridType::Vec2 to(1950, 1086);
  DistanceMap::Router::RouteCtx* ctx = new DistanceMap::Router::RouteCtx();
  ctx->type = -1;
  int count = 2000;
  int mv = 1;
  bool reached_target = false;
  DistanceMap::GridType::Point toPnt = {(int)(to.x / (info.mCellWidth * DistanceMap::CELL_MULT)),
                                        (int)(to.y / (info.mCellHeight * DistanceMap::CELL_MULT))};
  DistanceMap::GridType::Point prevPnt = toPnt;
  do {
    DistanceMap::GridType::Point fromPnt = {
        (int)(from.x / (info.mCellWidth * DistanceMap::CELL_MULT)),
        (int)(from.y / (info.mCellHeight * DistanceMap::CELL_MULT))};
    reached_target =
        (fromPnt.first == toPnt.first && fromPnt.second == toPnt.second);
#if 0
    // It does loop back ro dont check this
    if (prevPnt != fromPnt) {
      if (pathGrid[fromPnt.second][fromPnt.first] == 'x') {
        std::cerr << "ERROR: LOOPED BACK TO " << fromPnt.first << ","
                  << fromPnt.second << std::endl;
        break;
      }
    }
#endif
    prevPnt = fromPnt;
    auto v = pathGrid[fromPnt.second][fromPnt.first];
    if (v && v != 'x') {
      std::cerr << "ERROR: WALL " << fromPnt.first << "," << fromPnt.second
                << std::endl;
      break;
    }
    pathGrid[fromPnt.second][fromPnt.first] = 'x';
    std::cerr << "MOVEFROM: " << fromPnt.first << "," << fromPnt.second
              << std::endl;

    float ang = core.getMoveAngle(pNavigator, ctx, from, to, 0);
    std::pair<float, float> mv = computeDirection(ang);
    from.x += mv.first * 13;
    from.y += mv.second * 13;
    DistanceMap::GridType::Point nw = {(int)(from.x / (info.mCellWidth * DistanceMap::CELL_MULT)),
                                       (int)(from.y / (info.mCellHeight * DistanceMap::CELL_MULT))};
    std::cerr << "CTV MV " << mv.first << "," << mv.second << "  ang " << ang
              << " cell: " << fromPnt.first << "," << fromPnt.second << " -> "
              << nw.first << "," << nw.second << std::endl;

  } while (!reached_target && (--count > 0));
  delete ctx;

  for (int row = 0; row < pathGrid.size(); ++row) {
    for (int col = 0; col < pathGrid[0].size(); ++col) {
      int v = pathGrid[row][col];
      if (col == toPnt.first && row == toPnt.second) {
        std::cerr << "T";
      } else if (v == 0) {
        std::cerr << " ";
      } else if (v == 'x') {
        std::cerr << "x";
      } else {
        std::cerr << "#";
      }
    }
    std::cerr << std::endl;
  }

  if (reached_target) {
    std::cerr << "OK: PATH FOUND" << std::endl;
  } else {
    std::cerr << "ERROR: NO PATH" << std::endl;
  }

  return reached_target ? 0 : 1;
}
