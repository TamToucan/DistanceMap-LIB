#include <iostream>
#include <string>

#include "GridToGraph.hpp"
#include "GridTypes.hpp"
#include "RoomDetection.hpp"
#include "WallDistanceGrid.hpp"
#include "Debug.h"

using namespace DistanceMap;

int main(int /*argc*/, char ** /*argv*/) {
  SET_DEBUG("ALL");
  auto grid = GridToGraph::readGridFromFile("GRID.txt");
  if (grid.empty()) {
    std::cerr << "ERROR: GRID.txt empty or missing" << std::endl;
    return 1;
  }

  WallDistanceGrid wallDist = makeWallDistanceGrid(grid);
  RoomMap roomMap = makeRoomMap(grid, wallDist);

  std::cout << "=== Rooms (" << roomMap.rooms.size() << ") ===" << std::endl;
  for (const auto &r : roomMap.rooms) {
    std::cout << "  id=" << r.id << " area=" << r.area
              << " center=(" << r.center.first << "," << r.center.second << ")"
              << " maxWallDist=" << r.maxWallDist
              << " w=" << r.approxWidth << " h=" << r.approxHeight
              << " neighbors=[";
    for (size_t i = 0; i < r.neighborRoomIds.size(); ++i) {
      if (i) std::cout << ",";
      std::cout << r.neighborRoomIds[i];
    }
    std::cout << "]" << std::endl;
  }

  std::cout << "=== Floor cell labels (" << roomMap.width << "x"
            << roomMap.height << ") ===" << std::endl;
  for (int y = 0; y < roomMap.height; ++y) {
    for (int x = 0; x < roomMap.width; ++x) {
      if (grid[y][x]) {
        std::cout << '#';
      } else {
        int lbl = roomMap.labels[y][x];
        if (lbl == ROOM_NONE) {
          std::cout << '.';
        } else if (lbl < 10) {
          std::cout << static_cast<char>('0' + lbl);
        } else if (lbl < 36) {
          std::cout << static_cast<char>('a' + (lbl - 10));
        } else {
          std::cout << '?';
        }
      }
    }
    std::cout << '\n';
  }
  return 0;
}
