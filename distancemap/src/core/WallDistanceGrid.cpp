/*
 * WallDistanceGrid.cpp
 *
 *  Created on: 31 Dec 2024
 *      Author: tam
 */

#include "WallDistanceGrid.hpp"

#include <limits>
#include <queue>
#include <vector>

namespace DistanceMap {

//
// non-0 = wall
//
WallDistanceGrid makeWallDistanceGrid(const GridType::Grid &grid) {
  int rows = static_cast<int>(grid.size());
  int cols = static_cast<int>(grid[0].size());

  // Directions for moving (up, down, left, right)
  const std::pair<int,int> directions[4] = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  // BFS with int distances internally; clamp to uint8 on output.
  std::vector<int> dist(static_cast<size_t>(rows) * cols,
                        std::numeric_limits<int>::max());

  // BFS queue to store cells (row, col)
  std::queue<std::pair<int, int>> bfsQueue;

  // Add all wall cells to the queue and initialise their distances to 0
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (grid[r][c] != 0) {
        dist[r * cols + c] = 0;
        bfsQueue.push({r, c});
      }
    }
  }

  // Perform BFS
  while (!bfsQueue.empty()) {
    auto [cr, cc] = bfsQueue.front();
    bfsQueue.pop();
    int current_distance = dist[cr * cols + cc];
    for (const auto &[dr, dc] : directions) {
      int nr = cr + dr;
      int nc = cc + dc;
      if (nr >= 0 && nr < rows && nc >= 0 && nc < cols) {
        int &nd = dist[nr * cols + nc];
        if (nd > current_distance + 1) {
          nd = current_distance + 1;
          bfsQueue.push({nr, nc});
        }
      }
    }
  }

  // Pack into WallDistanceGrid, capping values at 255
  WallDistanceGrid result;
  result.width  = cols;
  result.height = rows;
  result.data.resize(static_cast<size_t>(rows) * cols);
  for (int i = 0; i < rows * cols; ++i) {
    int v = dist[i];
    result.data[i] = static_cast<uint8_t>(v > 255 ? 255 : v);
  }
  return result;
}

///////////////////////////////////////////////////////////////////////

//
// Function to compute directional distances
//
SightGrid makeSightGrid(const GridType::Grid &grid) {
  int rows = grid.size();
  int cols = grid[0].size();

  // Initialize a 3D vector to store distances in 4 directions8 for each cell
  // distance[r][c][0 1 2 3] = North, East, South, West
  std::vector<std::vector<std::vector<int>>> distance(
      rows, std::vector<std::vector<int>>(cols, std::vector<int>(4, -1)));

  // 0. North Sweep
  for (int c = 0; c < cols; ++c) {
    for (int r = 0; r < rows; ++r) {
      if (grid[r][c]) {
        distance[r][c][0] = 0; // Wall itself
      } else if (r > 0) {
        distance[r][c][0] =
            distance[r - 1][c][0] + 1; // Distance from the cell above
      } else {
        distance[r][c][0] = -1; // No wall in this direction
      }
    }
  }

  // 1. East Sweep
  for (int r = 0; r < rows; ++r) {
    for (int c = cols - 1; c >= 0; --c) {
      if (grid[r][c]) {
        distance[r][c][1] = 0; // Wall itself
      } else if (c < cols - 1) {
        distance[r][c][1] =
            distance[r][c + 1][1] + 1; // Distance from the cell to the right
      } else {
        distance[r][c][1] = -1; // No wall in this direction
      }
    }
  }

  // 2. South Sweep
  for (int c = 0; c < cols; ++c) {
    for (int r = rows - 1; r >= 0; --r) {
      if (grid[r][c]) {
        distance[r][c][2] = 0; // Wall itself
      } else if (r < rows - 1) {
        distance[r][c][2] =
            distance[r + 1][c][2] + 1; // Distance from the cell below
      } else {
        distance[r][c][2] = -1; // No wall in this direction
      }
    }
  }

  // 3. West Sweep
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (grid[r][c]) {
        distance[r][c][3] = 0; // Wall itself
      } else if (c > 0) {
        distance[r][c][3] =
            distance[r][c - 1][3] + 1; // Distance from the cell to the left
      } else {
        distance[r][c][3] = -1; // No wall in this direction
      }
    }
  }

  SightGrid sight;
  sight.sight = std::vector<std::vector<int>>(rows, std::vector<int>(cols, 0));

  //
  // Pack those distances into an int
  //
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      int packed = 0;
      // 0..3 = N,E,S,W
      for (int d = 0; d < 4; ++d) {
        int dist = distance[r][c][d];
        dist = std::min(dist, 0xff);
        packed |= dist << (8 * d);
      }
      sight.sight[r][c] = packed;
    }
  }
  return sight;
}

} // namespace DistanceMap
