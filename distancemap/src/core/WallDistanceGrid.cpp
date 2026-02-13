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
std::vector<std::vector<int>> makeWallDistanceGrid(const GridType::Grid &grid) {
  int rows = grid.size();
  int cols = grid[0].size();

  // Directions for moving (up, down, left, right)
  std::vector<std::pair<int, int>> directions = {
      {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

  // Initialize the distance grid with a large value
  std::vector<std::vector<int>> distanceGrid(
      rows, std::vector<int>(cols, std::numeric_limits<int>::max()));

  // BFS queue to store cells (row, col)
  std::queue<std::pair<int, int>> bfsQueue;

  // Add all wall cells to the queue and initialize their distances
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      if (grid[r][c] & GridType::WALL) {
        distanceGrid[r][c] = 0;
        bfsQueue.push({r, c});
      }
    }
  }

  // Perform BFS
  while (!bfsQueue.empty()) {
    auto [current_r, current_c] = bfsQueue.front();
    bfsQueue.pop();

    // Current cell's distance
    int current_distance = distanceGrid[current_r][current_c];

    // Process neighbors
    for (const auto &[dr, dc] : directions) {
      int neighbor_r = current_r + dr;
      int neighbor_c = current_c + dc;

      // Ensure neighbor is within bounds and update its distance if shorter
      if (neighbor_r >= 0 && neighbor_r < rows && neighbor_c >= 0 &&
          neighbor_c < cols) {
        if (distanceGrid[neighbor_r][neighbor_c] > current_distance + 1) {
          distanceGrid[neighbor_r][neighbor_c] = current_distance + 1;
          bfsQueue.push({neighbor_r, neighbor_c});
        }
      }
    }
  }

  return distanceGrid;
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
