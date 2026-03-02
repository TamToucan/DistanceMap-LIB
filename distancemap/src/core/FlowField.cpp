#include <cassert>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <vector>

#include "FlowField.hpp"
#include "GridToGraph.hpp"
#include "GridTypes.hpp"

#include "Debug.h"

namespace DistanceMap {
namespace FlowField {

// Maximum cost (our cost type is uint8_t, so 255 is max).
constexpr uint8_t MAX_COST = 255;
// Given uniform cost, each step costs 1.
constexpr uint8_t STEP_COST = 1;

void debugFlow(int lev, int curZone, int adjacentZone, SubGrid subGrid);

// Extract a subgrid for the target zone.
// zoneGrid: global vector of vector of GridPointInfo (size = rows x cols)
// origGrid: original grid with tile flags (e.g. WALL), same dimensions as
// zoneGrid. targetZone: the abstract node index that identifies the zone.
SubGrid extractZoneSubGrid(
    const std::vector<std::vector<GridType::GridPointInfo>> &zoneGrid,
    const std::vector<std::vector<int>> &origGrid, int targetZone) {
  int rows = zoneGrid.size();
  int cols = zoneGrid[0].size();

  // First, find the bounding box of cells that belong to the target zone.
  int minX = cols;
  int minY = rows;
  int maxX = -1;
  int maxY = -1;
  for (int y = 1; y < rows - 1; ++y) {
    for (int x = 1; x < cols - 1; ++x) {
      if (zoneGrid[y][x].closestAbstractNodeIdx == targetZone) {
        if (x < minX)
          minX = x;
        if (x > maxX)
          maxX = x;
        if (y < minY)
          minY = y;
        if (y > maxY)
          maxY = y;
      }
    }
  }
  LOG_DEBUG("Zone: " << targetZone << " bounding box: (" << minX << ", " << minY
                     << ")"
                     << "to(" << maxX << ", " << maxY << ")");

  // Validate that we found at least one cell for the zone.
  if (maxX < minX || maxY < minY)
    throw std::runtime_error("The target zone has no cells in the grid.");

  int subWidth = maxX - minX + 1;
  int subHeight = maxY - minY + 1;

  SubGrid subGrid;
  subGrid.width = subWidth;
  subGrid.height = subHeight;
  subGrid.offsetX = minX;
  subGrid.offsetY = minY;
  subGrid.grid.resize(subWidth * subHeight,
                      GridType::WALL); // Default: mark as WALL.

  // Fill subGrid: For each global cell in the bounding box,
  // if it belongs to targetZone and is not a wall in origGrid, mark it as
  // walkable (0). Otherwise, mark it as WALL.
  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x <= maxX; ++x) {
      int localX = x - minX;
      int localY = y - minY;
      int idx = localY * subWidth + localX;
      // Check: belongs to target zone and not a wall.
      if (zoneGrid[y][x].closestAbstractNodeIdx == targetZone &&
          (origGrid[y][x] & GridType::WALL) == 0) {
        subGrid.grid[idx] = 0; // Walkable.
        subGrid.floorCells.push_back({x, y});
      } else {
        subGrid.grid[idx] =
            GridType::WALL; // Not part of the zone or is a wall.
      }
    }
  }

  return subGrid;
}

std::vector<GridType::BoundaryInfo>
convertSinksToLocal(const GridType::BoundaryCells &globalSinks,
                    const SubGrid &subGrid) {
  std::vector<GridType::BoundaryInfo> localSinks;
  for (const auto &boundary : globalSinks) {
    int gx = boundary.sink.first;
    int gy = boundary.sink.second;
    // Only include sinks that are within the bounding box.
    if (gx >= subGrid.offsetX && gx < subGrid.offsetX + subGrid.width &&
        gy >= subGrid.offsetY && gy < subGrid.offsetY + subGrid.height) {
      int localX = gx - subGrid.offsetX;
      int localY = gy - subGrid.offsetY;
      localSinks.emplace_back(localX, localY, boundary.exitDirIdx);
    }
  }
  return localSinks;
}

// #########################################################################################

std::vector<uint16_t>
generateFlowFieldDial(const SubGrid &subGrid,
                      const std::vector<GridType::BoundaryInfo> &sinks) {
  const std::vector<int> &grid = subGrid.grid;
  int rows = subGrid.height;
  int cols = subGrid.width;
  // Allocate flat flow field; each cell's value is stored as:
  //   (cost << 8) | direction, where cost is in [0, MAX_COST] and direction in
  //   [0,7]
  // Initialize to MAX_COST and NO_DIR.
  const int cellCount = rows * cols;
  std::vector<uint16_t> costFlowField(cellCount, (MAX_COST << 8) | NO_DIR);

  // Create buckets: one bucket per cost value from 0 to MAX_COST.
  // Each bucket is a vector of flat indices (cells with that cost).
  std::vector<std::vector<int>> buckets(MAX_COST + 1);

  // Initialize sink cells.
  for (const auto &bi : sinks) {
    int x = bi.sink.first;
    int y = bi.sink.second;
    int idx = SubGrid::indexForLocal(x, y, cols);
    costFlowField[idx] =
        0 | SINK_BIT | bi.exitDirIdx; // cost 0, SINK, dir to adjacent zone
    buckets[0].push_back(idx);
  }

  // Dial's algorithm: iterate cost by cost.
  uint8_t cost = 0;
  while (cost <= MAX_COST) {
    auto &bucket = buckets[cost];
    if (bucket.empty()) {
      // no cells at this cost, move on
      ++cost;
      if (cost == 0)
        break;
      continue;
    }

    // Pop one cell from the back of the current-cost bucket
    int curIdx = bucket.back();
    bucket.pop_back();
    // Decode current cell's coordinates.
    int x = curIdx % cols;
    int y = curIdx / cols;
    uint8_t newCost = cost + STEP_COST;

    // For each of the 8 neighbors.
    for (int d = 0; d < 8; ++d) {
      int nx = x + GridType::directions8[d].first;
      int ny = y + GridType::directions8[d].second;
      // Check bounds
      if (!subGrid.isInsideLocal(nx, ny))
        continue;

      // Check wall
      int nIdx = SubGrid::indexForLocal(nx, ny, cols);
      if (grid[nIdx] & GridType::WALL)
        continue;

      uint8_t neighborCost = costFlowField[nIdx] >> 8;

      // If we found a shorter path to the neighbor.
      if (newCost < neighborCost) {
        // Store the new cost and the direction.
        // Here we store the reverse direction:
        // When moving from neighbor to current cell,
        // the opposite of d is (d + 4) % 8.
        costFlowField[nIdx] = (newCost << 8) | GridType::reverseDirIndex[d];
        buckets[newCost].push_back(nIdx);
      }
    }
  }
  return costFlowField;
}

// #########################################################################################

void generateFlowGrids(GridToGraph::Graph &graph) {
  const int rows = graph.infoGrid.size();
  const int cols = graph.infoGrid[0].size();

  LOG_INFO("##GenerateFlowGrids from AbstractLevels: "
           << graph.abstractLevels.size());
  for (int i = 0; i < graph.abstractLevels.size(); ++i) {
    auto &ablv = graph.abstractLevels[i];
    LOG_DEBUG("== Get subgrid bounding box for level: " << i);
    for (int zi = 0; zi < ablv.zones.size(); ++zi) {
      auto subgrid = extractZoneSubGrid(ablv.zoneGrid, graph.infoGrid, zi);
      ablv.subGrids.push_back(subgrid);
    }
    LOG_DEBUG("==> AbstractLevel: " << i
                                    << " subgrids: " << ablv.subGrids.size());
  }

  LOG_DEBUG("## FLOW: AbstractLevels: " << graph.abstractLevels.size());
  for (int levIdx = 0; levIdx < graph.abstractLevels.size(); ++levIdx) {
    auto &ablv = graph.abstractLevels[levIdx];
    LOG_DEBUG("  AbstractLevel: " << levIdx << " zones: " << ablv.zones.size());
    for (int zi = 0; zi < ablv.zones.size(); ++zi) {
      auto &subGrid = ablv.subGrids[zi];
      const auto &adjZones = ablv.zones[zi].adjacentZones;
      LOG_DEBUG("    Zone: " << zi << " subGrid: " << subGrid.offsetX << ", "
                             << subGrid.offsetY << " " << subGrid.width << "x"
                             << subGrid.height
                             << " adjZones: " << adjZones.size());
      for (int ni = 0; ni < adjZones.size(); ++ni) {
        int adjZone = adjZones[ni];
        const auto &boundaryCells = ablv.zones[zi].zoneBoundaryCellMap[adjZone];
        auto localSinks = convertSinksToLocal(boundaryCells, subGrid);

        LOG_DEBUG("      zone:" << adjZone << " sinks: " << localSinks.size());
        for (const auto &bi : localSinks) {
          LOG_DEBUG("        sink: " << (bi.sink.first + subGrid.offsetX) << ","
                                     << (bi.sink.second + subGrid.offsetY)
                                     << " dir: " << bi.exitDirIdx);
        }
        subGrid.costFlowFields.push_back(
            {adjZone, generateFlowFieldDial(subGrid, localSinks)});
        debugFlow(levIdx, zi, adjZone, subGrid);
      }
    }
  }
}

void debugFlow(int lev, int curZone, int adjacentZone, SubGrid subGrid) {
  std::ostringstream oss;
  oss << "FLOW_" << lev << "_z_" << curZone << "_to_" << adjacentZone << ".txt";
  std::ofstream outFile(oss.str());

  int cols = subGrid.width;
  int rows = subGrid.height;
  const auto &flowField = subGrid.getFlow(adjacentZone);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      int idx = SubGrid::indexForLocal(x, y, cols);
      uint16_t costDir = flowField[idx];
      int cost = costDir >> 8;
      int dir = costDir & 0xFF;
      outFile << (dir < 10 ? "  " : (dir < 100 ? " " : "")) << dir << " ";
    }
    outFile << std::endl;
  }
}

} // namespace FlowField
} // namespace DistanceMap
