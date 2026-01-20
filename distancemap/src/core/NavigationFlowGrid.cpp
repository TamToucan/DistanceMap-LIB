#include "NavigationFlowGrid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

#include "Debug.h"
#include "MathUtils.h"
#include "NavigationAPI.hpp"

namespace DistanceMap {
namespace Routing {

NavigationFlowGrid::NavigationFlowGrid(const GridType::Grid& infoGrid,
                                       const Router::Info& info)
    : NavigationAPI(infoGrid, info), m_cacheValid(false), m_cachedTarget(-1, -1) {
  // Pre-allocate distance map with same dimensions as infoGrid
  m_distanceMap.resize(m_gridHeight, std::vector<Cell>(m_gridWidth));
  LOG_DEBUG("NavigationFlowGrid initialized");
}

float NavigationFlowGrid::computeAngle(double dx, double dy) {
  if (dx == 0 && dy == 0) {
    return 0.0;  // No movement
  }
  return static_cast<float>(
      std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}

GridType::Point NavigationFlowGrid::nextPoint(const GridType::Point& from,
                                              const GridType::Point& dir) {
  return {from.first + dir.first, from.second + dir.second};
}

void NavigationFlowGrid::computeDistanceMap(GridType::Point target) {
  // Check if we can reuse cached distance map
  if (m_cacheValid && m_cachedTarget == target) {
    LOG_DEBUG("DistanceMap: Reusing cached map for target "
              << target.first << "," << target.second);
    return;
  }

  LOG_DEBUG("DistanceMap: Computing new map for target " << target.first << ","
                                                         << target.second);

  int rows = m_infoGrid.size();
  int cols = m_infoGrid[0].size();

  // Reset all cells to uncomputed state
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      m_distanceMap[y][x] = Cell();
    }
  }

  // Priority queue: (distance, point)
  using QueueItem = std::pair<uint16_t, GridType::Point>;
  auto cmp = [](const QueueItem& a, const QueueItem& b) {
    return a.first > b.first;
  };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(cmp)> queue(
      cmp);

  // Initialize target cell
  m_distanceMap[target.second][target.first].distance = 0;
  m_distanceMap[target.second][target.first].direction = SINK;
  queue.push({0, target});

  int cellsProcessed = 0;

  // Dijkstra's algorithm
  while (!queue.empty()) {
    auto [currentDist, current] = queue.top();
    queue.pop();

    int x = current.first;
    int y = current.second;

    // Skip if we've already found a better path
    if (currentDist > m_distanceMap[y][x].distance) {
      continue;
    }

    cellsProcessed++;

    // Explore all 8 neighbors
    for (int dirIdx = 0; dirIdx < 8; ++dirIdx) {
      const auto& dir = GridType::directions8[dirIdx];
      int nx = x + dir.first;
      int ny = y + dir.second;

      // Check bounds
      if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) {
        continue;
      }

      int neighborCell = m_infoGrid[ny][nx];

      // Skip walls
      // Note: We used to allow boundaries, but that causes the agent to walk
      // INTO the wall.
      if (neighborCell & GridType::WALL) {
        continue;
      }

      // Corner Checking:
      // If moving diagonally (odd dirIdx), check adjacent cardinals.
      // If either is a WALL, we can't move diagonally (corner cut).
      if (dirIdx % 2 != 0) {
        int c1 = (dirIdx - 1) & 7;  // Previous Cardinal
        int c2 = (dirIdx + 1) % 8;  // Next Cardinal

        // Helper to check if a neighbor is a blocking wall
        // FIX: Treat BOUNDARY as blocking for CORNER checks to avoid cutting
        // through walls
        auto isBlocking = [&](int cx, int cy) {
          if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
            return true;  // Bounds check (treat as wall)
          int cell = m_infoGrid[cy][cx];
          // We MUST block boundaries here, otherwise we try to walk through the
          // wall corner
          return (cell & GridType::WALL) != 0;
        };

        if (isBlocking(x + GridType::directions8[c1].first,
                       y + GridType::directions8[c1].second) ||
            isBlocking(x + GridType::directions8[c2].first,
                       y + GridType::directions8[c2].second)) {
          continue;
        }
      }

      // Calculate new distance
      // Cost: 10 for Cardinal, 14 for Diagonal
      uint16_t moveCost = (dirIdx % 2 == 0) ? 10 : 14;
      uint16_t newDist = currentDist + moveCost;

      // Check for overflow
      if (newDist >= std::numeric_limits<uint16_t>::max()) {
        continue;
      }

      // Update if we found a shorter path
      if (newDist < m_distanceMap[ny][nx].distance) {
        m_distanceMap[ny][nx].distance = newDist;
        // Store the REVERSE direction (from neighbor back to current, which
        // points toward target)
        m_distanceMap[ny][nx].direction = GridType::reverseDirIndex[dirIdx];
        queue.push({newDist, {nx, ny}});
      }
    }
  }

  // Update cache
  m_cachedTarget = target;
  m_cacheValid = true;

  for (const auto& row : m_distanceMap) {
    for (const auto& cell : row) {
      if (cell.distance == std::numeric_limits<uint16_t>::max()) {
        LOG_DEBUG_CONT("XXX");
      } else {
        LOG_DEBUG_CONT(std::setw(3) << cell.distance << " ");
      }
    }
    LOG_DEBUG("");
  }
  LOG_DEBUG("DistanceMap: Processed " << cellsProcessed << " cells");
}

GridType::Point NavigationFlowGrid::getNextMove(GridType::Point source,
                                                GridType::Point target) {
  // Handle source in wall
  int srcCell = m_infoGrid[source.second][source.first];
  if (srcCell & GridType::WALL) {
    // If boundary, move out of wall
    if (srcCell & GridType::BOUNDARY) {
      int dirIdx = srcCell & GridType::DIR_MASK;
      source.first += GridType::directions8[dirIdx].first;
      source.second += GridType::directions8[dirIdx].second;
      LOG_DEBUG("DistanceMap: Source in BOUNDARY, moved to "
                << source.first << "," << source.second);
    }

    // Check if still in wall
    srcCell = m_infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL) {
      LOG_ERROR("DistanceMap: Source " << source.first << "," << source.second
                                       << " is in WALL");
      return source;  // Can't move
    }
  }

  // Handle target in wall
  int tgtCell = m_infoGrid[target.second][target.first];
  if (tgtCell & GridType::WALL) {
    // If boundary, move out of wall
    if (tgtCell & GridType::BOUNDARY) {
      int dirIdx = tgtCell & GridType::DIR_MASK;
      target.first += GridType::directions8[dirIdx].first;
      target.second += GridType::directions8[dirIdx].second;
      LOG_DEBUG("DistanceMap: Target in BOUNDARY, moved to "
                << target.first << "," << target.second);
    }

    // Check if still in wall
    if (m_infoGrid[target.second][target.first] & GridType::WALL) {
      LOG_ERROR("DistanceMap: Target " << target.first << "," << target.second
                                       << " is in WALL");
      return source;  // Can't move
    }
  }

  // Compute/validate distance map
  computeDistanceMap(target);

  // If source is target, we're done
  if (source == target) {
    return source;
  }

  // Look up direction at source
  const Cell& cell = m_distanceMap[source.second][source.first];

  // Check if target is reachable
  if (cell.direction == NO_DIR ||
      cell.distance == std::numeric_limits<uint16_t>::max()) {
    LOG_DEBUG("DistanceMap: Target unreachable from "
              << source.first << "," << source.second << " - moving directly");
    // Fallback: move directly toward target
    int dx = target.first - source.first;
    int dy = target.second - source.second;
    // Clamp to [-1, 1]
    dx = std::max(-1, std::min(1, dx));
    dy = std::max(-1, std::min(1, dy));
    return nextPoint(source, {dx, dy});
  }

  // If we're at the sink, stay put
  if (cell.direction == SINK) {
    return source;
  }

  // Move in the computed direction
  const auto& dir = GridType::directions8[cell.direction];
  GridType::Point next = nextPoint(source, dir);

  LOG_DEBUG("DistanceMap: " << source.first << "," << source.second << " -> "
                            << next.first << "," << next.second
                            << " (dir=" << (int)cell.direction
                            << " dist=" << cell.distance << ")");

  return next;
}

float NavigationFlowGrid::getMoveDirection(Router::RouteCtx* ctx,
                                           GridType::Vec2 from,
                                           GridType::Vec2 to, int type) {
  // Convert world coordinates to grid coordinates
  GridType::Point fromPnt = {
      static_cast<int>(from.x / (m_info.mCellWidth * CELL_MULT)),
      static_cast<int>(from.y / (m_info.mCellHeight * CELL_MULT))};
  GridType::Point toPnt = {
      static_cast<int>(to.x / (m_info.mCellWidth * CELL_MULT)),
      static_cast<int>(to.y / (m_info.mCellHeight * CELL_MULT))};

  LOG_DEBUG("DistanceMap: getMoveDirection from "
            << from.x << "," << from.y << " (" << fromPnt.first << ","
            << fromPnt.second << ")"
            << " to " << to.x << "," << to.y << " (" << toPnt.first << ","
            << toPnt.second << ")");

  // Helper to compute precise angle to target point
  auto computePreciseAngle = [&](GridType::Point targetGridPt) -> float {
    double targetX, targetY;
    if (targetGridPt == toPnt) {
      // If moving to the final target tile, aim for the actual target position
      targetX = to.x;
      targetY = to.y;
    } else {
      // Otherwise aim for the center of the next tile
      targetX = (targetGridPt.first + 0.5) * (m_info.mCellWidth * CELL_MULT);
      targetY = (targetGridPt.second + 0.5) * (m_info.mCellHeight * CELL_MULT);
    }
    return computeAngle(targetX - from.x, targetY - from.y);
  };
  // FIX: Enforce a reuse limit if it is 0/Infinite
  if (ctx->reuseInit <= 0) ctx->reuseInit = 20;

  // Check if we can reuse previous direction (optimization)
  if (ctx->type == type) {
    auto dx = ctx->next.first - fromPnt.first;
    auto dy = ctx->next.second - fromPnt.second;

    // Check if we are still moving towards the next point (or it is adjacent)
    // Note: If we reached the next point (dx=0, dy=0), we fall through to
    // re-evaluate
    bool stillValid = (dx || dy);

    if (stillValid) {
      // Only allow reuse if the next point is adjacent
      bool isAdjacent = (std::abs(dx) <= 1 && std::abs(dy) <= 1);

      bool allowReuse = isAdjacent && (--ctx->reuseCnt > 0);

      if (allowReuse) {
        LOG_DEBUG("DistanceMap: Reusing direction (" << ctx->reuseCnt << " remaining)");
        ctx->didReuse = true;

        // Recompute angle based on precise position relative to the target cell
        // This allows smooth steering around corners/obs flow
        float nxtDir = computePreciseAngle(ctx->next);

        if (std::abs(nxtDir - ctx->curDir) > 0.1f) {
          LOG_DEBUG("DistanceMap: Direction updated: " << ctx->curDir << " => " << nxtDir);
          ctx->curDir = nxtDir;
        } else {
          LOG_DEBUG("DistanceMap: Direction unchanged: " << ctx->curDir);
        }
        return ctx->curDir;
      }
    }
  }

  // Update context
  ctx->from = fromPnt;
  ctx->to = toPnt;
  ctx->type = type;

  ctx->reuseCnt = ctx->reuseInit;

  // Get next move
  ctx->next = getNextMove(fromPnt, toPnt);

  // Compute angle (Precise)
  ctx->curDir = computePreciseAngle(ctx->next);

  LOG_DEBUG("DistanceMap: Next=" << ctx->next.first << "," << ctx->next.second
                                 << " Angle=" << ctx->curDir);

  return ctx->curDir;
}

}  // namespace Routing
}  // namespace DistanceMap
