#ifndef DISTANCE_MAP_NAVIGATOR_HPP
#define DISTANCE_MAP_NAVIGATOR_HPP

#include <limits>
#include <vector>

#include "DistanceMapDLL.hpp"
#include "GridTypes.hpp"
#include "NavigationAPI.hpp"
#include "Router.hpp"

namespace DistanceMap {
namespace Routing {

/**
 * NavigationFlowGrid - A simpler, more performant alternative to
 * NavigationGraph
 *
 * Uses a distance map (flow field) approach for navigation:
 * - When target changes: Compute distance map once using Dijkstra's algorithm
 * - For queries: O(1) lookup of pre-computed direction
 *
 * Performance: Optimized for scenarios with many agents (100s-1000s) moving
 * toward the same or slowly-changing target. Distance map computation is
 * ~O(W×H) but only happens when target changes.
 */
class DISTANCEMAP_API NavigationFlowGrid : public NavigationAPI {
 public:
  NavigationFlowGrid();
  ~NavigationFlowGrid() = default;

  NavigationFlowGrid(const GridType::Grid& infoGrid, const Router::Info& info);

  /**
   * Main entry point for movement - compatible with NavigationGraph API
   *
   * @param ctx Route context for caching and reuse logic
   * @param from Source position in world coordinates
   * @param to Target position in world coordinates
   * @param type Movement type identifier for context switching
   * @return Angle in degrees [0, 360) to move toward target
   */
  float getMoveDirection(Router::RouteCtx* ctx, GridType::Vec2 from,
                         GridType::Vec2 to, int type) override;

 private:
  /**
   * Cell data in the distance map
   */
  struct Cell {
    uint16_t distance;  // Distance to target in cells (65535 =
                        // unreachable/uncomputed)
    uint8_t direction;  // Direction index (0-7) to move toward target, or
                        // SINK/NO_DIR

    Cell()
        : distance(std::numeric_limits<uint16_t>::max()), direction(NO_DIR) {}
  };

  // Special direction values
  static constexpr uint8_t NO_DIR = 255;  // No direction computed
  static constexpr uint8_t SINK = 254;    // This cell is the target

  // Distance map cache
  std::vector<std::vector<Cell>> m_distanceMap;
  GridType::Point m_cachedTarget;
  bool m_cacheValid;

  /**
   * Compute distance map from target using Dijkstra's algorithm
   * Updates m_distanceMap with distance and direction for each cell
   */
  void computeDistanceMap(GridType::Point target);

  /**
   * Get next move from source toward target
   * Handles cache validation, wall/boundary cases
   */
  GridType::Point getNextMove(GridType::Point source, GridType::Point target);

  /**
   * Compute angle from direction vector
   */
  float computeAngle(double dx, double dy);

  /**
   * Get next point by moving in a direction
   */
  GridType::Point nextPoint(const GridType::Point& from,
                            const GridType::Point& dir);
};

}  // namespace Routing
}  // namespace DistanceMap

#endif  // DISTANCE_MAP_NAVIGATOR_HPP
