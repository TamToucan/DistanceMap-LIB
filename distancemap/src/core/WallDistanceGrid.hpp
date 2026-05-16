#ifndef DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_
#define DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_

/**
 * @file WallDistanceGrid.hpp
 * @brief Wall-distance and 4-direction sight grids derived from an input grid.
 * @details WallDistanceGrid: BFS from every wall; each cell stores its 8-byte
 * capped distance to the nearest wall. Useful for room-centre detection and
 * "open space" heuristics. SightGrid: per cell, the open distance in each
 * cardinal direction packed into one int; useful for visibility/line-of-sight
 * approximations.
 */

#include <cstdint>
#include <algorithm>
#include <limits>
#include <vector>
#include "GridTypes.hpp"

namespace DistanceMap {

//
// Flat, row-major grid of uint8_t wall-distance values.
// Each cell stores the BFS distance to the nearest wall, capped at 255.
// Value 0 means the cell itself is a wall.
//
/**
 * @struct WallDistanceGrid
 * @brief Row-major grid of BFS distance-to-nearest-wall (uint8 capped at 255).
 */
struct WallDistanceGrid {
    std::vector<uint8_t> data; ///< Row-major storage: data[y * width + x]
    int width  = 0;
    int height = 0;

    /// Returns the wall-distance at (x, y). Caller must ensure valid(x, y).
    uint8_t get(int x, int y) const {
        return data[static_cast<size_t>(y) * width + x];
    }

    /// Returns true if (x, y) is within grid bounds.
    bool valid(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }
};
/// Build a WallDistanceGrid from an input grid where non-zero = wall.
WallDistanceGrid makeWallDistanceGrid(const GridType::Grid& grid);

//
// A grid where each entry is a packed dist can see before Wall
//
/**
 * @struct SightGrid
 * @brief Per-cell open-distance in 4 cardinal directions, packed into one int.
 * @details Bits 0-7 = North open distance, 8-15 = East, 16-23 = South, 24-31 = West.
 * Open distance is the number of empty cells before hitting a wall.
 */
struct SightGrid {
	GridType::Grid sight;
	/// Open distance northward from (x,y) until a wall.
	int getNorth(int x, int y) const { return sight[y][x] & 0xff; }
	/// Open distance eastward from (x,y) until a wall.
	int getEast (int x, int y) const { return (sight[y][x] & 0xff00) >> 8; }
	/// Open distance southward from (x,y) until a wall.
	int getSouth(int x, int y) const { return (sight[y][x] & 0xff0000) >> 16; }
	/// Open distance westward from (x,y) until a wall.
	int getWest (int x, int y) const { return static_cast<unsigned int>(sight[y][x] & 0xff000000) >> 24; }
	/// Returns the largest open distance among N/E/S/W at (x,y).
    int getFurthest(int x, int y) const {
        return std::max({getNorth(x, y), getEast(x, y), getSouth(x, y), getWest(x, y)});
    }
	/// Returns the smallest non-zero open distance among N/E/S/W at (x,y).
    int getNearest(int x, int y) const {
        int n = getNorth(x,y);
        int e = getEast(x,y);
        int s = getSouth(x,y);
        int w = getWest(x,y);
        n = n ? n : std::numeric_limits<int>::max();
        e = e ? e : std::numeric_limits<int>::max();
        s = s ? s : std::numeric_limits<int>::max();
        w = w ? w : std::numeric_limits<int>::max();
        return std::min({n, e, s, w});
    }
};
/// Build a SightGrid from an input grid where non-zero = wall.
SightGrid makeSightGrid(const GridType::Grid& grid);

} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_ */
