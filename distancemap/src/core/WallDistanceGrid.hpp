#ifndef DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_
#define DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_

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
// Input grid is non-0 = wall
WallDistanceGrid makeWallDistanceGrid(const GridType::Grid& grid);

//
// A grid where each entry is a packed dist can see before Wall
//
struct SightGrid {
	GridType::Grid sight;
	int getNorth(int x, int y) const { return sight[y][x] & 0xff; }
	int getEast (int x, int y) const { return (sight[y][x] & 0xff00) >> 8; }
	int getSouth(int x, int y) const { return (sight[y][x] & 0xff0000) >> 16; }
	int getWest (int x, int y) const { return static_cast<unsigned int>(sight[y][x] & 0xff000000) >> 24; }
    int getFurthest(int x, int y) const {
        return std::max({getNorth(x, y), getEast(x, y), getSouth(x, y), getWest(x, y)});
    }
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
// Input grid is non-0 = wall
SightGrid makeSightGrid(const GridType::Grid& grid);

} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_ */
