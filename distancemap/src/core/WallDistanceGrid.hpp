#ifndef DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_
#define DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_

#include <vector>
#include "GridTypes.hpp"

namespace DistanceMap {

//
// Function to compute the distance grid
// Input grid is non-0 = wall
// Distance grid is a new grid the size of the input which has
// distance to the nearest wall (So 0 if it is a wall)
//
std::vector<std::vector<int>> makeWallDistanceGrid(const GridType::Grid& grid);

// A grid where each entry is a packed dist can see before Wall
struct SightGrid {
	GridType::Grid sight;
	int getNorth(int x, int y) const { return sight[y][x] & 0xff; }
	int getEast (int x, int y) const { return (sight[y][x] & 0xff00) >> 8; }
	int getSouth(int x, int y) const { return (sight[y][x] & 0xff0000) >> 16; }
	int getWest (int x, int y) const { return static_cast<unsigned int>(sight[y][x] & 0xff000000) >> 24; }
};


SightGrid makeSightGrid(const GridType::Grid& grid);

}



#endif /* DISTANCEMAP_SRC_WALLDISTANCEGRID_HPP_ */
