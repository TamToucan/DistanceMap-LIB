#pragma once

#include "GridTypes.hpp"
#include "WallDistanceGrid.hpp"
#include <vector>

namespace DistanceMap {

const int MIN_SEED_DIST  = 3;  ///< min wallDist to qualify as a room-center seed (≥7-tile open diameter)
const int MIN_GATE_DIST  = 2;  ///< wallDist threshold for a "wide" boundary cell pair
const int MIN_GATE_WIDTH = 3;  ///< min wide-gate cell pairs needed to trigger a region merge
const int MIN_AREA       = 50; ///< min floor cells for a region to survive as a room
const int ROOM_NONE      = -1; ///< sentinel: cell is unassigned (corridor or wall)

/**
 * @brief Statistics for a single detected room region.
 */
struct RoomRegion {
    int id;                           ///< Compact room index (0-based)
    int area;                         ///< Floor cell count
    GridType::Point center;           ///< Cell with the highest wallDist in this region
    int maxWallDist;                  ///< wallDist at center — room "radius" in cells
    int approxWidth;                  ///< Bounding-box width in cells
    int approxHeight;                 ///< Bounding-box height in cells
    std::vector<int> neighborRoomIds; ///< Ids of adjacent rooms after merge/discard
};

/**
 * @brief Per-cell room labels and region statistics for a level.
 *
 * @details
 * labels[y][x] is ROOM_NONE for walls and unassigned corridor cells,
 * or a room id in [0, rooms.size()) for confirmed room cells.
 */
struct RoomMap {
    std::vector<std::vector<int>> labels; ///< [y][x]: ROOM_NONE or room id
    std::vector<RoomRegion> rooms;
    int width  = 0;
    int height = 0;
};

/**
 * @brief Compute room labels for all FLOOR cells in an info grid.
 *
 * @param infoGrid  The completed infoGrid from GridToGraph::Graph.
 * @param wallDist  Pre-computed BFS wall-distance grid (dist==0 means wall).
 * @return          RoomMap with labels and per-room statistics.
 */
RoomMap makeRoomMap(const GridType::Grid &infoGrid,
                    const WallDistanceGrid &wallDist);

} // namespace DistanceMap
