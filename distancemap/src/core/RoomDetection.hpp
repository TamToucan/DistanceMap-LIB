#pragma once

/**
 * @file RoomDetection.hpp
 * @brief Detects "room" regions (wide open spaces) in a navigation graph.
 * @details Uses the wall-distance grid to find seed cells with large open
 * radius, grows regions from each seed, then merges regions sharing wide
 * boundary gates and discards regions smaller than a minimum area. Output is
 * a RoomMap of per-cell room ids plus a vector of RoomRegion summaries
 * (centre, area, dimensions, neighbour room ids). Corridors and walls are
 * left as ROOM_NONE.
 */

#include "GridTypes.hpp"
#include "WallDistanceGrid.hpp"
#include <vector>

namespace DistanceMap {

const int ROOM_NONE = -1; ///< sentinel: cell is unassigned (corridor or wall)

const int CORRIDOR_DIAM_TIGHT_MAX  = 2; ///< diameter <= 2 -> "tight" bucket
const int CORRIDOR_DIAM_NORMAL_MAX = 4; ///< diameter 3-4 -> "normal"; >4 -> "wide"

/**
 * @struct RoomParams
 * @brief Tuning thresholds for room detection.
 */
struct RoomParams {
    int minSeedDist  = 3;  ///< min wallDist to qualify as a room-center seed (≥7-tile open diameter)
    int minGateDist  = 2;  ///< wallDist threshold for a "wide" boundary cell pair
    int minGateWidth = 3;  ///< min wide-gate cell pairs needed to trigger a region merge
    int minArea      = 30; ///< min floor cells for a region to survive as a room
};

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
 * @brief Thickness profile of a corridor along its centerline.
 *
 * @details Fractions of the centerline length falling in each diameter bucket
 * (they sum to ~1.0), plus average/min/max diameter in tiles. Diameter at a
 * cell is 2*wallDist+1.
 */
struct CorridorThickness {
    float fracTight   = 0.0f; ///< fraction of length with diameter 1-2 tiles
    float fracNormal  = 0.0f; ///< diameter 3-4
    float fracWide    = 0.0f; ///< diameter 5+
    int   avgDiameter = 0;
    int   minDiameter = 0;
    int   maxDiameter = 0;
};

/**
 * @brief Records that a corridor crosses another at a skeleton junction.
 */
struct CorridorCrossing {
    int otherCorridorId;      ///< Corridor this one crosses
    GridType::Point junction; ///< Approx cell of the shared junction (a baseNode)
};

/**
 * @brief High-level summary of a single connection between two rooms.
 *
 * @details Abstract analogue of RoomRegion for the connective tissue. Holds the
 * rooms it joins, approximate length/wiggle/thickness, and any crossings with
 * other corridors. The raw centerline is kept only for debug/render.
 */
struct Corridor {
    int id;                                   ///< Compact corridor index (0-based)
    int roomA;                                ///< RoomRegion id at the start end
    int roomB;                                ///< RoomRegion id at the end end
    GridType::Point start{0, 0};              ///< Rough start cell (mouth at roomA)
    GridType::Point end{0, 0};                ///< Rough end cell (mouth at roomB)
    int length = 0;                           ///< Approx length in tiles (centerline cell count)
    float wiggle = 0.0f;                      ///< sum|turn angle| / length; 0 = straight
    CorridorThickness thickness;              ///< Thickness profile along the centerline
    std::vector<CorridorCrossing> crossings;  ///< Junctions shared with other corridors
    GridType::Path centerline;                ///< Skeleton cells (debug/render)
};

/**
 * @brief Per-cell room labels and region statistics for a level.
 *
 * @details
 * labels[y][x] is ROOM_NONE for walls and unassigned corridor cells,
 * or a room id in [0, rooms.size()) for confirmed room cells.
 */
struct RoomMap {
    std::vector<std::vector<int>> labels;         ///< [y][x]: ROOM_NONE or room id
    std::vector<std::vector<int>> corridorLabels; ///< [y][x]: ROOM_NONE or corridor id (centerline cells only)
    std::vector<RoomRegion> rooms;
    std::vector<Corridor> corridors;              ///< Abstract room-to-room connections
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
                    const WallDistanceGrid &wallDist,
                    const RoomParams &params = RoomParams{});

/**
 * @brief Populate roomMap.corridors from the thinned-skeleton graph.
 *
 * @details Run after makeRoomMap. Treats baseEdges whose cells are ROOM_NONE as
 * corridor space, enumerates room-pair connections through the skeleton, and
 * computes length/wiggle/thickness plus crossings at shared junction baseNodes.
 * See SPECIFICATION: specs/systems/corridor_detection.md.
 *
 * @param roomMap    RoomMap to populate (labels/rooms already computed).
 * @param baseNodes  Skeleton junction/branch cells (Graph::baseNodes).
 * @param baseEdges  Thinned-skeleton edges with full path lists (Graph::baseEdges).
 * @param wallDist   Pre-computed BFS wall-distance grid.
 */
void detectCorridors(RoomMap &roomMap,
                     const std::vector<GridType::Point> &baseNodes,
                     const std::vector<GridType::Edge> &baseEdges,
                     const WallDistanceGrid &wallDist);

} // namespace DistanceMap
