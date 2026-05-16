#ifndef DISTANCEMAP_SRC_GRIDTYPES_HPP_
#define DISTANCEMAP_SRC_GRIDTYPES_HPP_

/**
 * @file GridTypes.hpp
 * @brief Common grid types and packed-cell encodings used across DistanceMap.
 * @details Defines:
 *   - Primitive aliases: Grid, Point, Path, Vec2.
 *   - infoGrid bit flags (NODE/DEND/EDGE/XPND/BOUNDARY/WALL) and the
 *     accessor helpers for the packed fields.
 *   - 8-neighbour direction tables (directions8, reverseDirIndex,
 *     searchDirs4, dir4todir8Index).
 *   - Graph data structures: Edge, AbstractNode, AbstractEdge,
 *     BaseGraphInfo, BaseGraph.
 *   - Per-cell ZoneGrid (GridPointInfo) and per-zone metadata (ZoneInfo,
 *     BoundaryInfo, BoundaryCells).
 */

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace DistanceMap {
namespace GridType {

// InfoGrid has int's with bottom word set to an index
const int NODE = 0x010000; // baseNode index
const int DEND = 0x020000; // baseNode index
const int EDGE = 0x040000; // <0|1 2nd Half of path> <15bits baseEdge index>
const int XPND = 0x080000; // <3bit directions8 index to Edge> | <13 edge dist>
const int BOUNDARY = 0x400000; // <3bit directions8 index to non-WALL
const int WALL = 0x800000;     // 0

const int EDGE_MASK = 0x7fff;
const int EDGE_HALF = 0x8000;

const int DIR_MASK = 0x7;
const int XPND_DIR_SHIFT = 13;
inline int get_XPND_DIST(int cell) {
  return cell & ((1 << XPND_DIR_SHIFT) - 1);
}
inline int get_XPND_DIR(int cell) {
  return ((cell & 0xffff) >> XPND_DIR_SHIFT);
}

const int WALL_DIST_SHIFT = 3;
inline int get_WALL_DIR(int cell)  { return cell & DIR_MASK; }
inline int get_WALL_DIST(int cell) { return (cell >> WALL_DIST_SHIFT) & 0x1FFF; }

/// Hash functor for std::pair — used as key in unordered_map of cell pairs.
struct PairHash {
  template <typename T1, typename T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const {
    return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
  }
};

/// 2D row-major integer grid: grid[y][x].
using Grid = std::vector<std::vector<int>>;
/// 2D integer cell position (x, y).
using Point = std::pair<int, int>;
/// Ordered list of cells along a path.
using Path = std::vector<Point>;

/// Simple 2D float vector (positions/directions in world space).
struct Vec2 {
  float x, y;
  Vec2() : x(0), y(0) {}
  Vec2(float _x, float _y) : x(_x), y(_y) {}
  bool operator==(const Vec2 &other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const Vec2 &other) const { return !(*this == other); }
};

// Directions for 8 neighbouring cells
const static std::vector<GridType::Point> directions8 = {
    {1, 0},   // 0 Right
    {1, 1},   // 1 RD
    {0, 1},   // 2 Down
    {-1, 1},  // 3 LD
    {-1, 0},  // 4 Left
    {-1, -1}, // 5 LU
    {0, -1},  // 6 Up
    {1, -1},  // 7 RU
};

// Reverses directions8 index to the opposite dir
const static std::vector<int> reverseDirIndex = {4, 5, 6, 7, 0, 1, 2, 3};

// When going left->right, top->bottom, we only check 4 directions
// Directions: RIGHT, BOTTOM, BOTTOM_RIGHT, BOTTOM_LEFT
const static std::vector<std::pair<int, int>> searchDirs4 = {
    {1, 0},  // Right
    {0, 1},  // Bottom
    {1, 1},  // Bottom-right
    {-1, 1}, // Bottom-left
};
const static std::vector<int> dir4todir8Index = {
    0, // Right => Right
    2, // Bottom => Down
    1, // Bottom-right => Down-Right
    3, // Bottom-left => Down-Left
};

/// Base-graph edge: connects two baseNodes (or a baseNode and a deadEnd) and
/// carries the full thinned-skeleton path between them.
struct Edge {
  int from, to;        // Indices of connected nodes or deadEnds
  bool toDeadEnd;      // Whether `to` refers to a deadEnd
  GridType::Path path; // Path between the points
};

/// Cluster of nearby baseNodes treated as one node in an AbstractLevel.
struct AbstractNode {
  std::vector<int> baseNodes;   // Indices of nodes in the cluster
  GridType::Point center{0, 0}; // Geometric center (+= used so init to 0)
  int baseCenterNode{-1};       // index of closest baseNode
};

/// Edge between two AbstractNodes plus the underlying base-graph path that
/// realises it (path is cell-by-cell; nodePath is the baseNode sequence).
struct AbstractEdge {
  int from, to; // Indices of connected abstract nodes
  GridType::Path path;
  std::vector<int> nodePath;

  bool operator<(const AbstractEdge &other) const {
    if (from != other.from)
      return from < other.from;
    return to < other.to;
  }
};

/// One adjacency record in the BaseGraph adjacency list.
struct BaseGraphInfo {
  int neighbor;  // Neighboring base node index.
  int edgeIndex; // Index into the edges vector.
  bool forward;  // True if neighbor is the TO and indexing by FROM.
  int cost;      // Cost is the length of the edge path.
};

//            |-----baseEdge-----|
// Graph of Base nodes -> Base Node and EdgeIdx and Cost
// The from->to and to->from is stored for each baseEdge
// It excludes dead ends
using BaseGraph = std::vector<std::vector<BaseGraphInfo>>;

/// Per-cell info stored in a ZoneGrid: which AbstractNode this cell belongs
/// to (Voronoi-style assignment) and its BFS distance to that node's centre.
struct GridPointInfo {
  int16_t closestAbstractNodeIdx = -1;
  int16_t distanceToAbstractNode = std::numeric_limits<short int>::max();
};

/// Cell on a zone boundary plus the direction (directions8 index) that
/// crosses into the neighbouring zone.
struct BoundaryInfo {
  GridType::Point sink;
  int exitDirIdx = -1;

  // Constructor to allow brace-enclosed initialization
  BoundaryInfo(const GridType::Point &s, int d) : sink(s), exitDirIdx(d) {}
  BoundaryInfo(int x, int y, int d) : sink(x, y), exitDirIdx(d) {}

  bool operator==(const BoundaryInfo &other) const {
    return (sink == other.sink) && (exitDirIdx == other.exitDirIdx);
  }
};

struct BoundaryInfoHash {
  std::size_t operator()(const BoundaryInfo &b) const {
    return std::hash<int>()(b.sink.first) ^
           ((std::hash<int>()(b.sink.second) << 1) ^ b.exitDirIdx);
  }
};

//
// When make the boundaies I just want unique points and dont care which dir is
// used
//
struct BoundaryInfoSinkComparator {
  bool operator()(const BoundaryInfo &lhs, const BoundaryInfo &rhs) const {
    // Only compare the sink (x, y) for set uniqueness
    return lhs.sink == rhs.sink;
  }
};

struct BoundaryInfoSinkHash {
  std::size_t operator()(const BoundaryInfo &b) const {
    // Only hash the sink (x, y) for set storage
    return std::hash<int>()(b.sink.first) ^
           (std::hash<int>()(b.sink.second) << 1);
  }
};

// Adjacent Zone Index => unique set of of boundary cells
using BoundaryCells = std::unordered_set<BoundaryInfo, BoundaryInfoSinkHash,
                                         BoundaryInfoSinkComparator>;
// Adjacent Zone Index => vector of BoundaryCells, one slot per base edge
// crossing into that neighbor zone (segments parallel crossings).
using BoundaryCellMap = std::unordered_map<int, std::vector<BoundaryCells>>;

/// Aggregated info for a single zone: its base graph slice and its connections
/// to neighbouring zones (via boundary cells).
struct ZoneInfo {
  std::vector<int>
      baseNodeIdxs; // List of all the indexes of the base nodes in this zone
  std::vector<int>
      baseEdgeIdxs; // List of all the indexes of the base edges in this zone
  std::vector<int>
      adjacentZones; // List of all the indexes of the adjacent zones
  BoundaryCellMap zoneBoundaryCellMap; // List of boundary cells for each
                                       // neighbor zone can reach

  // O(1) membership sets – populated at zone build time alongside the vectors
  // above
  std::unordered_set<int> baseNodeIdxSet;
  std::unordered_set<int> baseEdgeIdxSet;
};

// 2D grid of information for each cell to aid navigation
using ZoneGrid = std::vector<std::vector<GridPointInfo>>;

/// Run BFS over the base graph and return a per-baseNode component id.
/// Used to detect disconnected components in a level.
std::vector<int> checkConnectivity(const BaseGraph &graph, int numBaseNodes);

/// Build the BaseGraph adjacency list from the flat Edge list.
/// Records both directions of each edge (forward = true for FROM->TO).
BaseGraph buildBaseGraph(const std::vector<Edge> &edges, int numBaseNodes);

} // namespace GridType
} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_GRIDTYPES_HPP_ */
