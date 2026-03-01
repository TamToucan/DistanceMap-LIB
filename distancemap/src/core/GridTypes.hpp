#ifndef DISTANCEMAP_SRC_GRIDTYPES_HPP_
#define DISTANCEMAP_SRC_GRIDTYPES_HPP_

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

struct PairHash {
  template <typename T1, typename T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const {
    return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
  }
};

using Grid = std::vector<std::vector<int>>;
using Point = std::pair<int, int>;
using Path = std::vector<Point>;

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
#if 0
const static std::vector<GridType::Point> directions8 = {
	{0, -1},  {0, 1},  {-1, 0}, {1, 0}, // 0 Up, 1 Down, 2 Left, 3 Right
	{-1, -1}, {1, -1}, {1, 1},  {-1, 1} // 4 LU, 5 RU, 6 RD, 7 LD
};

    // Reverses directions8 index to the opposite dir
const static std::vector<int> reverseDirIndex = { 1, 0, 3, 2, 6, 7, 4, 5 };

	// When going left->right, top->bottom, we only check 4 directions
    // Directions: RIGHT, BOTTOM, BOTTOM_RIGHT, BOTTOM_LEFT
const static std::vector<std::pair<int, int>> searchDirs4 = {
	{1, 0},   // Right
	{0, 1},   // Bottom
	{1, 1},   // Bottom-right
	{-1, 1},  // Bottom-left
};
const static std::vector<int> dir4todir8Index = {
	3, // Right => Right
	1, // Bottom => Down
	6, // Bottom-right => Down-Right
	7, // Bottom-left => Down-Left
};
#else
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
#endif

struct Edge {
  int from, to;        // Indices of connected nodes or deadEnds
  bool toDeadEnd;      // Whether `to` refers to a deadEnd
  GridType::Path path; // Path between the points
};

struct AbstractNode {
  std::vector<int> baseNodes; // Indices of nodes in the cluster
  GridType::Point center;     // Geometric center of the cluster
  int baseCenterNode;         // index of closest baseNode
};

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

struct BaseGraphInfo {
  int neighbor;  // Neighboring base node index.
  int edgeIndex; // Index into the edges vector.
  bool forward;  // True if neighbor is the TO and indexing by FROM.
  int cost;      // Cost is the length of the edge path.
};

//            |-----baseEdge-----|
// Graph of Base nodes -> Base Node and EdgeIdx and Cost
// The from->to and to->from is stored for each baseEdge
using BaseGraph = std::vector<std::vector<BaseGraphInfo>>;

struct GridPointInfo {
  int16_t closestAbstractNodeIdx = -1;
  int16_t distanceToAbstractNode = std::numeric_limits<short int>::max();
};

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
using BoundaryCellMap = std::unordered_map<int, BoundaryCells>;

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

std::vector<int> checkConnectivity(const BaseGraph &graph, int numBaseNodes);

BaseGraph buildBaseGraph(const std::vector<Edge> &edges, int numBaseNodes);

} // namespace GridType
} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_GRIDTYPES_HPP_ */
