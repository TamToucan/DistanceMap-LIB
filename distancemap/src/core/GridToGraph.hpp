#ifndef DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_
#define DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "DistanceMapDLL.hpp"
#include "FlowField.hpp"
#include "GridTypes.hpp"
#include "RoomDetection.hpp"
#include "SparseNavGraph.hpp"
#include "WallDistanceGrid.hpp"

//
// Take the 2D grid of EMPTY/PATH (i.e. 1 = floor, 0 = wall) and
// generate the thinned (1 pixel) 2D grid from that. This gives
// the sarting point to
// - work out all the "intersection" (.baseNodes)
// - find all the deadends (.deadEnds)
// - find the list of paths connecting nodes/deadends (.baseEdges)
// Then find the nodes and paths for the "main routes" to create
// an AbstractLevel
// - Repeat generating AbstractLevel, grouping increasing number
//   of baseNodes until small enough set of AbstractNodes
//  Each AbstractLevel has AbstractNodes which are connected
//  via AbstractEdges.
// An AbstractNode is a grouping of nearyby BaseNodes with the
// index of the .baseNodes for it's point.
// The AbstractEdges are the paths through the bottom level
// graph of baseNodes and baseEdges that connect nearby
// AbstractNodes
//
// So there is complex graph of baseNodes and baseEdges with
// each AbtractLevel being the simplified graph made by
// grouping baseNodes together into AbstractNodes and
// AbstraceEdges being the paths through the base graph
// between those nodes.
//
// Then for each AbstractLevel the 2D grid is split int
// a voronoi like set of ZoneInfo where a zone is a single
// AbstractNode.
// Each ZoneInfo has the list of all the baseNodes and baseEdges in
// the zone and the list of AbstractNode indexes of the adjacent zones.
// - Each zone also has the lis of cells the form the boundary with a
//   neighboring zone that it's actually connected to.
// The AbstractLevel also has
//
// ######################      4 Zones made for the 4 abstract nodes
// #            -       #      Zone0 adjacent to Zone1
// #     1      =       #      Zone1 adjacent to Zone0, Zone2
// #            -   2   #      Zone2 adjacent to Zone1, Zone3
// #--||---#-||--       #      Zone3 adjacent to Zone2
// #       #            #      The '-' are boundary cells
// #       ########-||--#
// #   0   #            #
// #       #    3       #
// #       #            #
// ######################
//
// For each Zone in a level (e.g. 3 above) I then create a SubGrid
// which has the min enclosing rectangle and it's position in the
// level's infoGrid.
// The subgrid.grid rectangle simply has WALL or 0 depending on whether it
// belongs to the zone or not (zones are any shape remember).
// Finally, and most importantly it creates a costFlowField for EACH neighbor
// zone than can be reached.
// So Zone3 subgrid would have 1 costFlowField since it is adjacent to Zone2
//
// The ZoneInfo has a map of adjZone -> vector of cell "islands" connect to that zone
// e.g. in the above
//  Zone1Map[0] -> 1 vector of the cells forming boundary with Zone0
//          [2] -> 2 vectors of the cells foming boundary with Zone2
//
//  Zone0Map[1] -> 1 vector of the cells for the "other side" of the boundary with zone1
//
//  Zone2Map[1] -> 2 vectors of the cells forming boundary with Zone1
//          [3] -> 1 vector of the cells forming boundary with Zone3
//
//  Zone3Map[2] -> 1 vector of the cells for the "other side" of the boundary with zone2
//
// There is also a zoneBridgeEdges which tells you which edges connect
// which zones and with what priority.

namespace DistanceMap {
namespace GridToGraph {

DISTANCEMAP_API std::vector<std::vector<int>>
gridToFloorGrid(const std::vector<std::vector<int>> &grid);
DISTANCEMAP_API std::vector<std::vector<int>>
readGridFromFile(const std::string &filename);

using namespace GridType;

// For INPUT GRID
const int EMPTY = 0x00;
const int PATH = 0x01; // NOTE: Must be 1 for dead end detection

// Describes an edge connecting two adjacent zones for the zoneBridgeEdges
// in the abstract level.
// - zoneFrom/zoneTo is the indices of the adjacent zones
// - islandSlot is the index of the island in the zone's cell boundary for zones
// - bridgePriority is the priority of the bridge. 0 = low, higher = more nodes
// are unreachable if that boundary island is removed
// 
struct ZoneBridgeEdge {
  int zoneFrom;
  int zoneTo;
  int islandSlot;
  int bridgePriority;
};

// An AbstractLevel takes the base nodes and groups them into
// clusters of size based on the level number. This creates
// it's own craft of abstract nodes and edges and a grid of
// ZonePointInfo.
// Each abstract node is then turned into a ZoneInfo with
// info on the base nodes/edges in that zone and the adjacent zones.
//
struct AbstractLevel {
  std::vector<AbstractEdge> abstractEdges;
  std::vector<AbstractNode> abstractNodes;
  GridType::ZoneGrid zoneGrid;
  std::vector<FlowField::SubGrid> subGrids;
  std::vector<GridType::ZoneInfo> zones;
  std::vector<ZoneBridgeEdge> zoneBridgeEdges; // Priority high -> low
};

// Map of ALL BaseFromIdx,BaseToIdx pairs returning the total length of path
// connecting node pair. i.e. the total length of all the paths to get from
// the first node to the second node.
using PathCostMap = std::unordered_map<std::pair<int, int>, int, PairHash>;

struct Graph {
  GridType::Grid infoGrid;
  BaseGraph baseGraph;
  PathCostMap pathCostMap;
  Routing::SparseNavGraph routingGraph;
  std::vector<Edge> baseEdges;
  std::vector<GridType::Point> baseNodes;
  std::vector<GridType::Point> deadEnds;
  std::vector<AbstractLevel> abstractLevels;
  WallDistanceGrid wallDistanceGrid;
  SightGrid sightGrid;
  RoomMap roomMap;
};

////////////////////////////////////////////////////////////////////////////////
//
// Floor must = PATH on input i.e. walkable, WALLS = EMPTY
//
DISTANCEMAP_API Graph makeGraph(const GridType::Grid &floorGrid);

} // namespace GridToGraph
} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_GRIDTOGRAPH_HPP_ */
