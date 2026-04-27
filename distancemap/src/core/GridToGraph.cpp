#include "GridToGraph.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <limits>
#include <queue>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "AbstractMST.hpp"
#include "Debug.h"
#include "GridTypes.hpp"
#include "TGA.hpp"
#include "WallDistanceGrid.hpp"
#include "ZSThinning.hpp"

#ifdef RELEASE_BUILD
#define NO_DIST_GRAPH_DEBUG
#endif

namespace DistanceMap {

using namespace GridType;

namespace {

// Helper function to check if a point is within bounds
inline bool isInBounds(int x, int y, int rows, int cols) {
  return x >= 0 && y >= 0 && x < cols && y < rows;
}

using Pattern = std::vector<std::vector<int>>;

// Define all patterns for nodes(middle point is the node)
//
const std::vector<Pattern> &getBasePatterns() {
  static std::vector<Pattern> patterns;

  if (patterns.size() != 0) {
    return patterns;
  }
  // 4-Way Junction (+-Junction)
  // 			-#-
  //			###
  //			-#-
  patterns.push_back({
      {0, 1, 0},
      {1, 1, 1},
      {0, 1, 0},
  });

  // 4-Way Junction (X-Junction)
  // 			#-#
  //			-#-
  //			#-#
  patterns.push_back({
      {1, 0, 1},
      {0, 1, 0},
      {1, 0, 1},
  });
  // 1-Way Junction (X-Junction)
  // 			#-#
  //			-#-
  //			#--
  patterns.push_back({
      {1, 0, 1},
      {0, 1, 0},
      {1, 0, 0},
  });
  // 1-Way Junction (X-Junction) 1 filled
  // 			#-#
  //			##-
  //			#--
  patterns.push_back({
      {1, 0, 1},
      {1, 1, 0},
      {1, 0, 0},
  });
  // T-Junction
  // 			-#-
  //			###
  //			---
  patterns.push_back({
      {0, 1, 0},
      {1, 1, 1},
      {0, 0, 0},
  });
  // T-junc with 1 diag
  // 			-#-
  //			-##
  //			#--
  patterns.push_back({
      {0, 1, 0},
      {0, 1, 1},
      {1, 0, 0},
  });
  // T-junc with 1 diag flipped
  // 			#--
  //			-##
  //			-##
  patterns.push_back({
      {1, 0, 0},
      {0, 1, 1},
      {0, 1, 1},
  });
  // T-junc with 1 diag flipped2
  // 			#--
  //			-##
  //			###
  patterns.push_back({
      {1, 0, 0},
      {0, 1, 1},
      {1, 1, 1},
  });

  // 3-way with overlap (1 unset)
  // 			--#
  //			###
  //			-#-
  patterns.push_back({
      {0, 0, 1},
      {1, 1, 1},
      {0, 1, 0},
  });
  // 3-way with overlap
  // 			-##
  //			###
  //			--#
  patterns.push_back({
      {0, 1, 1},
      {1, 1, 1},
      {0, 0, 1},
  });

  // T-jun 2 diag
  // 			-#-
  // 			-#-
  //			#-#
  patterns.push_back({
      {0, 1, 0},
      {0, 1, 0},
      {1, 0, 1},
  });

  // T-jun 2 diag (top set)
  // 			-##
  // 			-#-
  //			#-#
  patterns.push_back({
      {0, 1, 1},
      {0, 1, 0},
      {1, 0, 1},
  });

  // T-jun 2 diag (mid set)
  // 			-#-
  // 			-##
  //			#-#
  patterns.push_back({
      {0, 1, 0},
      {0, 1, 1},
      {1, 0, 1},
  });

  // 2 diag unset
  // 			#-#
  // 			-##
  //			###
  patterns.push_back({
      {1, 0, 1},
      {0, 1, 1},
      {1, 1, 1},
  });
  // 2 diag unset num2
  // 			#-#
  // 			-##
  //			##-
  patterns.push_back({
      {1, 0, 1},
      {0, 1, 1},
      {1, 1, 0},
  });

  // 2 adjacent unset
  //			###
  // 			-##
  // 			-##
  patterns.push_back({
      {1, 1, 1},
      {0, 1, 1},
      {0, 1, 1},
  });

  // 1 unset
  //			###
  // 			##-
  // 			###
  patterns.push_back({
      {1, 1, 1},
      {1, 1, 0},
      {1, 1, 1},
  });

  // T-junc with (bottom set)
  //			-#-
  // 			###
  // 			#--
  patterns.push_back({
      {0, 1, 0},
      {1, 1, 1},
      {1, 0, 0},
  });

  // T-junc with (both bottom set)
  //			-#-
  // 			###
  // 			#-#
  patterns.push_back({
      {0, 1, 0},
      {1, 1, 1},
      {1, 0, 1},
  });
  return patterns;
}

// Mirror the 3x3 pattern
//
Pattern mirrorPattern(const Pattern &pattern) {
  Pattern mirrored(3, std::vector<int>(3, 0));
  for (int i = 0; i < 3; ++i) {
    mirrored[i][0] = pattern[i][2];
    mirrored[i][1] = pattern[i][1];
    mirrored[i][2] = pattern[i][0];
  }
  return mirrored;
}

// Rotate the 3x3 pattern
//
Pattern rotatePattern(const Pattern &pattern) {
  Pattern rotated(3, std::vector<int>(3, 0));
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      rotated[j][(3 - 1) - i] = pattern[i][j]; // Rotate clockwise
    }
  }
  return rotated;
}

int makeID(const Pattern &pattern) {
  int id = 0;
  int bit = 0x01;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (pattern[i][j] == GridToGraph::PATH) {
        id |= bit;
      }
      bit <<= 1;
    }
  }
  return id;
}

// Use the 9 (3x3) bits (bit0 = top left) to created pattern ID
// NOTE: This is called with the Grid and Patterns
//
int getPatternID(const Grid &grid, int x, int y) {
  int id = 0;
  int bit = 0x01;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      if (grid[y + i][x + j] == GridToGraph::PATH) {
        id |= bit;
      }
      bit <<= 1;
    }
  }
  return id;
}

// Add the pattern to the list of IDs if not already present
// (some patterns are symmetrical so rotated and/or mirrored
// wont need added since they generate same ID since same pattern
//
bool addPatternID(const Pattern &p, std::vector<int> &ids) {
  int id = makeID(p);
  // Return true if first time ever stored id
  bool ret = (ids[id] == 0);
  ids[id] = 1;
  return ret;
}

//
// Take the base patterns and rotation them 0,90,180,270 degrees
// also mirror then rotate
// The returned array contains all bit patterbs (using the 9 (3x3) bits)
// of patterns that the middle of 3x3 is a node
//
const std::vector<int> &getAllPatterns() {
  static std::vector<Pattern> allPatterns;
  static std::vector<int> patternIDs(1 << 3 * 3, 0);
  static bool madePatterns = false;

  if (!madePatterns) {
    for (bool mirror : {false, true}) {
      for (const auto &pattern : getBasePatterns()) {
        Pattern current = mirror ? mirrorPattern(pattern) : pattern;
        for (int r = 0; r < 3; ++r) {
          if (addPatternID(current, patternIDs)) {
            allPatterns.push_back(current);
          }
          current = rotatePattern(current);
        }
        if (addPatternID(current, patternIDs)) {
          allPatterns.push_back(current);
        }
      }
    }
    madePatterns = true;
  }

  return patternIDs;
}

// The is only used by Deadend detection and only will only return 1 if
// all EMPTY and 1 PATH since PATH==1 and EMPTY==0 and all others > 1.
//
inline int countNeighbors(const Grid &grid, int x, int y) {
  return grid[y - 1][x - 1] + grid[y - 1][x] + grid[y - 1][x + 1]

         + grid[y][x - 1] + grid[y][x + 1]

         + grid[y + 1][x - 1] + grid[y + 1][x] + grid[y + 1][x + 1];
}

} // namespace

////////////////////////////////////////////////////////////////////////////////////////

namespace GridToGraph {

inline bool isNode(int cellValue) {
  // Is NODE or DEND bit set
  return (cellValue & (NODE | DEND)) != 0;
}

inline bool isPath(int cellValue) {
  // It's not empty and not a node
  // (assuming only one bit is set for PATH, or that PATH is bit 1)
  return cellValue && !isNode(cellValue);
}

#ifdef NO_DIST_GRAPH_DEBUG
void writeFloorGridToFile(const std::vector<std::vector<int>> &grid,
                          const std::string &filename) {}
void debugDump(const Graph &graph) {}
void debugGridEdges(const Graph &graphs) {}
void debugAbstractNodes(int pass, const AbstractLevel &ablv,
                        const Graph &graph) {}
void debugAbstractEdges(int pass, const AbstractLevel &ablv, const Graph &graph,
                        const char *fname) {}
void debugZones(int pass, const AbstractLevel &ablv, const Graph &graph) {}
void debugZoneEdges(int pass, const AbstractLevel &ablv, const Graph &graph) {}
void debugZoneBoundaries(int pass, const AbstractLevel &ablv) {}
void debugExpandedPaths(const Grid &grid) {}
#else
void writeFloorGridToFile(const std::vector<std::vector<int>> &grid,
                          const std::string &filename);
void debugDump(const Graph &graph);
void debugGridEdges(const Graph &graphs);
void debugAbstractNodes(int pass, const AbstractLevel &ablv,
                        const Graph &graph);
void debugAbstractEdges(int pass, const AbstractLevel &ablv, const Graph &graph,
                        const char *fname);
void debugZones(int pass, const AbstractLevel &ablv, const Graph &graph);
void debugZoneEdges(int pass, const AbstractLevel &ablv, const Graph &graph);
void debugZoneBoundaries(int pass, const AbstractLevel &ablv);
void debugExpandedPaths(const Grid &grid);
#endif

void extraThining(Grid &grid) {
  // Idea was to change non-thinned
  // ###
  // # #
  // ###
  // by taking corners, but crashed and cant be bothered
#if 0
    Pattern thinPatten = {
        {1, 1, 1 },
        {1, 0, 1 },
        {1, 1, 1 },
    };
    const int extraId = makeID(thinPatten);

    for (int y = 1; y < grid.size() - 3; ++y) {
    	for (int x = 1; x < grid[0].size() - 3; ++x) {
    		const int id = getPatternID(grid, x, y);
    		if (id == extraId) {
    			//if (!grid[y-1][x-1]) grid[y+0][x+0] = EMPTY; // TL
    			//if (!grid[y-1][x+3]) grid[y+0][x+2] = EMPTY; // TR
    			//if (!grid[y+3][x-1]) grid[y+2][x+0] = EMPTY; // BL
    			//if (!grid[y+3][x+3]) grid[y+2][x+2] = EMPTY; // BR
    		}
    	}
    }
#endif
}
///////////////////////////////////////////////////////////

std::vector<Point> detectDeadEnds(const Grid &grid) {
  std::vector<std::pair<int, int>> deadEnds;

  for (int y = 1; y < grid.size() - 1; ++y) {
    for (int x = 1; x < grid[0].size() - 1; ++x) {
      // NOTE: This relies on PATH==1
      if (grid[y][x] && countNeighbors(grid, x, y) == 1) {
        deadEnds.push_back({x, y});
      }
    }
  }

  return deadEnds;
}

std::vector<Point> detectNodes(const Grid &grid) {
  std::vector<std::pair<int, int>> nodes;
  const auto &patternIDS = getAllPatterns();
  for (int y = 0; y < grid.size() - 2; ++y) {
    for (int x = 0; x < grid[0].size() - 2; ++x) {
      // Check that the middle of the 3x3 to be checked is set
      // (could skip this since all patterns have middle bit set)
      //
      if (grid[y + 1][x + 1]) {
        // Get the ID from the 9 (3x3) bits and check if a pattern exists
        int id = getPatternID(grid, x, y);
        if (patternIDS[id]) {
          // Pattern exists (so it was a match) => store the Node
          nodes.push_back({x + 1, y + 1});
        }
      }
    }
  }

  return nodes;
}

///////////////////////////////////////////////////////////

namespace {
#ifndef NO_DIST_GRAPH_DEBUG
void makeTGA2(const char *name, const Grid &grid, unsigned int mask) {
  unsigned char *pPixel = new unsigned char[grid.size() * grid[0].size() * 4];
  unsigned char *pData = pPixel;
  for (int y = grid.size() - 1; y >= 0; --y) {
    int x = 0;
    for (int xy : grid[y]) {
      xy &= mask;
      switch (xy) {
      case GridToGraph::NODE:
        *pData++ = 0xff;
        *pData++ = 0xff;
        *pData++ = 0x00;
        *pData++ = 0xff;
        break;
      case GridToGraph::DEND:
        *pData++ = 0xff;
        *pData++ = 0x00;
        *pData++ = 0x00;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 1):
        *pData++ = 0x00;
        *pData++ = 0xff;
        *pData++ = 0xff;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 2):
        *pData++ = 0xff;
        *pData++ = 0x00;
        *pData++ = 0xff;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 3):
        *pData++ = 0x7f;
        *pData++ = 0x00;
        *pData++ = 0xff;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 2 | (XPND << 1)):
        *pData++ = 0x00;
        *pData++ = 0x7f;
        *pData++ = 0xff;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 3 | (XPND << 1)):
        *pData++ = 0x7f;
        *pData++ = 0x00;
        *pData++ = 0x7f;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 3 | (XPND << 2)):
        *pData++ = 0x7f;
        *pData++ = 0x00;
        *pData++ = 0x00;
        *pData++ = 0xff;
        break;
      case (GridToGraph::XPND << 3 | (XPND << 2) | (XPND << 1)):
        *pData++ = 0x7f;
        *pData++ = 0x7f;
        *pData++ = 0x7f;
        *pData++ = 0xff;
        break;
      case 0xff:
        *pData++ = 0x00;
        *pData++ = 0x00;
        *pData++ = 0x00;
        *pData++ = 0x00;
        break;
      case GridToGraph::WALL:
        *pData++ = 0x00;
        *pData++ = 0x6f;
        *pData++ = 0x00;
        *pData++ = 0xff;
        break;
      case GridToGraph::XPND:
        *pData++ = 0x4f;
        *pData++ = 0x4f;
        *pData++ = 0x4f;
        *pData++ = 0xff;
        break;
      case GridToGraph::BOUNDARY | GridToGraph::WALL:
        *pData++ = 0x00;
        *pData++ = 0xff;
        *pData++ = 0x00;
        *pData++ = 0xff;
        break;
      default:
        *pData++ = xy ? 0xff : 0x00;
        *pData++ = xy ? 0xff : 0x00;
        *pData++ = xy ? 0xff : 0x00;
        *pData++ = xy ? 0xff : 0x00;
        break;
      }
    }
  }
  bool ret =
      Stuff::TGA::saveToFile(name, grid[0].size(), grid.size(), 32, pPixel);
  delete[] pPixel;
  LOG_DEBUG("TGA: " << name << " => " << (ret ? "saved" : "*FAILED*"));
}
void makeTGA(const char *name, const Grid &grid, bool preProcess = false) {
  // Pre-process was just have bottom word (WALL,EMPTY)
  unsigned int mask = preProcess ? 0xffff : 0xffff0000;
  makeTGA2(name, grid, mask);
}
#else
void makeTGA2(const char *name, const Grid &grid, unsigned int mask) {}
void makeTGA(const char *name, const Grid &grid, bool preProcess = false) {}
#endif
} // namespace

/////////////////////////////////////////////////////////////////////////////////////////
//
// For a 2x2 of path cells turn it into a 2x2 of NODEs. makeGridNodes() will
// clready have marked 2 of them.
// Return a list of the 2x2 squares, each with 4 node indices (new or existing)
//
std::vector<std::vector<int>> createFixNodes(const Grid &grid, std::vector<Point> &nodes)
{
  std::vector<std::vector<int>> fixNodes;
  int rows = grid.size();
  int cols = grid[0].size();

  auto twoByTwo = [&grid](int x,int y) -> bool {
    return (grid[y][x] && grid[y][x+1] && grid[y+1][x] && grid[y+1][x+1]);
  };

  LOG_INFO("##ADD FIX NODES");
  const std::vector<Point> square = {{0,0},{1,0},{0,1},{1,1}};
  for (int y = 1; y < rows - 2; ++y) {
    for (int x = 1; x < cols - 2; ++x) {
      if (twoByTwo(x,y)) {
        LOG_INFO("  Found 2x2 path cells. Top-Left Corner: " << x << "," << y);
        std::vector<int> fixSquare;
        fixSquare.reserve(4);
        for (auto [dx,dy] : square) {
          int nx = x+dx;
          int ny = y+dy;
          auto it = std::find(nodes.begin(),nodes.end(),Point{nx,ny});
          if (it == nodes.end()) {
            LOG_INFO("    Adding Node " << nodes.size() << " at " << nx << "," << ny);
            fixSquare.push_back(nodes.size());
            nodes.push_back({nx,ny});
          }
          else {
            fixSquare.push_back(static_cast<int>(std::distance(nodes.begin(),it)));
            LOG_INFO("    Node " << fixSquare.back() << " already exists at " << nx << "," << ny);
          }
        }
        fixNodes.push_back(fixSquare);
      }
    }
  }
  LOG_INFO("##FIXED NODES: " << fixNodes.size());
  return fixNodes;
}

/////////////////////////////////////////////////////////////////////////////////////////

//
// Set the Node and DeadEnds
//
void markGridNodes(Grid &grid, const std::vector<Point> &nodes,
                   const std::vector<Point> &deadEnds) {
  int nodeIdx = GridToGraph::NODE;
  int dendIdx = GridToGraph::DEND;
  for (const auto &n : nodes) {
    grid[n.second][n.first] = nodeIdx++;
  }
  for (const auto &de : deadEnds) {
    grid[de.second][de.first] = dendIdx++;
  }
}

//
// Set the Paths to the edge index
//
void markGridPaths(Grid &grid, const std::vector<Edge> &edges) {
  int edgeIdx = GridToGraph::EDGE;
  for (const auto &e : edges) {
    edgeIdx &= ~GridType::EDGE_HALF; // clear half-flag carried over from previous edge
    int halfway = e.path.size() / 2;
    for (const auto &p : e.path) {
      if (isPath(grid[p.second][p.first])) {
        grid[p.second][p.first] = edgeIdx;
      }
      if (halfway && (--halfway == 0)) {
        edgeIdx |= GridType::EDGE_HALF;
      }
    }
    ++edgeIdx;
  }
}

// Validates the infoGrid after markGridPaths has stamped EDGE indices:
//  1. Every EDGE cell that has exactly 2 EDGE neighbours must share the same
//     edge index with both of them (a cell can't be a junction between two
//     distinct edges).
//  2. Every EDGE cell must appear in exactly one Edge::path (no cell claimed
//     by multiple edges, and none encoded in the grid but absent from every
//     path).
static void validateEdgeCells(const Grid &grid,
                               const std::vector<Edge> &edges) {
  const int rows = static_cast<int>(grid.size());
  const int cols = (rows > 0) ? static_cast<int>(grid[0].size()) : 0;

  // Build a map: cell -> list of edge indices whose path contains that cell.
  std::unordered_map<GridType::Point, std::vector<int>,
                     GridType::PairHash> cellToEdges;
  for (int ei = 0; ei < static_cast<int>(edges.size()); ++ei) {
    for (const auto &p : edges[ei].path) {
      cellToEdges[p].push_back(ei);
    }
  }

  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const int cell = grid[r][c];
      if (!(cell & GridType::EDGE))
        continue;
      const int edgeIdx = cell & GridType::EDGE_MASK;
      const GridType::Point pt{c, r};

      // Check 1: neighbours
      std::vector<int> neigbourEdgeIndices;
      for (const auto &d : GridType::directions8) {
        const int nc = c + d.first;
        const int nr = r + d.second;
        const int ncell = grid[nr][nc];
        if (ncell & (GridType::DEND | GridType::NODE)) {
          neigbourEdgeIndices.clear();
          break;
        }
        if (!(ncell & GridType::EDGE))
          continue;
        const int nEdgeIdx = ncell & GridType::EDGE_MASK;
        neigbourEdgeIndices.push_back(nEdgeIdx);
      }

      for (int i : neigbourEdgeIndices) {
        if (i != edgeIdx) {
          LOG_ERROR("VAL: EDGE cell (" << c << "," << r << ") idx=" << edgeIdx
                                       << " has neighbour edge: " << i);
        }
      }

      // Check 2: exactly one path claims this cell
      auto it = cellToEdges.find(pt);
      if (it == cellToEdges.end()) {
        LOG_ERROR("VAL: EDGE cell (" << c << "," << r << ") idx=" << edgeIdx
                  << " is not in any Edge::path");
      } else if (it->second.size() > 1) {
        LOG_ERROR("VAL: EDGE cell (" << c << "," << r << ") idx=" << edgeIdx
                  << " is in " << it->second.size() << " Edge::paths");
      }
    }
  }
}

// Detect self-loop PATH structures (A<->A): PATH cells that form a cycle back
// to the same NODE and were never incorporated into any edge.
//
// Call after fixBaseEdges() (edge set is complete) and before mergeFixNodes()
// (no deleted-node cells yet). At that point every PATH cell in infoGrid has
// raw value 1; any such cell absent from all Edge::paths is orphaned.
//
// For each connected component of orphaned cells the function:
//   1. Identifies the single adjacent NODE (the loop anchor).
//   2. DFS-traces the ordered path from the node, through orphaned cells,
//      back to the node.
//   3. Logs the node index, position, and ordered path.
static std::vector<Edge> detectLoopEdges(const Grid &infoGrid,
                             const std::vector<Edge> &edges,
                             const std::vector<Point> &nodes) {
  LOG_INFO("## detectLoopEdges");
  std::vector<Edge> loops;
  const int rows = static_cast<int>(infoGrid.size());
  const int cols = (rows > 0) ? static_cast<int>(infoGrid[0].size()) : 0;

  // Phase 1: build set of every grid point in any edge path.
  std::unordered_set<Point, GridType::PairHash> edgeCells;
  for (const auto &e : edges)
    for (const auto &p : e.path)
      edgeCells.insert(p);

  // Phase 2: mark orphaned PATH cells (value==1, not in any edge).
  std::vector<std::vector<bool>> isOrphaned(rows, std::vector<bool>(cols, false));
  bool anyOrphaned = false;
  for (int r = 0; r < rows; ++r)
    for (int c = 0; c < cols; ++c)
      if (infoGrid[r][c] == PATH && !edgeCells.count({c, r})) {
        isOrphaned[r][c] = true;
        anyOrphaned = true;
      }

  if (!anyOrphaned) {
    LOG_INFO("detectLoopEdges: OK");
    return loops;
  }

  // Phase 3: BFS to find connected components of orphaned cells.
  std::vector<std::vector<bool>> compVisited(rows, std::vector<bool>(cols, false));

  for (int r0 = 0; r0 < rows; ++r0) {
    for (int c0 = 0; c0 < cols; ++c0) {
      if (!isOrphaned[r0][c0] || compVisited[r0][c0])
        continue;

      // BFS: collect component cells.
      std::vector<Point> comp;
      std::queue<Point> q;
      q.push({c0, r0});
      compVisited[r0][c0] = true;
      while (!q.empty()) {
        auto [cx, cy] = q.front();
        q.pop();
        comp.push_back({cx, cy});
        for (const auto &d : directions8) {
          int nx = cx + d.first, ny = cy + d.second;
          if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
          if (isOrphaned[ny][nx] && !compVisited[ny][nx]) {
            compVisited[ny][nx] = true;
            q.push({nx, ny});
          }
        }
      }

      // Phase 4: find unique adjacent NODEs.
      std::unordered_set<int> adjNodeIdxs;
      for (const auto &[cx, cy] : comp) {
        for (const auto &d : directions8) {
          int nx = cx + d.first, ny = cy + d.second;
          if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
          int cell = infoGrid[ny][nx];
          if (cell & NODE)
            adjNodeIdxs.insert(cell & 0xffff);
        }
      }

      if (adjNodeIdxs.size() != 1) {
        // We assume this is where we have cut a corner of a path
        // The infoGrid hasn't been re-written since doing so would
        // lose all the orphaned cells (since it would match the list of edges)
        LOG_DEBUG("  IGNORE cut corner? at " << c0 << "," << r0);
        continue;
      }

      const int nodeIdx = *adjNodeIdxs.begin();
      const Point &nodePos = nodes[static_cast<std::size_t>(nodeIdx)];

      // Phase 5: DFS to trace ordered loop path from nodePos back to nodePos.
      std::vector<std::vector<bool>> dfsVisited(rows, std::vector<bool>(cols, false));
      Path loopPath;

      // Recursive DFS lambda: extend path through orphaned cells until we
      // reach a neighbour equal to nodePos (cycle closed).
      std::function<bool(int, int)> dfsLoop = [&](int cx, int cy) -> bool {
        for (const auto &d : directions8) {
          int nx = cx + d.first, ny = cy + d.second;
          if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
          // Cycle closed: found our way back to the anchor node.
          if (nx == nodePos.first && ny == nodePos.second) {
            loopPath.push_back(nodePos);
            return true;
          }
          // Continue through unvisited orphaned cells.
          if (isOrphaned[ny][nx] && !dfsVisited[ny][nx]) {
            dfsVisited[ny][nx] = true;
            loopPath.push_back({nx, ny});
            if (dfsLoop(nx, ny)) return true;
            loopPath.pop_back();
            dfsVisited[ny][nx] = false;
          }
        }
        return false;
      };

      bool found = false;
      for (const auto &d : directions8) {
        int sx = nodePos.first + d.first, sy = nodePos.second + d.second;
        if (sx < 0 || sx >= cols || sy < 0 || sy >= rows) continue;
        if (!isOrphaned[sy][sx]) continue;
        dfsVisited[sy][sx] = true;
        loopPath = {nodePos, {sx, sy}};
        if (dfsLoop(sx, sy)) {
          found = true;
          break;
        }
        dfsVisited[sy][sx] = false;
        loopPath.clear();
      }

      if (found) {
        if (loopPath.size() > 4) {
          LOG_INFO("  LOOP at node " << nodeIdx << " (" << nodePos.first << ","
                                     << nodePos.second
                                     << "): " << loopPath.size() << " cells");
          LOG_DEBUG_FOR(const auto &p : loopPath, "    " << p.first << "," << p.second);
          loops.push_back({ nodeIdx, nodeIdx, false, loopPath});
        } else {
          // It detects 3 cell PATHs e.g. 1->x->1. The min is 5 cells
          // since need a wall e.g
          //   0  
          // 1 # 3    0->1->2->3->0  so length = 5
          //   2  
          LOG_DEBUG_CONT("  IGNORE loop at " << nodeIdx << " (" << nodePos.first << ","
                                     << nodePos.second
                                     << "): " << loopPath.size() << " cells");
          LOG_DEBUG_FOR(const auto &p : loopPath, "    " << p.first << "," << p.second);
        }
      } else {
        LOG_ERROR("  orphaned cluster at " << c0 << "," << r0
                 << " size=" << comp.size() << " adjacent to node "
                 << nodeIdx << " but DFS found no loop path (anomaly)");
      }
    }
  }
  LOG_INFO("detectLoopEdges: " << loops.size() << " loop(s) found");
  return loops;
}
/////////////////////////////////////////////////////////////////////////////////////////

//
// Split a loop edge into two edges by adding a new NODE
// The new NODE is placed halfway along the loop path.
// e.g. Loop Edge 0->1->2->3->0 becomes edges 0->1->N
// and N->3->0 where N is the new NODE at cell 2
//
// The loop edges are not currently in the edges and need added.
// The loop node is in the list of nodes, so the new one needs added.
//
void splitLoopEdges(const std::vector<Edge>& loops,
                    std::vector<Edge> &edges, std::vector<Point> &nodes) {
  LOG_INFO("## splitLoopEdges"); 
  auto addEdge = [&edges](const Edge& e) -> void{
    edges.push_back(e);
    LOG_INFO("  New edge: " << edges.back().from << "->" << edges.back().to
             << " path: " << edges.back().path.size());
    LOG_DEBUG_CONT("  NewPath:");
    LOG_DEBUG_FOR(const auto &p : edges.back().path, " " << p.first << "," << p.second);
  };

  for (const auto &loop : loops) {
    const Path &loopPath = loop.path;
    const std::size_t mid = loopPath.size() / 2;

    // Add new node at the midpoint of the loop path
    const int newNodeIdx = static_cast<int>(nodes.size());
    nodes.push_back(loopPath[mid]);
    LOG_INFO("  New node: " << newNodeIdx << " at " << loopPath[mid].first << "," << loopPath[mid].second);
    
    // First edge: loop.from → new node (first half of path inclusive of mid)
    // +1 so that we include the new node in the path
    addEdge({loop.from, newNodeIdx, false,
                     Path(loopPath.begin(), loopPath.begin() + mid +1)});

    // Second edge: loop.from -> new node (second half of path starting at mid)
    // This is because Edges are stored as from < to
    Path revPath(loopPath.rbegin(), loopPath.rbegin() + (loopPath.size() - mid));
    addEdge({loop.from, newNodeIdx, false, revPath});
  }
}
                    
/////////////////////////////////////////////////////////////////////////////////////////

void markGridBoundaries(Grid &grid) {
  int rows = grid.size();
  int cols = grid[0].size();

  for (int r = 1; r < rows - 1; ++r) {
    for (int c = 1; c < cols - 1; ++c) {
      if (!(grid[r][c] & WALL)) {
        for (int dirIdx = 0; dirIdx < directions8.size(); ++dirIdx) {
          int nc = c + directions8[dirIdx].first;
          int nr = r + directions8[dirIdx].second;
          if (grid[nr][nc] & WALL) {
            // Tag wall with BOUNDARY and direction toward walkable space
            // - Clear out old dir bits and apply new ones safely
            int &cell = grid[nr][nc];
            cell = (cell & ~DIR_MASK) |
                   (GridType::reverseDirIndex[dirIdx] & DIR_MASK) | WALL |
                   BOUNDARY;
          }
        }
      }
    }
  }
}

//
// Before can expand need to put back the WALLs
// i.e. floorGrid had zeroes wherever walls, but graph.infoGrid
// has zeroes everywhere except path. So change graph.infoGrid
// to have WALL wherever was 0 in floorGrid
//
void restoreWalls(Grid &infoGrid, const Grid &floorGrid) {
  int rows = floorGrid.size();
  int cols = floorGrid[0].size();
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      if (!isPath(floorGrid[y][x])) {
        infoGrid[y][x] = GridToGraph::WALL;
      }
    }
  }
}

void markWallEscapes(Grid &infoGrid) {
  int rows = (int)infoGrid.size();
  int cols = (int)infoGrid[0].size();
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      int &cell = infoGrid[y][x];
      if (!(cell & GridType::WALL) || (cell & GridType::BOUNDARY))
        continue;
      int bestDist = 0;
      int bestDir  = 0;
      for (int d = 0; d < (int)GridType::directions8.size(); ++d) {
        auto dxy = GridType::directions8[d];
        int nx = x + dxy.first;
        int ny = y + dxy.second;
        int steps = 1;
        while (nx >= 0 && nx < cols && ny >= 0 && ny < rows) {
          if (!(infoGrid[ny][nx] & GridType::WALL)) {
            if (bestDist == 0 || steps < bestDist) {
              bestDist = steps;
              bestDir  = d;
            }
            break;
          }
          nx += dxy.first;
          ny += dxy.second;
          ++steps;
        }
      }
      if (bestDist > 0) {
        cell |= (bestDir & GridType::DIR_MASK) |
                (bestDist << WALL_DIST_SHIFT);
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////

Path simplifyPath(const Path& inPath, const Grid& floorGrid) {
  Path simplePath;
  if (inPath.size() == 2) {
    int dx = std::abs(inPath[0].first - inPath[1].first);
    int dy = std::abs(inPath[0].second - inPath[1].second);
    LOG_DEBUG_CONT("simplifyPath: SHORT "
                   << inPath[0].first << "," << inPath[0].second << " -> "
                   << inPath[1].first << "," << inPath[1].second);
    if (dx == 1 && dy == 1) {
      bool isCornerWall1 = !floorGrid[inPath[0].second][inPath[1].first];
      bool isCornerWall2 = !floorGrid[inPath[1].second][inPath[0].first];
      if ((isCornerWall1 ^ isCornerWall2) == true) {
        LOG_DEBUG(" IGNORE");
        return simplePath;
      }
      else {
        LOG_DEBUG(" KEEP " << isCornerWall1 << " " << isCornerWall2);
      } 
    }
    else {
      LOG_DEBUG(" NOT DIAG");
    }
  }
  if (inPath.size() < 3)
    return inPath;

  LOG_DEBUG("simplifyPath: inPath.size():" << inPath.size());
  simplePath.push_back(inPath[0]);

  for (size_t i = 1; i < inPath.size() - 1; i++) {
    const auto &prev = inPath[i - 1];
    const auto &curr = inPath[i];
    const auto &next = inPath[i + 1];

    LOG_DEBUG("  idx:" << i << " prev:" << prev.first << "," << prev.second
                       << " curr:" << curr.first << "," << curr.second
                       << " next:" << next.first << "," << next.second);
    // Check if "prev -> next" is a valid diagonal move
    int dx = next.first - prev.first;
    int dy = next.second - prev.second;
    if (std::abs(dx) <= 1 && std::abs(dy) <= 1) {
      // Check if the inner corner clips a solid tile
      // This is to help movement around corners, trying to
      // move diagonal can hit the corner of a wall
      bool isSafe = true;
#if 1
      if (std::abs(dx) == 1 && std::abs(dy) == 1) {
        // The corner is the one that is not prev or next or curr
        // of the 2x2 square
        int cornerX = (prev.first == curr.first) ? next.first
                     : (prev.first == next.first ? curr.first
                     : prev.first);
        int cornerY = (prev.second == curr.second) ? next.second
                     : (prev.second == next.second ? curr.second
                     : prev.second);

        // If 0 (EMPTY) => not floor => corner is wall
        bool isCornerWall = !floorGrid[cornerY][cornerX];
        LOG_DEBUG("      DIAG c1: " << cornerX << "," << cornerY << " => " << isCornerWall);
        // If corner is WALL (not-floor) then 3 cells are L shape going round wall
        // So do NOT go diagonal to avoid cutting corner
        if (isCornerWall) {
          LOG_DEBUG("      => do not cut wall");
          isSafe = false;
        }
      }
#endif
      if (isSafe) {
        // Skip the current point (diagonal move is valid)
        simplePath.push_back(next);
        LOG_DEBUG("  => store NXT (inc i)");
        i++;
      } else {
        LOG_DEBUG("  => store CUR");
        simplePath.push_back(curr);
      }
    } else {
      LOG_DEBUG("  => store CUR");
      simplePath.push_back(curr);
    }
  }

  // Ensure the last point is added
  if (simplePath.back() != inPath.back()) {
    simplePath.push_back(inPath.back());
  }

  return simplePath;
}

/////////////////////////////////////////////////////////////////////////////////////////

// Custom comparator as a lambda function
struct EdgeComparator {
  constexpr bool operator()(const Edge &a, const Edge &b) const {
    if (a.from != b.from)
      return a.from < b.from;
    if (a.to != b.to)
      return a.to < b.to;
    if (a.toDeadEnd != b.toDeadEnd)
      return a.toDeadEnd < b.toDeadEnd;
    return a.path < b.path; // Compare paths
  }
};

class EdgeAdder {
  std::set<Edge, EdgeComparator> edges;

  const Grid &grid;
  const Grid &floorGrid;
  std::set<std::pair<int, int>> adjacentNodePairs;

public:
  // Grid has NODE, DEND, PATH, EMPTY (must be 0)
  EdgeAdder(const Grid &g, const Grid &f) : grid(g), floorGrid(f) {}

  // Accessor Methods
  const std::set<Edge, EdgeComparator> &getEdges() { return edges; }
  const Grid &getGrid() { return grid; }

  // Create a new edge from the Path and add it to the set
  bool addEdge(Path &path) {
    Edge newEdge;
    if (!getEdge(path, newEdge)) {
      return false;
    }

    auto itr = edges.find(newEdge);
    if (itr != edges.end()) {
      LOG_DEBUG_CONT(" => DUP AddEdge EDGE: "
                     << newEdge.from << "->" << newEdge.to
                     << (newEdge.toDeadEnd ? " (DEAD)" : "") << " P: ");
      LOG_DEBUG_FOR(const auto &p : newEdge.path,
                    p.first << "," << p.second << "  ");
      return false;
    }

    LOG_DEBUG_CONT(" STORE AddSEdge: " << newEdge.from << "->" << newEdge.to
                                       << (newEdge.toDeadEnd ? " (DEAD)" : "")
                                       << " P: ");
    LOG_DEBUG_FOR(const auto &p : newEdge.path,
                  p.first << "," << p.second << "  ");

    edges.insert(itr, newEdge);
    return true;
  }

private:
  // Populate newEdge with the Edge created from the Path (if it is valid)
  bool getEdge(Path &path, Edge &newEdge) {
    if (path.empty()) {
      LOG_DEBUG("getEdge => EMPTY");
      return false;
    }
    int sx = path[0].first;
    int sy = path[0].second;
    int ex = path[path.size() - 1].first;
    int ey = path[path.size() - 1].second;

    int startRC = grid[sy][sx];
    int endRC = grid[ey][ex];

    int node0 = startRC & 0xffff;
    int node1 = endRC & 0xffff;
    bool isDead0 = (startRC & DEND);
    bool isDead1 = (endRC & DEND);
    bool isDeadEnd = isDead0 || isDead1;
    bool bothDead = isDeadEnd && (isDead0 == isDead1);

    // Should not both be dead and if Node0 == Node1 then one must be a deadEnd
    bool startEndOk = (!bothDead) && ((node0 != node1) || isDeadEnd);
    if (!startEndOk) {
      LOG_DEBUG(" GetEdge START = END: " << (isDead0 ? "DEAD " : "") << node0
                                         << " == " << (isDead1 ? "DEAD " : "")
                                         << node1);
      return false;
    }

    // 2 nodes next to each other.
    // Store the Pairs and use it to abort early
    if (path.size() == 2) {
      // Store in (smallerNode, largerNode) order
      std::pair<int, int> edge = (node0 < node1) ? std::make_pair(node0, node1)
                                                 : std::make_pair(node1, node0);

      if (adjacentNodePairs.count(edge)) {
        LOG_DEBUG(" GetEdge DUP: " << (isDeadEnd ? "DEAD " : "") << edge.first
                                   << " to " << edge.second);
        return false;
      }
      adjacentNodePairs.insert(edge);
    }

    LOG_DEBUG_CONT(" GetEdge " << ((startRC & DEND) ? "DEND " : "NODE ")
                               << (startRC & 0xffff) << " -> "
                               << ((endRC & DEND) ? "DEND " : "NODE ")
                               << (endRC & 0xffff) << "   P: ");
    LOG_DEBUG_FOR(const auto &p : path, "  " << p.first << "," << p.second);

    // Ensure Node1 is the dead, or if none dead then node0 is the lower
    if (isDead0 || (!isDeadEnd && (node0 > node1))) {
      std::swap(isDead0, isDead1);
      std::swap(node0, node1);
      std::reverse(path.begin(), path.end());
    }

    // Simplify the path so wont get duplicates
    Path simplePath = simplifyPath(path, floorGrid);
    if (simplePath.empty()) {
      LOG_DEBUG("=> IGNORE PATH");
      return false;
    }
    if (simplePath.size() != path.size()) {
      LOG_DEBUG_CONT(" GetEdge INPUT  ");
      LOG_DEBUG_FOR(const auto &p : path, "  " << p.first << "," << p.second);
      LOG_DEBUG_CONT(" GetEdge SIMPLE ");
      LOG_DEBUG_FOR(const auto &p : simplePath,
                    "  " << p.first << "," << p.second);
    }

    // Make the new Edge to add
    newEdge = {node0, node1, isDeadEnd, simplePath};
    return true;
  }
};

//
// Starting at the given path cell in the grid find the edge and add it
// Visited tracks which cells have been visited for the path except for
// NODE/DEND cells which are never marked as visited (since there are
// multiple paths to them, but path cells are used once
//
Path followPath(int startX, int startY, EdgeAdder &edgeAdder,
                std::vector<std::vector<bool>> &visited) {
  LOG_DEBUG("## followPath " << startX << "," << startY);
  // If we fail to find a valid path, return empty
  Path emptyPath;
  // Grid has NODE, DEND, PATH, EMPTY (must be 0)
  const Grid &grid = edgeAdder.getGrid();

  // Directions in your preferred order: up, left, right, down, then diagonals
  static const std::vector<std::pair<int, int>> directionsHVD = {
      {0, -1},  // up
      {-1, 0},  // left
      {1, 0},   // right
      {0, 1},   // down
      {-1, -1}, // diag up-left
      {1, -1},  // diag up-right
      {-1, 1},  // diag down-left
      {1, 1}    // diag down-right
  };

  // We'll keep a copy of 'visited' to revert if needed
  std::vector<std::vector<bool>> tempVisited = visited;

  // Helper: Mark a PATH cell visited in tempVisited
  auto markPathVisited = [&](int x, int y) { tempVisited[y][x] = true; };

  // If the start cell is PATH, mark it visited in the temp array
  if (isPath(grid[startY][startX])) {
    markPathVisited(startX, startY);
  }

  // =========================
  // Try each direction in HVD
  // =========================
  for (auto hvd : directionsHVD) {
    auto dx = hvd.first;
    auto dy = hvd.second;
    int nx = startX + dx;
    int ny = startY + dy;

    // Skip empty or visited path cells
    if (grid[ny][nx] == 0)
      continue; // EMPTY
    LOG_DEBUG("  dir:" << dx << "," << dy << "  => " << nx << "," << ny);
    if (isPath(grid[ny][nx]) && tempVisited[ny][nx]) {
      LOG_DEBUG("  " << nx << "," << ny << " => Visited (or Node");
      continue; // Already visited path cell
    }

    // FORWARD PATH: from neighbor
    std::deque<std::pair<int, int>> forwardPath;
    std::pair<int, int> fwd = {nx, ny};
    LOG_DEBUG("  => FWD Start " << fwd.first << "," << fwd.second);

    while (true) {
      // If it's a PATH cell, mark visited
      if (isPath(grid[fwd.second][fwd.first]) &&
          !tempVisited[fwd.second][fwd.first]) {
        markPathVisited(fwd.first, fwd.second);
        LOG_DEBUG("    VISIT " << fwd.first << "," << fwd.second);
      }

      forwardPath.push_back(fwd);

      // If we reached a node, check if it's the same node or a new one
      if (isNode(grid[fwd.second][fwd.first])) {
        // If it's literally the same node as (startX, startY), skip it
        if (fwd.first == startX && fwd.second == startY) {
          // This forward direction loops back to the start node => break/fail
          // We'll try the next direction
          LOG_DEBUG_CONT("       Unvist: ");
          for (const auto p : forwardPath) {
            if (isPath(grid[p.second][p.first])) {
              tempVisited[p.second][p.first] = false;
              LOG_DEBUG_CONT(p.first << "," << p.second << " ");
            }
          }
          LOG_DEBUG("");
          forwardPath.clear();
          LOG_DEBUG("    LOOP => clear path");
        }
        LOG_DEBUG("  NODE " << fwd.first << "," << fwd.second << " => BREAK");
        break;
      }

      // Try directions8 again in HVD order from 'fwd'
      bool foundNext = false;
      for (auto [dx2, dy2] : directionsHVD) {
        int fx = fwd.first + dx2;
        int fy = fwd.second + dy2;

        // Skip empty or visited path
        if (grid[fy][fx] == 0)
          continue;
        if (isPath(grid[fy][fx]) && tempVisited[fy][fx])
          continue;
        LOG_DEBUG("    " << fx << "," << fy
                         << " => Not visited (or is Node) => BREAK");
        // We found an unvisited path or a node => move forward
        fwd = {fx, fy};
        foundNext = true;
        break;
      }
      if (!foundNext) {
        // Could not proceed to any next cell => path fails
        LOG_DEBUG_CONT("  FWD NOT FOUND => clear: ");
        LOG_DEBUG_FOR(const auto p : forwardPath, p.first << "," << p.second << "  ");
        forwardPath.clear();
        break;
      }
    }

    // If forwardPath is empty => that direction failed, try next direction
    if (forwardPath.empty()) {
      LOG_DEBUG("  EMPTY => Continue");
      continue;
    }

    // BACKWARD PATH: from (startX, startY)
    std::deque<std::pair<int, int>> backwardPath;
    std::pair<int, int> bck = {startX, startY};

    while (true) {
      LOG_DEBUG("  BCK " << bck.first << "," << bck.second);
      // If it's PATH, mark visited
      if (((grid[bck.second][bck.first] & (NODE | DEND)) == 0) // is PATH
          && !tempVisited[bck.second][bck.first]) {
        markPathVisited(bck.first, bck.second);
        LOG_DEBUG("    VISIT " << bck.first << "," << bck.second);
      }

      backwardPath.push_front(bck);

      // If we reached a node, we stop
      if ((grid[bck.second][bck.first] & (NODE | DEND)) != 0) {
        LOG_DEBUG("    NODE => break");
        break;
      }

      bool foundNext = false;
      for (auto [dx2, dy2] : directionsHVD) {
        // NOTE: subtract the dir to check dirs in opposite directions8 to
        // forward
        int bx = bck.first - dx2;
        int by = bck.second - dy2;
        LOG_DEBUG("    bck dir: " << dx2 << "," << dy2 << " => " << bx << "," << by
                  << " grid: " << std::hex << (int)grid[by][bx] << std::dec);
        if (grid[by][bx] == 0)
          continue; // empty
        if (isPath(grid[by][bx]) && tempVisited[by][bx]) {
          LOG_DEBUG("      " << bx << "," << by << " => Visited (or Node)");
          continue;
        }

        bck = {bx, by};
        foundNext = true;
        LOG_DEBUG("      " << bx << "," << by << " => bck found next");
        break;
      }
      if (!foundNext) {
        LOG_DEBUG_CONT("    BCK NOT FOUND => clear: ");
        LOG_DEBUG_FOR(const auto p : backwardPath, p.first << "," << p.second << "  ");
        // Could not proceed => path fails
        backwardPath.clear();
        break;
      }
    }

    // If backward path is empty => that direction fails, revert & try next
    if (backwardPath.empty()) {
      LOG_DEBUG("  BCK EMPTY => CONTINUE");
      continue;
    }

    // If we get here, we have forward + backward paths => combine them
    // They share the start cell once, but typically that's okay.
    Path result;
    for (auto &p : backwardPath) {
      result.push_back(p);
    }
    for (auto &p : forwardPath) {
      result.push_back(p);
    }

    // We have a valid path => commit changes to 'visited'
    if (edgeAdder.addEdge(result)) {
      visited = tempVisited;
      return result;
    }
    tempVisited = visited; // Revert changes
    LOG_DEBUG("  ADD EDGE FAILED => Revert");
    // return emptyPath;
  }

  // If no direction yields a path, revert everything and return empty
  return emptyPath;
}

//
// Grid has NODE, DEND, PATH, EMPTY (must be 0)
//
std::vector<Edge> findEdges(const Grid &grid, const Grid &floorGrid) {
  LOG_INFO("##FINDEDGES");
  const int rows = (int)grid.size();
  const int cols = (int)grid[0].size();
  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

  EdgeAdder edgeAdder(grid, floorGrid);

  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      if (grid[y][x] == EMPTY) {
        continue;
      }
      if (isPath(grid[y][x]) && visited[y][x]) {
        continue;
      }

      LOG_DEBUG("#PNT " << x << "," << y << "   Grid: " << (grid[y][x]));

      Path path = followPath(x, y, edgeAdder, visited);
    }
  }
  const std::set<Edge, EdgeComparator> &edges = edgeAdder.getEdges();
  std::vector<Edge> result;
  result.reserve(edgeAdder.getEdges().size());
  std::copy(edges.begin(), edges.end(), std::back_inserter(result));
  return result;
}

Grid rewriteInfoGrid(const Grid &grid, const std::vector<Edge> &edges) {
  Grid result = Grid(grid.size(), std::vector<int>(grid[0].size(), EMPTY));
  for (const auto &e : edges) {
    for (const auto &p : e.path) {
      result[p.second][p.first] = PATH;
    }
  }
  // Preserve NODE and DEND cells from the grid — they already encode their
  // index (set by markGridNodes) and must win over any PATH written above
  for (int r = 0; r < static_cast<int>(grid.size()); ++r) {
    for (int c = 0; c < static_cast<int>(grid[0].size()); ++c) {
      if ((grid[r][c] & NODE) || (grid[r][c] & DEND)) {
        result[r][c] = grid[r][c];
      }
    }
  }
  return result;
}

  std::vector<Edge> findNodeEdges(const std::vector<Point> &nodes,
                                  const Grid &grid, const Grid &floorGrid) {
  const int rows = (int)grid.size();
  const int cols = (int)grid[0].size();
  std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

  EdgeAdder edgeAdder(grid, floorGrid);

  LOG_INFO("## FIND NODE EDGES for " << nodes.size() << " unconnected nodes");
  for (const auto &n : nodes) {
    int x = n.first;
    int y = n.second;
    Path path;
    do {
      path = followPath(x, y, edgeAdder, visited);
    } while (!path.empty());
  }
  const std::set<Edge, EdgeComparator> &edges = edgeAdder.getEdges();
  std::vector<Edge> result;
  result.reserve(edgeAdder.getEdges().size());
  std::copy(edges.begin(), edges.end(), std::back_inserter(result));
  LOG_INFO("  FOUND " << result.size() << " edges");
  return result;
}

//
// Unfortunately there are a few scenarios that dont connect. Basically
// some places aren't marked as nodes (some areas where "double/triple thick"
// This allows a path between A and B that bypasses C, but marks the
// path cells as visited, so C doesnt have a path of A or B e.g.
//
//    A--x---B    x isn't marked as a node
//       |        (real scenario is more messy)
//       C        So A->B cells visited => no paths from C
//
// To fix it have version of findEdges that just checks for paths from the
// list of nodes. Any new edges are added to the complete list
//
// NOTE: I think this can create wrong edges e.g. get A->x and C->A,
// so 'x' is bypassed (because of diagonal) however I think the fixNodes
// now means this who function is not needed.
//
BaseGraph fixBaseEdges(std::vector<Edge> &baseEdges, const Grid &infoGrid,
                       const Grid &floorGrid,
                       const std::vector<Point> &baseNodes) {
  LOG_INFO("## FIX BASE EDGES");
  // First Make a graph edgeNode1 -> list of (edgeNode2, edgeIndex, 1->2 flag,
  // cost)
  BaseGraph baseGraph = GridType::buildBaseGraph(baseEdges, baseNodes.size());

  // Use graph to check all base nodes connected
  std::vector<int> unconnected =
      GridType::checkConnectivity(baseGraph, baseNodes.size());
  if (unconnected.empty()) {
    return baseGraph;
  }
  LOG_INFO("  Unconnected nodes: " << unconnected.size());

  // Turn list of unconnected node indexes to list of Nodes
  std::vector<Point> unconnectedNodes;
  for (int idx : unconnected) {
    unconnectedNodes.push_back(baseNodes[idx]);
  }

  // Find the edges for those nodes
  std::vector<Edge> connections =
      findNodeEdges(unconnectedNodes, infoGrid, floorGrid);

  // Make an EdgeComparator set of all current edges
  GridToGraph::EdgeComparator comp;
  std::set<Edge, EdgeComparator> edgeSet(baseEdges.begin(), baseEdges.end(),
                                         comp);
  for (const auto &e : connections) {
    if (edgeSet.find(e) == edgeSet.end()) {
      edgeSet.insert(e);
      baseEdges.push_back(e);
    }
  }

  // Return the new graph with the new edges
  return buildBaseGraph(baseEdges, baseNodes.size());
}

/////////////////////////////////////////////////////////////////////////////////////////

//
// Each fixNodes entry is 4 node indexs which form a 2x2 square of nodes
// This function merges those nodes into a single node and deletes the 3 others
// It updates the edges and the grid with the new information i.e.
// - Any Edge using a deleted node uses the new node and it's path is changed
// - The grid of a deleted node has NODE removed becaomes an EDGE with the halfwaybit
//  and the edge index
//
// Therefore the number of edges does not change, but 3*fixNodes.size() nodes are removed
//
void mergeFixNodes(const std::vector<std::vector<int>> &fixNodes,
                   std::vector<Point> &nodes, std::vector<Edge> &edges,
                   Grid &grid) {
  LOG_INFO("## MERGE FIX NODES: fixes: " << fixNodes.size()
          << " nodes: " << nodes.size() << " edges: " << edges.size());
  if (fixNodes.empty()) return;

  // Phase 1: Build remap (deleted -> survivor).  Survivor = lowest index in group.
  // Path-compress afterwards so chains caused by overlapping 2x2 blocks resolve
  // to the ultimate survivor.
  std::unordered_map<int, int> remap;
  std::set<int> deleted;

  for (const auto &group : fixNodes) {
    int keep = *std::min_element(group.begin(), group.end());
    LOG_INFO("  Keep:" << keep << " from "
      << group[0] << "," << group[1] << "," << group[2] << "," << group[3]);
    for (int idx : group) {
      if (idx != keep) {
        remap[idx] = keep;
        deleted.insert(idx);
        LOG_INFO("    node " << idx << "(" << nodes[idx].first << "," << nodes[idx].second << ")"
           << " merged to " << keep << "(" << nodes[keep].first << "," << nodes[keep].second << ")");
      }
    }
  }

  LOG_INFO("FIXING CHAINS");
  for (auto &kv : remap) {
    int final = kv.second;
    while (remap.count(final)) {
      LOG_INFO("  Following chain: " << final << "->" << remap[final]);
      final = remap[final];
    }
    LOG_INFO("  -> Map " << kv.first << " from " << kv.second << " to " << final);
    kv.second = final;
  }

  auto remapNode = [&](int idx) -> int {
    auto it = remap.find(idx);
    return (it != remap.end()) ? it->second : idx;
  };

  // Phase 2: Update edges — remap endpoints, extend paths at deleted nodes,
  // remove self-loops (intra-block edges), deduplicate.
  //std::set<std::tuple<int, int, bool>> seenPairs;
  //std::set<Edge, EdgeComparator> seenEdges;
  std::vector<Edge> newEdges;

  LOG_INFO("FIXING EDGES");
  for (auto &e : edges) {
    const bool fromDeleted = deleted.count(e.from) > 0;
    const bool toDeleted   = !e.toDeadEnd && deleted.count(e.to) > 0;

    const int newFrom = remapNode(e.from);
    const int newTo   = e.toDeadEnd ? e.to : remapNode(e.to);

    bool changed = (newFrom != e.from) || (newTo != e.to);
    if (changed) LOG_DEBUG("Edge " << e.from << "->" << e.to << " becomes " << newFrom << "->" << newTo);
    // Intra-block edge becomes a self-loop after merging — discard it
    if (!e.toDeadEnd && newFrom == newTo) {
      LOG_INFO("  removing self-loop edge " << e.from << "->" << e.to);
      continue;
    }

    // Extend path so it terminates at the survivor's cell, not the deleted cell.
    // The deleted cell becomes an interior EDGE cell via markGridPaths downstream.
    if (fromDeleted) {
      LOG_DEBUG("  prefix path with node: " << newFrom << " "
        << nodes[newFrom].first << "," << nodes[newFrom].second);
      e.path.insert(e.path.begin(), nodes[newFrom]);
    }
    if (toDeleted) {
      LOG_DEBUG("  extend path with node: " << newTo << " "
        << nodes[newTo].first << "," << nodes[newTo].second);
      e.path.push_back(nodes[newTo]);
    }

    e.from = newFrom;
    e.to   = newTo;

    // Normalise: non-dead-end edges must have from < to (matches getEdge convention)
    if (!e.toDeadEnd && e.from > e.to) {
      std::swap(e.from, e.to);
      std::reverse(e.path.begin(), e.path.end());
      LOG_DEBUG("  swapped edge " << e.from << "->" << e.to);
    }
#if 0
    // Deduplicate by (from, to, toDeadEnd)
    auto key = std::make_tuple(e.from, e.to, e.toDeadEnd);
    if (seenPairs.count(key)) {
      LOG_INFO("  dropping duplicate edge " << e.from << "->" << e.to);
      continue;
    }
    seenPairs.insert(key);
#endif
    newEdges.push_back(std::move(e));
    if (changed) LOG_DEBUG("=>New Edge: " << e.from << "->" << e.to);
  }
  edges = std::move(newEdges);

  // Phase 3: Clear deleted node cells to PATH=1 so rewriteInfoGrid/markGridPaths
  // can reclaim them as interior edge cells.
  for (int idx : deleted) {
    const Point &p = nodes[idx];
    grid[p.second][p.first] = PATH;
    LOG_INFO("Need to fix grid cell: " << p.first <<","<< p.second);
  }

  // Phase 4: Compact nodes vector, build old-index -> new-index map.
  const int oldSize = static_cast<int>(nodes.size());
  std::vector<int> newIdx(oldSize, -1);
  std::vector<Point> newNodes;
  newNodes.reserve(oldSize - static_cast<int>(deleted.size()));
  for (int i = 0; i < oldSize; ++i) {
    if (!deleted.count(i)) {
      newIdx[i] = static_cast<int>(newNodes.size());
      newNodes.push_back(nodes[i]);
      LOG_DEBUG("Node: " << i << "=> New Node: " << newIdx[i] << " " << nodes[i].first << "," << nodes[i].second);
    }
  }
  nodes = std::move(newNodes);

  // Phase 5: Remap edge from/to to compacted indices.
  for (auto &e : edges) {
    LOG_DEBUG("Edge: " << e.from << "->" << e.to << " becomes "
                       << newIdx[e.from] << "->" << (e.toDeadEnd ? -e.to : newIdx[e.to]));
    e.from = newIdx[e.from];
    if (!e.toDeadEnd) {
      e.to = newIdx[e.to];
    }
    if (e.from < 0 || (!e.toDeadEnd && e.to < 0))
      LOG_ERROR("mergeFixNodes: invalid node index after compaction");
  }

  // Phase 6: Update all NODE grid cells to use compacted indices.
  // DEND cells are unaffected (dead-end indices are in a separate namespace).
  LOG_DEBUG("Update Node indices in infoGrid");
  for (int r = 0; r < static_cast<int>(grid.size()); ++r) {
    for (int c = 0; c < static_cast<int>(grid[r].size()); ++c) {
      const int cell = grid[r][c];
      if (cell & NODE) {
        const int oldNodeIdx = cell & 0xffff;
        const int ni = newIdx[oldNodeIdx];
        if (ni >= 0) {
          LOG_DEBUG("Node: " << oldNodeIdx << "=> New Node: " << ni << " at " << c <<","<< r);
          grid[r][c] = NODE | ni;
        }
      }
    }
  }

  LOG_INFO("  MERGE FIX NODES Done: remmoved: " << deleted.size() 
           << " nodes: " << nodes.size() << " edges: " << edges.size());
}

/////////////////////////////////////////////////////////////////////////////////////////

void expandPaths(Grid &infoGrid) {
  LOG_INFO("## EXPAND PATHS");
  int rows = infoGrid.size();
  int cols = infoGrid[0].size();

  // Queue: (row, col, distance, dir, srcX, srcY)
  std::queue<std::tuple<int, int, int, int, int, int>> q;

  // Initialize from ALL source types
  LOG_DEBUG("Generate expansion queue");
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      int cell = infoGrid[r][c];

      LOG_DEBUG(" " << c << "," << r << " = " << std::hex << cell << std::dec);
      // Check if this is any type of source cell
      if ((cell & GridType::NODE) || (cell & GridType::DEND) ||
          (cell & GridType::EDGE)) {
        q.emplace(c, r, 0, -1, c, r);
        LOG_DEBUG("  PUSH " << c << "," << r << " " << std::hex << cell
                            << std::dec);
      }
    }
  }

  // BFS expansion with distance checking
  LOG_DEBUG("BFS Expansion of queue");
  while (!q.empty()) {
    auto [x, y, dist, dir, srcX, srcY] = q.front();
    q.pop();

    LOG_DEBUG("  Check: " << x << ", " << y << " dir:" << dir << " dst:" << dist
                          << " from srcxy: " << srcX << "," << srcY);
    for (int d = 0; d < 8; ++d) {
      int nx = x + directions8[d].first;
      int ny = y + directions8[d].second;
      int &cell = infoGrid[ny][nx];

      // Must be empty
      if (cell & 0xffff0000)
        continue;
      LOG_DEBUG("    checkDir: " << d << " xy: " << nx << "," << ny << " = "
                                 << std::hex << cell << std::dec);

      int revDir = (d + 4) & 7;
      if (revDir == dir) {
        // Store XPND
        int newDist = dist + 1;
        cell = GridType::XPND | (dir << GridType::XPND_DIR_SHIFT) | (dist + 1);
        LOG_DEBUG("    CNTU: " << nx << "," << ny << " dir:" << dir
                               << " dist:" << (dist + 1) << " (" << std::hex
                               << cell << std::dec << ")");
        q.emplace(nx, ny, (dist + 1), dir, srcX, srcY);
      } else {
        LOG_DEBUG("    TURN: " << nx << "," << ny << " dir:" << revDir
                               << " dist:1"
                               << " (" << std::hex << cell << std::dec << ")");
        cell = GridType::XPND | (revDir << GridType::XPND_DIR_SHIFT) | 1;
        q.emplace(nx, ny, 1, revDir, x, y);
      }
    }
  }
}

//////////////////////////////////////////////////////////////

// Euclidean distance function
double euclideanDistance(const Point &a, const Point &b) {
  return std::sqrt(std::pow(a.first - b.first, 2) +
                   std::pow(a.second - b.second, 2));
}

// Finds the base node closest to the cluster center
int findCentralNode(const AbstractNode &abstractNode,
                    const std::vector<Point> &nodes) {
  if (abstractNode.baseNodes.empty())
    return -1;

  Point centroid = abstractNode.center;
  int closestNode = abstractNode.baseNodes[0];
  double minDistance = euclideanDistance(centroid, nodes[closestNode]);

  for (int nodeIdx : abstractNode.baseNodes) {
    double d = euclideanDistance(centroid, nodes[nodeIdx]);
    if (d < minDistance) {
      minDistance = d;
      closestNode = nodeIdx;
    }
  }
  return closestNode;
}

// **Finds the shortest path between two base nodes using BFS**
Path findPathBetweenNodes(int startNode, int endNode,
                          const std::vector<Edge> &edges,
                          const std::vector<Point> &nodes) {
  std::queue<int> q;
  std::vector<int> prev(nodes.size(), -1);
  std::vector<int> edgeUsed(nodes.size(), -1);
  std::vector<bool> visited(nodes.size(), false);

  LOG_INFO("PATH BETWEEN: " << startNode << " -> " << endNode);
  q.push(startNode);
  visited[startNode] = true;

  while (!q.empty()) {
    int current = q.front();
    q.pop();

    if (current == endNode)
      break;

    for (int edgeIdx = 0; edgeIdx < edges.size(); ++edgeIdx) {
      const auto &edge = edges[edgeIdx];
      if (edge.toDeadEnd)
        continue;

      int neighbor = -1;
      if (edge.from == current)
        neighbor = edge.to;
      else if (edge.to == current)
        neighbor = edge.from;

      if (neighbor != -1 && !visited[neighbor]) {
        LOG_DEBUG("  Checked edge: " << edgeIdx << " " << edge.from << " -> "
                                     << edge.to << " cur: " << current
                                     << " neigh: " << neighbor);
        visited[neighbor] = true;
        prev[neighbor] = current;
        edgeUsed[neighbor] = edgeIdx;
        q.push(neighbor);
        LOG_DEBUG("  store: " << neighbor << " = " << current);
      }
    }
  }

  // If no path was found, return empty path
  if (prev[endNode] == -1)
    return {};

  // **Reconstruct the path from startNode -> endNode**
  Path fullPath;
  int at = endNode;

  while (at != startNode) {
    int edgeIdx = edgeUsed[at];
    if (edgeIdx == -1)
      break;

    const auto &edgePath = edges[edgeIdx].path;
    if (edges[edgeIdx].to == at) {
      fullPath.insert(fullPath.end(), edgePath.begin(), edgePath.end());
    } else {
      fullPath.insert(fullPath.end(), edgePath.rbegin(), edgePath.rend());
    }

    at = prev[at];
  }
  LOG_DEBUG_CONT("  NODES: ");
  LOG_DEBUG_FOR(int n : prev, n << " ");
  LOG_DEBUG_CONT("FINAL PATH BETWEEN: " << startNode << " -> " << endNode);
  LOG_DEBUG_FOR(const auto &pnt : fullPath,
                "  " << pnt.first << "," << pnt.second);

  return fullPath;
}

static std::vector<int> dbscan(const std::vector<Point> &points, double eps,
                               int minPts) {
  const int UNVISITED = -1;
  const int NOISE = -2;

  std::vector<int> labels(points.size(), UNVISITED);
  int clusterId = 0;

  auto regionQuery = [&](const Point &p) -> std::vector<int> {
    std::vector<int> neighbors;
    for (size_t i = 0; i < points.size(); ++i) {
      if (euclideanDistance(p, points[i]) <= eps) {
        neighbors.push_back(i);
      }
    }
    return neighbors;
  };

  auto expandCluster = [&](int pointIdx, int clusterId,
                           const std::vector<int> &neighbors) {
    labels[pointIdx] = clusterId;

    std::vector<int> queue = neighbors;
    while (!queue.empty()) {
      int idx = queue.back();
      queue.pop_back();

      if (labels[idx] == UNVISITED) {
        labels[idx] = clusterId;

        auto subNeighbors = regionQuery(points[idx]);
        if (subNeighbors.size() >= static_cast<size_t>(minPts)) {
          queue.insert(queue.end(), subNeighbors.begin(), subNeighbors.end());
        }
      }

      if (labels[idx] == NOISE) {
        labels[idx] = clusterId;
      }
    }
  };

  for (size_t i = 0; i < points.size(); ++i) {
    if (labels[i] != UNVISITED) {
      continue;
    }

    auto neighbors = regionQuery(points[i]);
    if (neighbors.size() < static_cast<size_t>(minPts)) {
      labels[i] = NOISE;
    } else {
      expandCluster(i, clusterId, neighbors);
      ++clusterId;
    }
  }

  return labels;
}
////////////////////////////////////////////////////////////////////

// **Creates the Abstract Graph from clustered base nodes**
std::vector<AbstractNode> createAbstractNodes(const std::vector<Point> &nodes,
                                              double clusteringEps,
                                              int minClusterSize) {
  // Step 1: Cluster points
  std::vector<int> clusterLabels = dbscan(nodes, clusteringEps, minClusterSize);

  // Step 2: Create Abstract Nodes
  // NOTE: We temporarily use center to calc the geometric center
  std::unordered_map<int, AbstractNode> clusters;
  clusters.reserve(nodes.size());
  for (size_t i = 0; i < nodes.size(); ++i) {
    int clusterId = clusterLabels[i];
    if (clusterId < 0) continue; // UNVISITED or NOISE: skip
    auto &cluster = clusters[clusterId]; // inserts if missing
    cluster.baseNodes.push_back(static_cast<int>(i));
    cluster.center.first += nodes[i].first;
    cluster.center.second += nodes[i].second;
  }
  // FIX: If all the nodes are noise, create a cluster for each node
  // i.e. each base node is an abstract node
  if (clusters.empty()) {
    for (size_t i = 0; i < nodes.size(); ++i) {
      auto &cluster = clusters[i];
      cluster.baseNodes.push_back(static_cast<int>(i));
      cluster.center = nodes[i];
    }
  }

  // Calculate the geometric center of each cluster
  // NOTE: This will be overwritten by the closest base node in Step 4
  // but it is used to find the closest base node in Step 4
  // This is done to ensure center is a FLOOR tile position
  LOG_DEBUG("##CREATE ANODEs. clusters: " << clusters.size());
  for (auto &[i, cluster] : clusters) {
    int totalPoints = cluster.baseNodes.size();
    cluster.center.first /= totalPoints;
    cluster.center.second /= totalPoints;
  }
  // Step 3: Collect abstract nodes into a vector
  std::vector<AbstractNode> abstractNodes;
  for (const auto &[_, cluster] : clusters) {
    abstractNodes.push_back(cluster);
  }

  // Step 4: Assign closest base node to each abstract node
  // NOTE: We overwrite the center with the closest base node
  std::unordered_map<int, int> closestMap;
  int idx = 0;
  for (auto &abNode : abstractNodes) {
    int baseIdx = findCentralNode(abNode, nodes);
    if (baseIdx == -1) {
      LOG_ERROR("Could not find central node for idx: " << idx);
    }
    abNode.baseCenterNode = baseIdx;
    abNode.center = nodes[baseIdx]; // Guarantee center is a valid floor point
    closestMap[idx] = baseIdx;
    ++idx;
  }

  LOG_DEBUG("   => MADE ABNODEs: " << abstractNodes.size());
  return abstractNodes;
}

///////////////////////////////////////////////////////////////////////

// Compute direction as an index into the directions8 array
int computeDirection(const Point &from, const Point &to) {
  int dx = to.first - from.first;
  int dy = to.second - from.second;

  // Normalize dx and dy to -1, 0, or 1
  dx = (dx == 0) ? 0 : (dx / std::abs(dx));
  dy = (dy == 0) ? 0 : (dy / std::abs(dy));

  // Find the index in the directions8 array
  for (size_t i = 0; i < directions8.size(); ++i) {
    if (directions8[i].first == dx && directions8[i].second == dy) {
      return i;
    }
  }
  return -1; // This should never happen
}

//////////////////////////////////////////////////////////////////////

std::vector<std::vector<int>>
computeAllPathDists(const std::vector<Edge> &baseEdges, int numNodes) {
  // Build adjacency list
  std::vector<std::vector<std::pair<int, int>>> adjList(numNodes);
  for (const auto &edge : baseEdges) {
    if (edge.toDeadEnd)
      continue;
    int weight = edge.path.size();
    adjList[edge.from].emplace_back(edge.to, weight);
    adjList[edge.to].emplace_back(edge.from, weight);
  }

  // Initialize distance matrix
  const int INF = std::numeric_limits<int>::max();
  std::vector<std::vector<int>> dist(numNodes, std::vector<int>(numNodes, INF));

  for (int start = 0; start < numNodes; ++start) {
    std::vector<int> localDist(numNodes, INF);
    localDist[start] = 0;
    std::set<std::pair<int, int>> pq;
    pq.insert({0, start});

    while (!pq.empty()) {
      auto [currentDist, currentNode] = *pq.begin();
      pq.erase(pq.begin());
      if (currentDist > localDist[currentNode])
        continue;

      for (const auto &[neighbor, weight] : adjList[currentNode]) {
        int newDist = currentDist + weight;
        if (newDist < localDist[neighbor]) {
          if (localDist[neighbor] != INF) {
            pq.erase({localDist[neighbor], neighbor});
          }
          localDist[neighbor] = newDist;
          pq.insert({newDist, neighbor});
        }
      }
    }

    for (int end = 0; end < numNodes; ++end) {
      dist[start][end] = localDist[end];
    }
  }

  return dist;
}

////////////////////////////////////////////////////////

//
// Generates the Voronoi-like zones around each AbstractNode using path-based
// BFS
// - Populates ZoneGrid with closest AbstractNode index and distance to it
//
std::vector<ZoneInfo>
generateAbstractZones(ZoneGrid &zoneGrid, const Grid &grid,
                      const BaseGraph &baseGraph,
                      const std::vector<Edge> &baseEdges,
                      const std::vector<AbstractNode> &abstractNodes) {
  LOG_INFO("## generateAbstractZones");
  int rows = grid.size();
  int cols = grid[0].size();

  // Create ZoneGrid. Note that GridPointInfo inis each field to appropriate
  // value
  zoneGrid = ZoneGrid(rows, std::vector<GridPointInfo>(cols));

  using QueueElement =
      std::tuple<int, int, int,
                 double>; // (row, col, abstractNodeIdx, pathCost)
  std::queue<QueueElement> q;

  // Initialize BFS from each Abstract Node
  for (size_t i = 0; i < abstractNodes.size(); ++i) {
    Point nodePos = abstractNodes[i].center;
    // Add abstract node to queue at dist 0
    q.push({nodePos.first, nodePos.second, static_cast<int>(i), 0});
    // set the navgrid closes node to itself and dist to 0
    zoneGrid[nodePos.second][nodePos.first].closestAbstractNodeIdx = i;
    zoneGrid[nodePos.second][nodePos.first].distanceToAbstractNode = 0;
  }

  // Multi-source path-based BFS
  LOG_DEBUG("  QSIZE: " << q.size());
  while (!q.empty()) {
    auto [c, r, abstractIdx, cost] = q.front();
    q.pop();

    // Check the neighbors of the cur zone cell
    for (const auto &[dc, dr] : directions8) {
      int nc = c + dc;
      int nr = r + dr;

      if (grid[nr][nc] & WALL) {
        continue;
      }

      // Expand cost of zone
      int newCost = cost + 1;

      // Only update if we found a shorter path (dist stats at MAX)
      if (newCost < zoneGrid[nr][nc].distanceToAbstractNode) {
        zoneGrid[nr][nc].closestAbstractNodeIdx = abstractIdx;
        zoneGrid[nr][nc].distanceToAbstractNode = newCost;
        // Add this cell to the queue for further expansion
        q.emplace(nc, nr, abstractIdx, newCost);
      }
    }
  }

  // Sloped edge cells are tagged WALL|BOUNDARY but are physically occupiable.
  // The BFS skips all WALL cells, leaving them with closestAbstractNodeIdx=-1.
  // Copy GridPointInfo from the adjacent walkable cell (indicated by the low
  // 3 bits of the BOUNDARY cell) so zone-level routing works when the agent
  // stands against a slope.
  for (int r = 1; r < rows - 1; ++r) {
    for (int c = 1; c < cols - 1; ++c) {
      int cell = grid[r][c];
      if ((cell & WALL) && (cell & BOUNDARY)) {
        int dirIdx = cell & DIR_MASK;
        int wc = c + directions8[dirIdx].first;
        int wr = r + directions8[dirIdx].second;
        zoneGrid[r][c] = zoneGrid[wr][wc];
      }
    }
  }

  std::vector<ZoneInfo> zones(abstractNodes.size());

  // If this edge crosses zones update the zone infos
  auto addEdge = [baseEdges, zoneGrid](int edgeIdx,
                                       std::vector<ZoneInfo> &zones) -> void {
    const Edge &edge = baseEdges[edgeIdx];
    GridType::Point from = edge.path.front();
    GridType::Point to = edge.path.back();
    int fromZone = zoneGrid[from.second][from.first].closestAbstractNodeIdx;
    int toZone = zoneGrid[to.second][to.first].closestAbstractNodeIdx;

    LOG_DEBUG("  Processing edge "
              << edgeIdx << " from (" << from.first << "," << from.second
              << ") to (" << to.first << "," << to.second << ")"
              << " fromZone=" << fromZone << " toZone=" << toZone);

    // Add the edge to ALL zones it passes through (fromZone and toZone)
    std::vector<int> zonesToAdd = {fromZone, toZone};

    // If the edge crosses zones
    if (fromZone != toZone) {
      // Add the boundaryZone if not already in the adjacentZones list
      if (std::find(zones[fromZone].adjacentZones.begin(),
                    zones[fromZone].adjacentZones.end(),
                    toZone) == zones[fromZone].adjacentZones.end()) {
        zones[fromZone].adjacentZones.push_back(toZone);
        zones[toZone].adjacentZones.push_back(fromZone);
      }
    }

    // Add the edge to zones it belongs to
    for (int zoneIdx : zonesToAdd) {
      if (zones[zoneIdx].baseEdgeIdxSet.insert(edgeIdx).second) {
        zones[zoneIdx].baseEdgeIdxs.push_back(edgeIdx);
        LOG_DEBUG("    Added edge " << edgeIdx << " to zone " << zoneIdx);
      } else {
        LOG_DEBUG("    Edge " << edgeIdx << " already in zone " << zoneIdx);
      }
    }
  };

  // First, add all edges from the graph to their appropriate zones
  LOG_DEBUG("Processing " << baseEdges.size() << " total edges");
  for (int edgeIdx = 0; edgeIdx < baseEdges.size(); ++edgeIdx) {
    const Edge &edge = baseEdges[edgeIdx];
    /*
    // COMMENTED OUT: We need DeadEnd edges in the Zone list!
    // Why? Because Walkabout tracks visited DeadEnds as visited EDGES.
    // If we exclude them here, the Zone Edge Count (Total) is smaller than
    Visited Count (Current),
    // causing premature "Reset Cycle" logic in NavigatorWalkabout.
    if (edge.toDeadEnd) {
      LOG_DEBUG("  Skipping edge " << edgeIdx << " (dead end)");
      continue; // Skip dead end edges
    }
    */
    addEdge(edgeIdx, zones);
  }

  // generate Nodes and boundaries
  for (int y = 1; y < rows - 1; ++y) {
    for (int x = 1; x < cols - 1; ++x) {
      int cell = grid[y][x];
      if (cell & WALL)
        continue;

      int curZone = zoneGrid[y][x].closestAbstractNodeIdx;
      int curBaseNode = abstractNodes[curZone].baseCenterNode;

      // Bottom bits of infoGrid are node or edge index
      int idx = cell & GridType::EDGE_MASK;

      // If node and/or new zone boundary then add to zone's list
      if (cell & NODE) {
        zones[curZone].baseNodeIdxs.push_back(idx);
        zones[curZone].baseNodeIdxSet.insert(idx);
      }
    }
  }

  return zones;
}

////////////////////////////////////////////////////////
#if 0
// Resolve which baseEdge index a boundary cell at (x,y) belongs to. Used to
// segment BoundaryCellMap into one slot per base-edge crossing.
//   EDGE  → low bits are baseEdge index directly.
//   NODE  → scan baseGraph[nodeIdx]; the connecting baseEdge is the one whose
//           neighbor base node lives in `neighborZone`.
//   XPND  → walk dist*dir per hop while still XPND; terminus is EDGE or NODE.
//   else  → 8-neighbour scan for any EDGE cell; sentinel -1 if nothing found.
static int findEdgeIdxForCell(const Grid &infoGrid,
                              const BaseGraph &baseGraph,
                              const std::vector<Edge> &baseEdges,
                              const std::vector<Point> &baseNodes,
                              const std::vector<Point> &dendNodes,
                              const ZoneGrid &zoneGrid,
                              int x, int y, int neighborZone) {
  int rows = static_cast<int>(infoGrid.size());
  int cols = static_cast<int>(infoGrid[0].size());

  auto nodeToEdge = [&](int nodeIdx) -> int {
    if (nodeIdx < 0 || nodeIdx >= static_cast<int>(baseNodes.size()))
      return -1;
    for (const auto &info : baseGraph[nodeIdx]) {
      if (info.neighbor < 0 ||
          info.neighbor >= static_cast<int>(baseNodes.size()))
        continue;
      const Point &nbrPt = baseNodes[info.neighbor];
      int nbrZone = zoneGrid[nbrPt.second][nbrPt.first].closestAbstractNodeIdx;
      if (nbrZone == neighborZone) {
        return info.edgeIndex;
      }
    }
    LOG_ERROR("Could not find edge for node: " << nodeIdx << " zone: " << neighborZone);
    return -1;
  };

  auto dendToEdge = [&](int dendIdx) -> int {
    // A dead-end is a leaf — exactly one edge connects to it, with
    // toDeadEnd=true and to==dendIdx. The edge's `from` is a base node;
    // forward to nodeToEdge to find the crossing edge into neighborZone.
    for (const auto &e : baseEdges) {
      if (e.toDeadEnd && e.to == dendIdx) {
        return nodeToEdge(e.from);
      }
    }
    LOG_ERROR("Could not find edge for dend: " << dendIdx
                                               << " zone: " << neighborZone);
    return -1;
  };

  auto xpndToEdge = [&](int x, int y) -> int {
    // Follow XPND pointers — multiply dist*dir per hop — until terminus
    // lands on an EDGE or NODE cell.
    int nx = x;
    int ny = y;
    int nCell = infoGrid[y][x];
    int safety = 0;

    while (nCell & XPND) {
      int dir = get_XPND_DIR(nCell);
      int dst = get_XPND_DIST(nCell);

      nx += directions8[dir].first * dst;
      ny += directions8[dir].second * dst;

      if (!isInBounds(nx, ny, rows, cols)) {
        nCell = 0;
        break;
      }

      nCell = infoGrid[ny][nx];

      if (++safety > 64)
        break; // pathological case
    }

    if (nCell & EDGE) return nCell & EDGE_MASK;
    if (nCell & NODE) return nodeToEdge(nCell & EDGE_MASK);
    if (nCell & DEND) return dendToEdge(nCell & EDGE_MASK);
    LOG_ERROR("Could not find edge for XPND: " << x << "," << y << " in zone "
                                               << neighborZone);
    return -1;
  };

  int cell = infoGrid[y][x];

  if (cell & EDGE) return cell & EDGE_MASK;

  if (cell & XPND) return xpndToEdge(x, y);
  if (cell & NODE) return nodeToEdge(cell & EDGE_MASK);
  if (cell & DEND) return dendToEdge(cell & EDGE_MASK);
  if (cell & BOUNDARY) {
    int dirIdx = cell & GridType::DIR_MASK;
    int wx = x + GridType::directions8[dirIdx].first;
    int wy = y + GridType::directions8[dirIdx].second;
    if (!isInBounds(wx, wy, rows, cols)) {
      return -1;
    }
    cell = infoGrid[wy][wx];
    if (cell & WALL) return -1;
    if (cell & EDGE) return cell & EDGE_MASK;
    if (cell & NODE) return nodeToEdge(cell & EDGE_MASK);
    if (cell & XPND) return xpndToEdge(wx, wy);
  }
  // Ignore pure WALL cells
  return -1;
}
#endif

void generateZoneBoundaries(AbstractLevel &abstractLevel,
                            const Grid &infoGrid,
                            const BaseGraph &baseGraph,
                            const std::vector<Edge> &baseEdges,
                            const std::vector<Point> &baseNodes,
                            const std::vector<Point> &dendNodes) {
  LOG_INFO("## generateZoneBoundaries");
  const ZoneGrid &zoneGrid = abstractLevel.zoneGrid;
  std::vector<ZoneInfo> &zones = abstractLevel.zones;
  int rows = zoneGrid.size();
  int cols = zoneGrid[0].size();

  // For each unordered zone pair, map baseEdge index → slot index. Both sides
  // of a crossing share the slot (slot-parity), so they line up across the
  // two zones' BoundaryCellMaps.
  std::unordered_map<std::pair<int, int>, std::unordered_map<int, int>,
                     PairHash>
      pairEdgeSlots;

  auto ensureSlot = [](BoundaryCellMap &bcm, int neighborZone,
                       int slot) -> BoundaryCells & {
    auto &vec = bcm[neighborZone];
    if (static_cast<int>(vec.size()) <= slot)
      vec.resize(slot + 1);
    return vec[slot];
  };
  // Phase 1: collect every zone-crossing cell into slot 0 of both sides.
  for (int y = 1; y < rows - 1; ++y) {
    for (int x = 1; x < cols - 1; ++x) {
      int curZone = zoneGrid[y][x].closestAbstractNodeIdx;
      if (curZone == -1)
        continue;
      LOG_DEBUG("Cell: " << x << "," << y << " is in zone " << curZone);

      // Only 4 dirs needed — left-to-right, top-to-bottom scan covers the
      // mirrored crossing in the same iteration.
      for (int dir4 = 0; dir4 < static_cast<int>(searchDirs4.size()); ++dir4) {
        int nx = x + searchDirs4[dir4].first;
        int ny = y + searchDirs4[dir4].second;

        int neighborZone = zoneGrid[ny][nx].closestAbstractNodeIdx;
        LOG_DEBUG("..Check:" << nx << "," << ny << ":" << neighborZone);
        if (neighborZone == -1 || neighborZone == curZone)
          continue;

        int dir8 = dir4todir8Index[dir4];
        LOG_DEBUG("....=> dir8:" << dir8 << " add " << x <<","<< y << " " << nx<<","<<ny);
        ensureSlot(zones[curZone].zoneBoundaryCellMap, neighborZone, 0)
            .insert({{x, y}, dir8});
        ensureSlot(zones[neighborZone].zoneBoundaryCellMap, curZone, 0)
            .insert({{nx, ny}, reverseDirIndex[dir8]});
      }
    }
  }
  for (int zA=0; zA < zones.size(); ++zA) {
    for (auto &kv : zones[zA].zoneBoundaryCellMap) {
      LOG_DEBUG("Zone " << zA << " has boundary with zone " << kv.first);
      for (auto &bi : kv.second[0]) {
        LOG_DEBUG_CONT(" " << bi.sink.first << "," << bi.sink.second);
      }
      LOG_DEBUG(" ");
    }
  }

  // ---------------------------------------------------------------------------
  // Phase 2 — Split each (zoneA <-> zoneB) crossing into "islands".
  //
  // After Phase 1, every zone pair (zA, zB) that touches has exactly ONE slot
  // in its zoneBoundaryCellMap: kv.second[0] holds every cell on zA's side of
  // the shared border between the two zones. But two zones can be adjacent
  // along multiple physically separate stretches of border (e.g. two doorways
  // in the same wall, or a corridor that wraps around an obstacle). Each such
  // stretch is an "island" — a connected component of boundary cells.
  //
  // Goal of Phase 2: replace that single slot with one slot PER island, and
  // do it symmetrically so that zA->zB slot k and zB->zA slot k describe the
  // SAME crossing from opposite sides. Downstream code (FlowField, Guard
  // navigator, ZoneBridgeEdge graph) keys behaviour on this slot index, so
  // the two sides MUST agree on the indexing.
  //
  // We can't just BFS one side and mirror via exitDir, because BoundaryCells
  // is keyed by sink position only — a cell that points in two directions
  // collapses to a single stored exitDir, so some B-side cells have no A-side
  // cell pointing at them. The robust fix is to flood-fill over the UNION of
  // both sides' cell sets under 8-neighbour adjacency. Each connected
  // component naturally splits into its A-members and B-members, giving us a
  // matched pair of slots without any direction-mirroring guesswork.
  // ---------------------------------------------------------------------------
  for (int zA = 0; zA < static_cast<int>(zones.size()); ++zA) {
    for (auto &kv : zones[zA].zoneBoundaryCellMap) {
      int zB = kv.first;

      // Each crossing is symmetric (zA<->zB == zB<->zA). Process the canonical
      // ordering only — when the outer loop reaches zB it would redo the same
      // work and clobber what we just wrote.
      if (zA >= zB)
        continue;

      // Phase 1 may have left the slot empty if the two zones don't actually
      // share any boundary cells; nothing to split.
      if (kv.second.empty())
        continue;

      // cellsA = zA's view of the border with zB (Phase-1 single slot).
      const BoundaryCells &cellsA = kv.second[0];

      // Look up the symmetric entry on zB's side. If it's missing or empty
      // the data is malformed for this pair; skip rather than crash.
      auto bIt = zones[zB].zoneBoundaryCellMap.find(zA);
      if (bIt == zones[zB].zoneBoundaryCellMap.end() || bIt->second.empty())
        continue;
      const BoundaryCells &cellsB = bIt->second[0];

      // Build position->BoundaryInfo lookups for both sides. We need O(1)
      // "is there an A-cell here? a B-cell here?" tests during the flood fill,
      // and we need to recover the original BoundaryInfo (sink + exitDir +
      // edgeIdx) when assigning a cell to its island bucket.
      std::unordered_map<Point, BoundaryInfo, PairHash> pointToA;
      pointToA.reserve(cellsA.size());
      for (const auto &bi : cellsA)
        pointToA.emplace(bi.sink, bi);

      std::unordered_map<Point, BoundaryInfo, PairHash> pointToB;
      pointToB.reserve(cellsB.size());
      for (const auto &bi : cellsB)
        pointToB.emplace(bi.sink, bi);

      // An island is a pair (A-cells in this component, B-cells in this
      // component). Either vector may be empty if the component happens to
      // sit entirely on one side, but in practice both will populate because
      // boundary cells come from edges that cross the zone divide.
      using IslandPair =
          std::pair<std::vector<BoundaryInfo>, std::vector<BoundaryInfo>>;
      std::vector<IslandPair> islands;

      // Visited set is shared across all flood fills in this (zA, zB) pair so
      // we don't re-enter a component when iterating seeds.
      std::unordered_set<Point, PairHash> visited;
      visited.reserve(pointToA.size() + pointToB.size());

      // Iterative DFS flood-fill from `start`, walking 8-connected cells that
      // belong to either pointToA or pointToB. Every visited cell is added to
      // the appropriate side(s) of the current island. A cell that exists in
      // BOTH maps (same sink position appears on both sides — possible at
      // diagonal corner pinches) is added to both islA and islB.
      auto seedFrom = [&](const Point &start) {
        LOG_DEBUG("Starting island at: " << start.first << "," << start.second);
        std::vector<BoundaryInfo> islA;
        std::vector<BoundaryInfo> islB;

        std::vector<Point> stack{start};
        visited.insert(start);

        while (!stack.empty()) {
          Point p = stack.back();
          stack.pop_back();

          // Record this cell on the A side if it came from cellsA.
          auto itA = pointToA.find(p);
          if (itA != pointToA.end()) {
            islA.push_back(itA->second);
            LOG_DEBUG("..Adding A " << p.first << "," << p.second);
          }
          // ...and/or on the B side if it came from cellsB.
          auto itB = pointToB.find(p);
          if (itB != pointToB.end()) {
            islB.push_back(itB->second);
            LOG_DEBUG("..Adding B " << p.first << "," << p.second);
          }

          // Expand to 8-connected neighbours. A neighbour is part of THIS
          // island only if it (a) has not been visited, and (b) appears in
          // either pointToA or pointToB (i.e. it is a boundary cell for this
          // particular zone pair — neighbours that belong to other crossings
          // or to the zone interior are not part of the component).
          for (const auto &d : directions8) {
            Point np{p.first + d.first, p.second + d.second};
            if (visited.count(np))
              continue;
            if (!pointToA.count(np) && !pointToB.count(np))
              continue;
            visited.insert(np);
            stack.push_back(np);
          }
        }

        // Discard empty components (defensive — shouldn't happen given seeds
        // are themselves boundary cells, but cheap to guard).
        if (!islA.empty() || !islB.empty())
          islands.push_back({std::move(islA), std::move(islB)});
      };

      // Seed from every A-cell first, then every B-cell. The visited set
      // means each component is only flooded once regardless of which side's
      // cell triggers it. Seeding from both sides guarantees we don't miss a
      // component made entirely of B-cells that no A-cell happens to touch.
      for (const auto &bi : cellsA) {
        if (!visited.count(bi.sink))
          seedFrom(bi.sink);
      }
      for (const auto &bi : cellsB) {
        if (!visited.count(bi.sink))
          seedFrom(bi.sink);
      }

      // Materialise the islands into the BoundaryCells slot vectors. Index k
      // is the SAME island viewed from each side, so newSlotsA[k] and
      // newSlotsB[k] together describe one connected crossing of the border.
      std::vector<BoundaryCells> newSlotsA(islands.size());
      std::vector<BoundaryCells> newSlotsB(islands.size());
      LOG_DEBUG("Adding slots to: " << zA << "<->" << zB);
      for (int k = 0; k < static_cast<int>(islands.size()); ++k) {
        LOG_DEBUG("  Island " << k);
        for (const auto &bi : islands[k].first) {
          LOG_DEBUG("    A " << bi.sink.first << "," << bi.sink.second);
          newSlotsA[k].insert(bi);
        }
        for (const auto &bi : islands[k].second) {
          LOG_DEBUG("    B " << bi.sink.first << "," << bi.sink.second);
          newSlotsB[k].insert(bi);
        }
      }

      // Replace the Phase-1 single-slot vectors with the per-island vectors.
      // After this point both zA->zB and zB->zA agree: slot k on either side
      // points at the same physical island in the world.
      kv.second = std::move(newSlotsA);
      bIt->second = std::move(newSlotsB);

      LOG_INFO("Boundary " << zA << "<->" << zB << " islands: "
                              << islands.size());
    }
  }
  LOG_INFO("## generateZoneBoundaries DONE");
}

namespace {

struct ZBEdge {
  int u;
  int v;
  int slot;
  int edgeId;
};
//
// Make list of edges going from low zone to high zone storing
// the lowZone (u), highZone (v), island slot (k), monotonic edgeId (edgeId)
// e.g.
//   0 2 0 0      |--1--(2)----(3)
//   0 2 1 1     (0)     |      |
//   1 3 0 2      |--0----      --2-(1)
//   2 3 0 3 
//
static std::vector<ZBEdge> buildEdgeList(const AbstractLevel &ablv) {
  std::vector<ZBEdge> edges;
  int edgeId = 0;
  for (int u = 0; u < static_cast<int>(ablv.zones.size()); ++u) {
    for (const auto &kv : ablv.zones[u].zoneBoundaryCellMap) {
      int v = kv.first;
      if (u >= v)
        continue;
      const auto &slots = kv.second;
      for (int k = 0; k < static_cast<int>(slots.size()); ++k) {
        if (slots[k].empty())
          continue;
        edges.push_back({u, v, k, edgeId++});
      }
    }
  }
  return edges;
}

static std::vector<bool>
findBridges(const std::vector<std::vector<std::pair<int, int>>> &adj,
            int numEdges, int numNodes) {
  std::vector<bool> isBridge(numEdges, false);
  std::vector<int> disc(numNodes, -1);
  std::vector<int> low(numNodes, -1);
  int timer = 0;

  struct DfsFrame {
    int node;
    int parentEdgeId;
    int it;
  };

  for (int start = 0; start < numNodes; ++start) {
    if (disc[start] != -1)
      continue;
    std::vector<DfsFrame> stack;
    stack.push_back({start, -1, 0});
    disc[start] = low[start] = timer++;
    while (!stack.empty()) {
      DfsFrame &f = stack.back();
      if (f.it < static_cast<int>(adj[f.node].size())) {
        std::pair<int, int> ne = adj[f.node][f.it++];
        int nbr = ne.first;
        int eid = ne.second;
        if (eid == f.parentEdgeId)
          continue;
        if (disc[nbr] == -1) {
          disc[nbr] = low[nbr] = timer++;
          stack.push_back({nbr, eid, 0});
        } else {
          low[f.node] = std::min(low[f.node], disc[nbr]);
        }
      } else {
        int childNode = f.node;
        int childLow = low[childNode];
        int parentEdgeId = f.parentEdgeId;
        stack.pop_back();
        if (!stack.empty()) {
          int parentNode = stack.back().node;
          if (childLow > disc[parentNode] && parentEdgeId >= 0) {
            isBridge[parentEdgeId] = true;
          }
          low[parentNode] = std::min(low[parentNode], childLow);
        }
      }
    }
  }
  return isBridge;
}

static int componentSizeAfterRemoving(const std::vector<ZBEdge> &edges,
                                      int removeEdgeId, int startNode,
                                      int numNodes) {
  std::vector<std::vector<int>> adj(numNodes);
  for (const auto &e : edges) {
    if (e.edgeId == removeEdgeId)
      continue;
    adj[e.u].push_back(e.v);
    adj[e.v].push_back(e.u);
  }
  std::vector<bool> visited(numNodes, false);
  std::vector<int> queue;
  queue.push_back(startNode);
  visited[startNode] = true;
  int count = 0;
  while (!queue.empty()) {
    int n = queue.back();
    queue.pop_back();
    ++count;
    for (int nb : adj[n]) {
      if (visited[nb])
        continue;
      visited[nb] = true;
      queue.push_back(nb);
    }
  }
  return count;
}

} // namespace

static void buildZoneBridgeGraph(AbstractLevel &ablv) {
  ablv.zoneBridgeEdges.clear();
  int numNodes = static_cast<int>(ablv.zones.size());
  if (numNodes == 0)
    return;

  std::vector<ZBEdge> edges = buildEdgeList(ablv);
  if (edges.empty()) {
    LOG_INFO("ZoneBridgeGraph: edges=0 bridges=0");
    return;
  }

  // The edges ia list of all fromLowZone -> toHighZone labeled with unique edgeId
  // Make a mapping of zone -> vector of {adjacentZone, edgeId} pairs
  //
  //    fromZone -> [ {toZone, edgeId}, ... , {...}]
  //    toZone -> [ {fromZone, eqdgeId}, ... , {...}]
  //
  std::vector<std::vector<std::pair<int, int>>> adj(numNodes);
  for (const auto &e : edges) {
    adj[e.u].push_back({e.v, e.edgeId});
    adj[e.v].push_back({e.u, e.edgeId});
  }

  // Find the edges which when cut remove at least one node from the graph
  std::vector<bool> isBridge =
      findBridges(adj, static_cast<int>(edges.size()), numNodes);

  // For the bridge notes see how many nodes remain after removing. Use
  //    min(remaining, total-remaining) as the priority value.

  int bridgeCount = 0;
  ablv.zoneBridgeEdges.reserve(edges.size() * 2);
  for (const auto &e : edges) {
    int priority = 0;
    if (isBridge[e.edgeId]) {
      int s = componentSizeAfterRemoving(edges, e.edgeId, e.u, numNodes);
      priority = std::min(s, numNodes - s);
      ++bridgeCount;
    }
    ablv.zoneBridgeEdges.push_back({e.u, e.v, e.slot, priority});
    ablv.zoneBridgeEdges.push_back({e.v, e.u, e.slot, priority});
  }

  // Sort by 
  //  - priority (descending)
  //  - zoneFrom (ascending)
  //  - zoneTo   (ascending)
  //  - edgeId   (ascending)
  // i.e. if two edges have same priority, the one with lower zoneFrom
  // goes first, if same, then lower zoneTo, if same then use slot
  std::sort(ablv.zoneBridgeEdges.begin(), ablv.zoneBridgeEdges.end(),
            [](const ZoneBridgeEdge &a, const ZoneBridgeEdge &b) {
              if (a.bridgePriority != b.bridgePriority)
                return a.bridgePriority > b.bridgePriority;
              if (a.zoneFrom != b.zoneFrom)
                return a.zoneFrom < b.zoneFrom;
              if (a.zoneTo != b.zoneTo)
                return a.zoneTo < b.zoneTo;
              return a.islandSlot < b.islandSlot;
            });

  int top1 = 0;
  int top2 = 0;
  int top3 = 0;
  for (const auto &zbe : ablv.zoneBridgeEdges) {
    int p = zbe.bridgePriority;
    if (p > top1) {
      top3 = top2;
      top2 = top1;
      top1 = p;
    } else if (p < top1 && p > top2) {
      top3 = top2;
      top2 = p;
    } else if (p < top2 && p > top3) {
      top3 = p;
    }
  }
  LOG_INFO("ZoneBridgeGraph: edges=" << edges.size()
                                     << " bridges=" << bridgeCount
                                     << " top3=" << top1 << "," << top2 << ","
                                     << top3);
}

// Main function to generate extra AbstractEdges
// This is needed since the current AbstractEdges might not
// connect every zone ie. for each pair of adjacnent zones
// there is no AbstractEdge create an "extra" one
std::vector<AbstractEdge>
makeExtraAbstractEdges(const std::vector<Point> &baseNodes,
                       const std::vector<Edge> &baseEdges,
                       const std::vector<AbstractNode> &abstractNodes,
                       const std::vector<AbstractEdge> &abstractEdges,
                       const std::vector<ZoneInfo> &zones,
                       const ZoneGrid &zoneGrid, const BaseGraph &baseGraph) {
  LOG_INFO("## MAKE EXTRA ABSTRACT EDGES");
  // make current connections
  std::unordered_map<std::pair<int, int>, bool, PairHash> connectedZones;
  for (const auto &edge : abstractEdges) {
    connectedZones[{edge.from, edge.to}] = true;
    connectedZones[{edge.to, edge.from}] = true;
  }

  // For each zone, check it's adjacent zones and add edges if not already
  // connected
  std::vector<AbstractEdge> extraEdges;
  for (int zoneA = 0; zoneA < abstractNodes.size(); ++zoneA) {
    const ZoneInfo &infoA = zones[zoneA];
    for (int zoneB : infoA.adjacentZones) {
      // Check already connected
      if (connectedZones.find({static_cast<int>(zoneA), zoneB}) !=
          connectedZones.end())
        continue;

      // Connect the two zones
      connectedZones[{zoneA, zoneB}] = true;
      connectedZones[{zoneB, zoneA}] = true;

      std::vector<AbstractNode> adjacentNodes;
      adjacentNodes.push_back(abstractNodes[zoneA]);
      adjacentNodes.push_back(abstractNodes[zoneB]);
      std::vector<Edge> adjacentBaseEdges;
      bool isPossible = false;
      // check all the edges in the zone
      for (int baseIdx : infoA.baseEdgeIdxs) {
        // For this edge get the from and to zones that they are closest to
        Edge adjacentEdge = baseEdges[baseIdx];
        int toZone = zoneGrid[adjacentEdge.path.back().second]
                             [adjacentEdge.path.back().first]
                                 .closestAbstractNodeIdx;
        int fromZone = zoneGrid[adjacentEdge.path.front().second]
                               [adjacentEdge.path.front().first]
                                   .closestAbstractNodeIdx;
        // If both ends of the edge area closest to the current zone then just
        // add edge to the list
        if (toZone == fromZone) {
          adjacentBaseEdges.push_back(adjacentEdge);
        }
        // If one of the edges is closest to the other zone then add it to the
        // list and remember we can get to other zone
        else if ((toZone == zoneB) || (fromZone == zoneB)) {
          adjacentBaseEdges.push_back(adjacentEdge);
          isPossible = true;
        }
      }
      // If none of the edges connect to the other zone then skip
      if (!isPossible)
        continue;

      // We have at least one edge that connects the zones add all the internal
      // edges of other zone to list
      const ZoneInfo &infoB = zones[zoneB];
      for (int baseIdx : infoB.baseEdgeIdxs) {
        Edge adjacentEdge = baseEdges[baseIdx];
        int toZone = zoneGrid[adjacentEdge.path.back().second]
                             [adjacentEdge.path.back().first]
                                 .closestAbstractNodeIdx;
        int fromZone = zoneGrid[adjacentEdge.path.front().second]
                               [adjacentEdge.path.front().first]
                                   .closestAbstractNodeIdx;
        // An internal edge has both ends closest to the other zone
        if (toZone == fromZone) {
          adjacentBaseEdges.push_back(adjacentEdge);
        }
      }

      LOG_INFO("###### ZONE: a:" << zoneA << " b: " << zoneB
                                 << " try find route: edges: "
                                 << adjacentBaseEdges.size());
      std::vector<AbstractEdge> newEdges =
          AbstractMST::generateMSTAbstractEdges(baseGraph, baseEdges, baseNodes,
                                                adjacentNodes);
      if (newEdges.size() > 1) {
        LOG_ERROR("*********** EEEEK. No new edges ********** ");
      }
      LOG_INFO("###### => GOT " << newEdges.size() << " extra edges");
      for (const auto &newEdge : newEdges) {
        const Edge &fromBase = adjacentBaseEdges[newEdge.from];
        const Edge &toBase = adjacentBaseEdges[newEdge.to];
        LOG_DEBUG(" ***** ADDed zone connect: "
                  << zoneA << "->" << zoneB << "  EDGE:   frombase: "
                  << fromBase.from << "->" << fromBase.to
                  << "  toBase: " << toBase.from << "->" << toBase.to);
        LOG_DEBUG_CONT("       Path sz:" << newEdge.path.size() << "  ");
        LOG_DEBUG_FOR(const auto &p : newEdge.path,
                      p.first << "," << p.second << "  ");
        extraEdges.push_back({zoneA, zoneB, newEdge.path});
      }
    }
  }
  LOG_INFO("##=> EXTRA EDGES: " << extraEdges.size());
  return extraEdges;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<AbstractLevel> makeAbstractLevels(const Graph &graph) {
  std::vector<AbstractLevel> abstractLevels;
  LOG_INFO("## MAKE ABSTRACT LEVELS");
  do {
    int pass = abstractLevels.size();

    LOG_INFO("== MAKE LEVEL: " << pass);
    abstractLevels.push_back({});
    AbstractLevel &ablv = abstractLevels.back();

    // Find the abstract Nodes
    //
    LOG_INFO("   MAKE ABNODEs: " << pass);
    ablv.abstractNodes =
        createAbstractNodes(graph.baseNodes, 5.0, 2 * pass + 2);
    LOG_INFO("   => ABNODEs: " << ablv.abstractNodes.size());
    // Check we actually got some nodes
    if (ablv.abstractNodes.empty()) {
      LOG_INFO("   => NO ABNODEs => BREAK");
      break;
    }
    debugAbstractNodes(pass, ablv, graph);

    // Make Min Span Tree of abstract nodes to get abstract edges
    //
    LOG_INFO("## MAKE ABEDGEs: " << pass);
    ablv.abstractEdges = AbstractMST::generateMSTAbstractEdges(
        graph.baseGraph, graph.baseEdges, graph.baseNodes, ablv.abstractNodes);
    LOG_INFO("   => ABEDGEs: " << ablv.abstractEdges.size());
    debugAbstractEdges(pass, ablv, graph, "ABSTRACT");

    // Generate a zone for each abstract node
    //
    ablv.zones =
        generateAbstractZones(ablv.zoneGrid, graph.infoGrid, graph.baseGraph,
                              graph.baseEdges, ablv.abstractNodes);
    debugZones(pass, ablv, graph);
    debugZoneEdges(pass, ablv, graph);

    generateZoneBoundaries(ablv, graph.infoGrid, graph.baseGraph,
                           graph.baseEdges, graph.baseNodes, graph.deadEnds);
    debugZoneBoundaries(pass, ablv);

    // Fix connections to AbstractNodes
    //
    LOG_INFO("   MAKE EXTRA: " << pass);
    std::vector<AbstractEdge> extraEdges = makeExtraAbstractEdges(
        graph.baseNodes, graph.baseEdges, ablv.abstractNodes,
        ablv.abstractEdges, ablv.zones, ablv.zoneGrid, graph.baseGraph);
    LOG_INFO("   => EXTRA: " << extraEdges.size());
    // Add the extra edges to the abstract edges
    ablv.abstractEdges.insert(ablv.abstractEdges.end(), extraEdges.begin(),
                              extraEdges.end());
    debugAbstractEdges(pass, ablv, graph, "ABEXTRA");

    // Add zone bridge graph
    buildZoneBridgeGraph(ablv);

  } while (abstractLevels.back().zones.size() > 16);

  LOG_INFO("#=> MAKE LEVEL: " << abstractLevels.size() << " zones:"
                              << abstractLevels.back().zones.size());
  return abstractLevels;
}

Graph makeGraph(const Grid &floorGrid) {
  LOG_INFO("## MAKE GRAPH");
  {
    writeFloorGridToFile(floorGrid, "GRID.txt");
  }
  //
  // Need to copy the floorGrid since ZSThinning removes floor
  //
  Graph graph;
  graph.infoGrid = floorGrid;
  // ## DEBUG
  {
    // Invert floor grid so EMPTY is wall pixel
    auto tempGrid = graph.infoGrid;

    for (auto &r : tempGrid) {
      for (auto &c : r) {
        c = (c == EMPTY) ? WALL : EMPTY;
      }
    }
    makeTGA("GRID_INPUT.tga", tempGrid);
  }

  //
  // Thin the floor down to a single path
  //
  Algo::ZSThinning(graph.infoGrid);
  extraThining(graph.infoGrid);
  {
    makeTGA("GRID_THIN.tga", graph.infoGrid, true);
  }

  //
  // Get the deadEnds and Nodes
  //
  graph.deadEnds = detectDeadEnds(graph.infoGrid);
  graph.baseNodes = detectNodes(graph.infoGrid);

  {
    Grid tempGrid(graph.infoGrid.size(),
                  std::vector<int>(graph.infoGrid[0].size(), EMPTY));
    for (int n = 0; n < graph.baseNodes.size(); ++n) {
      int c = graph.baseNodes[n].first;
      int r = graph.baseNodes[n].second;
      tempGrid[r][c] = NODE;
    }
    makeTGA("GRID_NODES_BFR.tga", tempGrid);
  }

  //
  // A 2x2 of path cells onlt gets 2 nodes, but there are cases where
  // this causes problems e.g.
  //
  //    1--x--A
  //       |  |
  //       y--B
  //       |   
  //    2---   
  //
  // findEdges() can creates 1 -> 2, but A can be unconnected and fixBaseEdges()
  // can create 1 -> A. But this means 1->2 and 1->A share cells which is not
  // allowed. Easiest fix I can see is to make x and y NODEs then everthing works.
  //
  std::vector<std::vector<int>> fixNodes = createFixNodes(graph.infoGrid, graph.baseNodes);
  {
    // Debug
    Grid tempGrid(graph.infoGrid.size(),
                  std::vector<int>(graph.infoGrid[0].size(), EMPTY));
    for (int n = 0; n < graph.baseNodes.size(); ++n) {
      int c = graph.baseNodes[n].first;
      int r = graph.baseNodes[n].second;
      tempGrid[r][c] = NODE;
    }
    makeTGA("GRID_NODES_AFT.tga", tempGrid);
  }

  //
  // Set NODE and DEND in infoGrid
  //
  markGridNodes(graph.infoGrid, graph.baseNodes, graph.deadEnds);


  //
  // Find all the BaseEdges connecting all the NODE and DENDs
  // - Need floorGrid to check for cutting corner on diagonal moves
  //
  graph.baseEdges = findEdges(graph.infoGrid, floorGrid);
  
  //
  // Unfortunately there are a few scenarios that dont connect. Basically
  // some places aren't marked as nodes (some areas where "double/triple thick"
  // This allows a path between A and B that bypasses C, but marks the
  // path cells as visited, so C doesnt have a path of A or B e.g.
  //
  //    A--x---B    x isn't marked as a node
  //       |        (real scenario is more messy)
  //       C        So A->B cells visited => no paths from C
  //
  // To fix it have version of findEdges that just checks for paths from the
  // list of nodes. Any new edges are added to the complete list
  //
  // NOTE: This might not be needed because of the fixNodes changes
  // i.e. I think this only happened at 2x2 path cell areas
  //
  graph.baseGraph =
      fixBaseEdges(graph.baseEdges, graph.infoGrid, floorGrid, graph.baseNodes);

  //
  // Detect self-loop PATH structures (A<->A) — orphaned cells never
  // incorporated into any edge because getEdge() rejects from==to.
  //
  std::vector<Edge> loops = detectLoopEdges( graph.infoGrid, graph.baseEdges, graph.baseNodes);

  //
  // Split the loops into 2 edges by adding a new NODE in the middle of the loop
  //
  splitLoopEdges(loops, graph.baseEdges, graph.baseNodes);

  // Detect PATH cells shared by two edge paths (caused by findNodeEdges
  // re-walking cells already claimed by findEdges) and reroute the
  // duplicate edge around the cells owned by the original edge.
  mergeFixNodes(fixNodes, graph.baseNodes, graph.baseEdges, graph.infoGrid);

  //
  // Build the base graph
  // This is a list of (non-dead-end) edges info indexed by node
  //
  graph.baseGraph = GridType::buildBaseGraph(graph.baseEdges, graph.baseNodes.size());

  //
  // Where it simplified the path it will have left the PATH value for infoGrid.
  // Simple fix is to re-write the infoGrid with NODE, DEND, PATH, EMPTY using
  // the created data.
  //
  graph.infoGrid = rewriteInfoGrid(graph.infoGrid, graph.baseEdges);
  {
    makeTGA2("GRID_REWRITE.tga", graph.infoGrid, 0xffff0001);
  }

  //
  // Mark the edges in the infoGrid and their index
  //
  markGridPaths(graph.infoGrid, graph.baseEdges);

  // Validation and debug
  validateEdgeCells(graph.infoGrid, graph.baseEdges);
  debugGridEdges(graph);

  //
  // - Restore Walls (since thinning make FLOOR = EMPTY, then expand Paths to
  // fill EMPTY
  // - Mark the WALLs which are/ next to an empty cell with BOUNDARY
  // - Expand out the paths to mark them as XPND
  //
  restoreWalls(graph.infoGrid, floorGrid);
  markGridBoundaries(graph.infoGrid);
  markWallEscapes(graph.infoGrid);
  expandPaths(graph.infoGrid);

  //
  // Make Wall Distance Grid
  // Construct a specific grid for this, interpreting 0 (EMPTY) in floorGrid as
  // WALL
  //
  LOG_INFO("Making Wall Distance Grid...");
  {
    GridType::Grid wallGrid = floorGrid;
    for (int r = 0; r < wallGrid.size(); ++r) {
      for (int c = 0; c < wallGrid[0].size(); ++c) {
        // Assuming 0 is Wall/Background in floorGrid
        if (wallGrid[r][c] == 0) {
          wallGrid[r][c] = GridType::WALL;
        } else {
          wallGrid[r][c] = 0; // Clear path/walkable to 0 for distance calc
        }
      }
    }
    graph.wallDistanceGrid = makeWallDistanceGrid(wallGrid);
    graph.sightGrid        = makeSightGrid(wallGrid);
  }

  {
    LOG_INFO("============== INFO GRID ==============");
    LOG_DEBUG_FOR(int i = 0; i < graph.infoGrid[0].size();
                  ++i, "   " << (i % 10));
    int r = 0;
    for (const auto &row : graph.infoGrid) {
      for (int lp = 0; lp < 3; ++lp) {
        LOG_DEBUG_CONT((r % 10) << " ");
        for (const auto &cell : row) {
          std::string c = " 0 ";
          if (cell & NODE)
            c = "NNN";
          else if (cell & DEND)
            c = "ddd";
          else if (cell & EDGE)
            c = "---";
          else if (cell & WALL)
            c = "###";
          else if (cell & XPND)
            c = "xxx";
          if (c != "xxx") {
            LOG_DEBUG_CONT(std::hex << c << std::dec);
          } else if (lp == 1) {
            LOG_DEBUG_CONT(get_XPND_DIR(cell) << ":" << get_XPND_DIST(cell));
          } else {
            LOG_DEBUG_CONT("   ");
          }
          LOG_DEBUG_CONT(" ");
        }
        LOG_DEBUG("");
      }
      LOG_DEBUG("");
      ++r;
    }
    LOG_DEBUG("#############");
  }
  debugExpandedPaths(graph.infoGrid);

  //
  // Generate the abstract levels with abstract nodes and edges, zones
  //
  LOG_INFO("===MAKE ABSTRACT");
  graph.abstractLevels = makeAbstractLevels(graph);

  //
  // Make the subgrid flow fields
  //
  LOG_INFO("===MAKE FLOW");
  FlowField::generateFlowGrids(graph);

  //
  //
  //
  LOG_INFO("==MAKE PATHS");
  computeAllPathDists(graph.baseEdges, graph.baseNodes.size());

  //
  // Pre-compute routing graph info
  //
  LOG_INFO("==ROUTING GRAPH");
  graph.routingGraph = Routing::buildSparseGraph(
      graph.baseNodes, graph.deadEnds, graph.baseEdges, graph.infoGrid);

  LOG_INFO("==ABSTRACT GRAPH CONNECTIVITY");
  Routing::buildAbstractConnectivity(graph.routingGraph, graph.abstractLevels,
                                     graph.infoGrid, graph.baseNodes);

  LOG_INFO("## ======= GRAPH MADE =====");
  debugDump(graph);
  return graph;
}

///////////////////////////////////////////////////////////

const char *makeDebugName(int pass, const char *name) {
  static char fname[256];
  sprintf(fname, "GRID_%d_%s.tga", pass, name);
  return fname;
}

#ifndef NO_DIST_GRAPH_DEBUG
void debugGridEdges(const Graph &graph) {
  std::vector<int> uncon =
      checkConnectivity(graph.baseGraph, graph.baseNodes.size());
  Grid tempGrid(graph.infoGrid.size(),
                std::vector<int>(graph.infoGrid[0].size(), EMPTY));
  for (const auto &e : graph.baseEdges) {
    for (int i = 1; i < e.path.size() - 1; ++i) {
      int c = e.path[i].first;
      int r = e.path[i].second;
      tempGrid[r][c] = PATH;
    }
    int c = e.path[0].first;
    int r = e.path[0].second;
    tempGrid[r][c] =
        (std::find(uncon.begin(), uncon.end(), e.from) == uncon.end())
            ? NODE
            : XPND << 2;
    c = e.path[e.path.size() - 1].first;
    r = e.path[e.path.size() - 1].second;
    tempGrid[r][c] =
        (e.toDeadEnd) ? DEND
        : (std::find(uncon.begin(), uncon.end(), e.from) == uncon.end())
            ? NODE
            : XPND << 2;
  }
  makeTGA2("GRID_EDGES.tga", tempGrid, 0xffffffff);
}

void debugAbstractNodes(int pass, const AbstractLevel &ablv,
                        const Graph &graph) {
  Grid tempGrid(graph.infoGrid.size(),
                std::vector<int>(graph.infoGrid[0].size(), EMPTY));
  for (const auto &e : graph.baseEdges) {
    for (int i = 1; i < e.path.size() - 1; ++i) {
      int c = e.path[i].first;
      int r = e.path[i].second;
      tempGrid[r][c] = XPND << 1;
    }
    if (e.path.size() > 1) {
      int c = e.path[0].first;
      int r = e.path[0].second;
      tempGrid[r][c] = NODE;
      c = e.path[e.path.size() - 1].first;
      r = e.path[e.path.size() - 1].second;
      tempGrid[r][c] = NODE;
    }
  }
  for (int n = 0; n < graph.baseNodes.size(); ++n) {
    int c = graph.baseNodes[n].first;
    int r = graph.baseNodes[n].second;
    tempGrid[r][c] = NODE;
  }
  for (const auto &n : ablv.abstractNodes) {
    int c = graph.baseNodes[n.baseCenterNode].first;
    int r = graph.baseNodes[n.baseCenterNode].second;
    LOG_DEBUG(c << "," << r << ")");
    tempGrid[r][c] = XPND << 2;
  }
  makeTGA(makeDebugName(pass, "ALL_NODES"), tempGrid);
}

void debugAbstractEdges(int pass, const AbstractLevel &ablv, const Graph &graph,
                        const char *fname) {
  Grid tempGrid(graph.infoGrid.size(),
                std::vector<int>(graph.infoGrid[0].size(), EMPTY));

  for (int idx = 0; idx < ablv.abstractEdges.size(); ++idx) {
    const auto &e = ablv.abstractEdges.at(idx);
    for (int i = 1; i < e.path.size() - 1; ++i) {
      int c = e.path[i].first;
      int r = e.path[i].second;
      if (tempGrid[r][c] != (XPND << 2))
        tempGrid[r][c] = XPND << 1;
    }
    int c = e.path[e.path.size() - 1].first;
    int r = e.path[e.path.size() - 1].second;
    tempGrid[r][c] = XPND << 2;
    c = e.path[0].first;
    r = e.path[0].second;
    tempGrid[r][c] = XPND << 2;
    LOG_DEBUG("## ABEDGE " << idx << "   ab:" << e.from << " -> " << e.to
                           << " (base: "
                           << ablv.abstractNodes[e.from].baseCenterNode
                           << " -> " << ablv.abstractNodes[e.to].baseCenterNode
                           << ")");
  }
  LOG_INFO("BASENODE: " << graph.baseNodes.size());
  LOG_INFO("BASEEDGE: " << graph.baseEdges.size());
  LOG_INFO("ABNODE: " << ablv.abstractNodes.size());
  LOG_INFO("ABEDGE: " << ablv.abstractEdges.size());
  makeTGA(makeDebugName(pass, fname), tempGrid);
}

void debugZones(int pass, const AbstractLevel &ablv, const Graph &graph) {
  LOG_INFO("DEBUG ZONES");
  int i = 0;
  for (const auto &zi : ablv.zones) {
    LOG_DEBUG_CONT("Level: " << i << " => zones: ");
    LOG_DEBUG_FOR(const auto &n : zi.adjacentZones, n << " ");
    LOG_DEBUG_CONT("         Nodes: ");
    LOG_DEBUG_FOR(const auto &n : zi.baseNodeIdxs, n << " ");
    LOG_DEBUG_CONT("         Edges: ");
    LOG_DEBUG_FOR(const auto &n : zi.baseEdgeIdxs, n << " ");
    LOG_DEBUG("");
    ++i;
  }
  std::vector<int> vals = {
      GridToGraph::NODE,
      GridToGraph::DEND,
      (GridToGraph::XPND << 1),
      (GridToGraph::XPND << 2),
      (GridToGraph::XPND << 3),
      (GridToGraph::XPND << 2) | (XPND << 1),
      (GridToGraph::XPND << 3) | (XPND << 1),
      (GridToGraph::XPND << 3) | (XPND << 2),
      (GridToGraph::XPND << 3) | (XPND << 2) | (XPND << 1),
      GridToGraph::WALL,
      GridToGraph::XPND,
      GridToGraph::BOUNDARY | GridToGraph::WALL,
  };

  int cols = graph.infoGrid[0].size();
  int rows = graph.infoGrid.size();
  Grid tempGrid(rows, std::vector<int>(cols, EMPTY));
  for (int row = 0; row < graph.infoGrid.size(); ++row) {
    for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
      int zn = ablv.zoneGrid[row][col].closestAbstractNodeIdx;
      if (zn != -1) {
        int val = vals[zn % vals.size()];
        tempGrid[row][col] = val;
      }
    }
  }
  makeTGA(makeDebugName(pass, "ZONES"), tempGrid);
}

void debugZoneBoundaries(int pass, const AbstractLevel &ablv) {
  int cols = ablv.zoneGrid[0].size();
  int rows = ablv.zoneGrid.size();
  Grid tempGrid(rows, std::vector<int>(cols, EMPTY));
  for (const auto &zone : ablv.zones) {
    for (const auto &zoneBoundary : zone.zoneBoundaryCellMap) {
      const auto &slots = zoneBoundary.second;
      for (const auto &slot : slots) {
        for (const auto &boundary : slot) {
          int c = boundary.sink.first;
          int r = boundary.sink.second;
          tempGrid[r][c] = WALL;
        }
      }
    }
  }
  makeTGA(makeDebugName(pass, "BCELLS"), tempGrid);
}

void debugDump(const Graph &graph) {
  Grid tempGrid(graph.infoGrid);

  int l = 0;
  LOG_INFO("DEBUG DUMP");
  LOG_DEBUG("############ LEVELS ");
  for (const auto &ablv : graph.abstractLevels) {
    int z = 0;
    for (const auto &srcZoneInfo : ablv.zones) {
      LOG_DEBUG_CONT("ablv: " << l << " z: " << z << " b: ");
      LOG_DEBUG_FOR(int b : srcZoneInfo.baseNodeIdxs, " " << b);
      LOG_DEBUG_CONT("ablv: " << l << " z: " << z << " e: ");
      LOG_DEBUG_FOR(int e : srcZoneInfo.baseEdgeIdxs, " " << e);
      ++z;
    }
    ++l;
  }

  LOG_DEBUG("############ NODES");
  for (int i = 0; i < graph.baseNodes.size(); ++i) {
    const auto &n = graph.baseNodes[i];
    LOG_DEBUG(i << "  " << n.first << "," << n.second);
  }
  LOG_DEBUG("############ EDGES");
  for (int i = 0; i < graph.baseEdges.size(); ++i) {
    const auto &e = graph.baseEdges[i];
    LOG_DEBUG_CONT(i << "  " << e.from << "->" << e.to
                     << (e.toDeadEnd ? " DE" : " "));
    LOG_DEBUG_FOR(const auto p : e.path,
                  "  " << p.first << "," << p.second << " ");
  }
  int lv = 0;
  for (const auto &ablv : graph.abstractLevels) {
    LOG_DEBUG("########################################################");
    LOG_DEBUG("## ABSTRACT LEVEL " << lv);
    LOG_DEBUG("Abstract Nodes:  " << ablv.abstractNodes.size());
    int idx = 0;
    for (const auto &node : ablv.abstractNodes) {
      LOG_DEBUG(" " << idx << " Center: (" << node.center.first << ", "
                    << node.center.second << ")"
                    << " nodesSz: " << node.baseNodes.size()
                    << " baseIdx:" << node.baseCenterNode << " ("
                    << graph.baseNodes[node.baseCenterNode].first << ","
                    << graph.baseNodes[node.baseCenterNode].second << ")");
      ++idx;
    }

    LOG_DEBUG("");
    LOG_DEBUG("Abstract Edges: " << ablv.abstractEdges.size());
    for (const auto &edge : ablv.abstractEdges) {
      LOG_DEBUG("Edge from Cluster " << edge.from << " to Cluster " << edge.to
                                     << " with cost " << edge.path.size());
    }
    for (const auto &e : ablv.abstractEdges) {
      LOG_DEBUG("  ABEDGE: " << e.from << " to " << e.to
                             << " p: " << e.path.size());
      const auto &f = ablv.abstractNodes[e.from];
      const auto &t = ablv.abstractNodes[e.to];
      LOG_DEBUG("    CENTER: " << f.center.first << "," << f.center.second
                               << " to: " << t.center.first << ","
                               << t.center.second);
      LOG_DEBUG("    BASE F:" << f.baseCenterNode << " FROM: "
                              << graph.baseNodes[f.baseCenterNode].first << ","
                              << graph.baseNodes[f.baseCenterNode].second);
      LOG_DEBUG("    BASE T:" << t.baseCenterNode << " TO  : "
                              << graph.baseNodes[t.baseCenterNode].first << ","
                              << graph.baseNodes[t.baseCenterNode].second);
      for (const auto &p : e.path) {
        if (tempGrid[p.second][p.first] == (GridToGraph::XPND << 2))
          continue;
        tempGrid[p.second][p.first] = GridToGraph::XPND << 1;
        LOG_DEBUG("      " << p.first << "," << p.second);
      }
      if (f.baseCenterNode != -1) {
        tempGrid[graph.baseNodes[f.baseCenterNode].second]
                [graph.baseNodes[f.baseCenterNode].first] =
                    GridToGraph::XPND << 2;
        LOG_DEBUG("##SET F " << graph.baseNodes[f.baseCenterNode].first << ","
                             << graph.baseNodes[f.baseCenterNode].second);
      }
      if (t.baseCenterNode != -1) {
        tempGrid[graph.baseNodes[t.baseCenterNode].second]
                [graph.baseNodes[t.baseCenterNode].first] =
                    GridToGraph::XPND << 2;
        LOG_DEBUG("##SET F" << graph.baseNodes[t.baseCenterNode].first << ","
                            << graph.baseNodes[t.baseCenterNode].second);
      }
    }
    LOG_DEBUG("");
    LOG_DEBUG("Base Nodes:  " << graph.baseNodes.size());
    int i = 0;
    for (const auto &node : graph.baseNodes) {
      LOG_DEBUG("  " << i++ << " " << node.first << "," << node.second);
    }

    LOG_DEBUG("NODES: " << graph.baseNodes.size());
    LOG_DEBUG("DENDS: " << graph.deadEnds.size());
    LOG_DEBUG("EDGES: " << graph.baseEdges.size());
    LOG_DEBUG("ABNOD: " << ablv.abstractNodes.size());
    LOG_DEBUG("ABEDG: " << ablv.abstractEdges.size());

    makeTGA("GRID_FULL.tga", tempGrid);
    for (int i = 0; i < graph.baseEdges.size(); ++i) {
      const Edge &e = graph.baseEdges[i];
      LOG_DEBUG("Path: " << i << " F:" << e.from << " T:" << e.to
                         << (e.toDeadEnd ? " DEAD" : ""));
      if (e.toDeadEnd)
        continue;
      LOG_DEBUG_FOR(const auto &p : e.path, "  " << p.first << "," << p.second);
    }

    for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
      if (col > 9)
        LOG_DEBUG(col << "  ");
      else
        LOG_DEBUG(col << "   ");
    }
    LOG_DEBUG("");
    LOG_DEBUG("INFO GRID (nodes/edges)");
    for (int row = 0; row < graph.infoGrid.size(); ++row) {
      if (row > 9)
        LOG_DEBUG(row << "  ");
      else
        LOG_DEBUG(row << "   ");

      for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
        int n = graph.infoGrid[row][col];
        if (n & NODE) {
          n &= 0xffff;
          if (n >= 10) {
            LOG_DEBUG_CONT(n << "n ");
          } else if (n >= 0) {
            LOG_DEBUG_CONT(n << "n  ");
          } else {
            LOG_DEBUG_CONT("--- ");
          }
        } else if (n & EDGE) {
          int e = n & GridType::EDGE_MASK;
          if (e >= 100) {
            LOG_DEBUG_CONT(e << "e");
          } else if (e >= 10) {
            LOG_DEBUG_CONT(e << "e ");
          } else if (e >= 0) {
            LOG_DEBUG_CONT(e << "e  ");
          } else {
            LOG_DEBUG_CONT("--- ");
          }
        } else if (n & XPND) {
          int c = n & 0xffff;
          if (c >= 100) {
            LOG_DEBUG_CONT(c << "c");
          } else if (c >= 10) {
            LOG_DEBUG_CONT(c << "c ");
          } else if (c >= 0) {
            LOG_DEBUG_CONT(c << "c  ");
          } else {
            LOG_DEBUG_CONT("--- ");
          }
        } else {
          LOG_DEBUG_CONT("--- ");
        }
      }
      LOG_DEBUG(" #");
    }

    bool toggle = true;
    LOG_DEBUG("");
    LOG_DEBUG("NAV GRID (closet absNode/baseNode)");
    LOG_DEBUG_CONT("    ");
    for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
      if (col > 9)
        LOG_DEBUG_CONT(col << "   ");
      else
        LOG_DEBUG_CONT(col << "    ");
    }
    LOG_DEBUG("");
    for (int row = 0; row < graph.infoGrid.size();) {
      if (toggle) {
        if (row > 9)
          LOG_DEBUG_CONT(row << "  ");
        else
          LOG_DEBUG_CONT(row << "   ");
        for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
          if (graph.infoGrid[row][col] & WALL) {
            LOG_DEBUG_CONT("     ");
            continue;
          }
          Point p = {col, row};
          auto it =
              std::find(graph.baseNodes.begin(), graph.baseNodes.end(), p);
          if (it == graph.baseNodes.end()) {
            LOG_DEBUG_CONT("     ");
            continue;
          }

          int baseIdx = std::distance(graph.baseNodes.begin(), it);
          if (baseIdx >= 10) {
            LOG_DEBUG_CONT(baseIdx);
          } else {
            LOG_DEBUG_CONT(baseIdx << " ");
          }
          int abIdx = 0;
          for (AbstractNode abNod : ablv.abstractNodes) {
            if (abNod.baseCenterNode == baseIdx) {
              if (abIdx >= 10) {
                LOG_DEBUG_CONT("/" << abIdx);
              } else {
                LOG_DEBUG_CONT("/" << abIdx << " ");
              }
              abIdx = -1;
              break;
            }
            ++abIdx;
          }
          if (abIdx != -1) {
            LOG_DEBUG_CONT("   ");
          }
        }
        LOG_DEBUG("");
        toggle = false;
        continue;
      }
      LOG_DEBUG_CONT("    ");

      toggle = true;

      ++row;
    }

    LOG_DEBUG("");
    LOG_DEBUG("ABSTRACT NODES (abnod / base center)");
    for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
      if (col > 9)
        LOG_DEBUG_CONT(col << "   ");
      else
        LOG_DEBUG_CONT(col << "    ");
    }
    for (int row = 0; row < graph.infoGrid.size(); ++row) {
      if (row > 9)
        LOG_DEBUG(row << "   ");
      else
        LOG_DEBUG(row << "    ");

      for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
        bool found = false;
        int nidx = 0;
        for (const auto &n : ablv.abstractNodes) {
          const auto &p = graph.baseNodes[n.baseCenterNode];
          if (p.first == col && p.second == row) {
            int b = n.baseCenterNode;
            if (b >= 10) {
              LOG_DEBUG_CONT(nidx << "/" << b << " ");
            } else {
              LOG_DEBUG_CONT(nidx << "/" << b << "  ");
            }
            found = true;
            break;
          }
          ++nidx;
        }
        if (!found) {
          int eidx = 0;
          for (const auto &e : ablv.abstractEdges) {
            for (const auto &p : e.path) {
              if (p.first == col && p.second == row) {
                if (eidx >= 10) {
                  LOG_DEBUG_CONT(eidx << "e  ");
                } else {
                  LOG_DEBUG_CONT(eidx << "e   ");
                }
                found = true;
                break;
              }
            }
            if (found)
              break;
            ++eidx;
          }
        }
        if (!found) {
          LOG_DEBUG_CONT("---- ");
        }
      }
      LOG_DEBUG(" #");
    }

    LOG_DEBUG("WALLS");
    for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
      if (col % 10) {
        LOG_DEBUG_CONT(" ");
      } else {
        LOG_DEBUG_CONT((col / 10));
      }
    }
    LOG_DEBUG("");
    LOG_DEBUG_FOR(int col = 0; col < graph.infoGrid[0].size();
                  ++col, (col % 10));
    for (int row = 0; row < graph.infoGrid.size(); ++row) {
      for (int col = 0; col < graph.infoGrid[0].size(); ++col) {
        LOG_DEBUG_CONT(((graph.infoGrid[row][col] & WALL) ? "#" : " "));
      }
      LOG_DEBUG(" " << row);
    }
    ++lv;
  }
}
#endif

#ifndef NO_DIST_GRAPH_DEBUG
void writeFloorGridToFile(const std::vector<std::vector<int>> &grid,
                          const std::string &filename) {
  std::ofstream outFile(filename);
  if (!outFile) {
    throw std::runtime_error("Could not open file for writing");
  }

  for (const auto &row : grid) {
    for (int cell : row) {
      outFile << (cell == GridToGraph::EMPTY ? '#' : ' ');
    }
    outFile << std::endl;
  }
}
#endif

std::vector<std::vector<int>>
gridToFloorGrid(const std::vector<std::vector<int>> &grid) {
  std::vector<std::vector<int>> floorGrid;
  for (const auto &row : grid) {
    std::vector<int> floorRow;
    for (int cell : row) {
      floorRow.push_back(cell ? GridToGraph::EMPTY : GridToGraph::PATH);
    }
    floorGrid.push_back(floorRow);
  }
  return floorGrid;
}

std::vector<std::vector<int>> readGridFromFile(const std::string &filename) {
  std::ifstream inFile(filename);
  if (!inFile) {
    throw std::runtime_error("Could not open file for reading");
  }

  std::vector<std::vector<int>> grid;
  std::string line;

  while (std::getline(inFile, line)) {
    std::vector<int> row;
    for (char c : line) {
      if (c == '#') {
        row.push_back(1);
      } else if (c == ' ') {
        row.push_back(0);
      }
      // Note: other characters are ignored
    }
    if (!row.empty()) {
      grid.push_back(row);
    }
  }

  return grid;
}

#ifndef NO_DIST_GRAPH_DEBUG
void debugZoneEdges(int pass, const AbstractLevel &ablv, const Graph &graph) {
  LOG_INFO("## ZONE EDGES DETAILED DUMP");
  LOG_INFO("================================================");

  // First, show all edges and which zones they should belong to
  LOG_DEBUG("ALL EDGES AND THEIR ZONE SPANS:");
  for (int edgeIdx = 0; edgeIdx < graph.baseEdges.size(); ++edgeIdx) {
    const auto &edge = graph.baseEdges[edgeIdx];
    GridType::Point from = edge.path.front();
    GridType::Point to = edge.path.back();
    int fromZone =
        ablv.zoneGrid[from.second][from.first].closestAbstractNodeIdx;
    int toZone = ablv.zoneGrid[to.second][to.first].closestAbstractNodeIdx;

    LOG_DEBUG("Edge " << edgeIdx << ": (" << from.first << "," << from.second
                      << ") -> (" << to.first << "," << to.second
                      << ") spans zones " << fromZone << " -> " << toZone
                      << ((fromZone != toZone) ? " (CROSSES ZONES)" : ""));
  }

  LOG_DEBUG("");
  LOG_DEBUG("ZONE CONTENTS:");
  for (int zoneIdx = 0; zoneIdx < ablv.zones.size(); ++zoneIdx) {
    const auto &zone = ablv.zones[zoneIdx];
    LOG_DEBUG("Zone " << zoneIdx << ":");
    LOG_DEBUG_CONT("  Base Nodes (" << zone.baseNodeIdxs.size() << "): ");
    LOG_DEBUG_FOR(const auto &nodeIdx : zone.baseNodeIdxs, nodeIdx << " ");

    LOG_DEBUG_CONT("  Base Edges (" << zone.baseEdgeIdxs.size() << "): ");
    LOG_DEBUG_FOR(const auto &edgeIdx : zone.baseEdgeIdxs, edgeIdx << " ");

    LOG_DEBUG_CONT("  Adjacent Zones: ");
    LOG_DEBUG_FOR(const auto &adjZone : zone.adjacentZones, adjZone << " ");

    // Show which edges should be in this zone but aren't
    LOG_DEBUG_CONT("  MISSING EDGES (should be here but aren't): ");
    for (int edgeIdx = 0; edgeIdx < graph.baseEdges.size(); ++edgeIdx) {
      const auto &edge = graph.baseEdges[edgeIdx];
      GridType::Point from = edge.path.front();
      GridType::Point to = edge.path.back();
      int fromZone =
          ablv.zoneGrid[from.second][from.first].closestAbstractNodeIdx;
      int toZone = ablv.zoneGrid[to.second][to.first].closestAbstractNodeIdx;

      // Edge should be in this zone if either end is in this zone
      bool shouldBeHere = (fromZone == zoneIdx || toZone == zoneIdx);
      bool isHere =
          (std::find(zone.baseEdgeIdxs.begin(), zone.baseEdgeIdxs.end(),
                     edgeIdx) != zone.baseEdgeIdxs.end());

      if (shouldBeHere && !isHere) {
        LOG_DEBUG_CONT(edgeIdx << " ");
      }
    }
    LOG_DEBUG("");
    LOG_DEBUG("---");
  }
}
#endif

#ifndef NO_DIST_GRAPH_DEBUG
void debugExpandedPaths(const Grid &infoGrid) {
  for (int r = 0; r < infoGrid.size(); ++r) {
    for (int c = 0; c < infoGrid[0].size(); ++c) {
      int cell = infoGrid[r][c];
      if (!(cell & XPND))
        continue;
      int x = c;
      int y = r;
      do {
        int dir = get_XPND_DIR(cell);
        int dst = get_XPND_DIST(cell);
        for (int d = 0; d < dst; ++d) {
          x += directions8[dir].first;
          y += directions8[dir].second;
          cell = infoGrid[y][x];
          if (cell & WALL) {
            LOG_DEBUG("ERROR: From " << c << "," << r << " moving in " << dir
                                     << " for " << dst << ". When at " << x
                                     << "," << y << " (dst:" << d + 1
                                     << ") hit WALL");
          }
        }
      } while (cell & XPND);
    }
  }
}
#endif
} // namespace GridToGraph
} // namespace DistanceMap
