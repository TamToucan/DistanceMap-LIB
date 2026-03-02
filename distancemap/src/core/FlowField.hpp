#ifndef DISTANCEMAP_SRC_FLOWFIELD_HPP_
#define DISTANCEMAP_SRC_FLOWFIELD_HPP_

#include <cstdint>

#include "Debug.h"
#include "GridTypes.hpp"
#include <vector>

// Forward Dec
namespace DistanceMap {
namespace GridToGraph {
struct Graph;
}
} // namespace DistanceMap

namespace DistanceMap {
namespace FlowField {

constexpr uint8_t NO_DIR = 127;
constexpr uint8_t SINK_BIT = 0x80;

struct SubGrid {
  // Flat grid array to be used as the input for generateFlowFieldDial.
  // I think this could be cleared after generateFlowFieldDial is done.
  std::vector<int> grid;
  // List for { adjacent zone, flow field} pairs
  std::vector<std::pair<int, std::vector<uint16_t>>> costFlowFields;

  // List of all floor cells in the subgrid (map coordinates)
  std::vector<GridType::Point> floorCells;

  int width;
  int height;
  int offsetX; // Global x coordinate of the subgrids top-left cell.
  int offsetY; // Global y coordinate of the subgrids top-left cell.
  // Internal use uses local grid coords
  bool isInsideLocal(int x, int y) const {
    return (x >= 0 && x < width && y >= 0 && y < height);
  }

  // For client use. Takes map coords
  bool isInside(int x, int y) const {
    return isInsideLocal(x - offsetX, y - offsetY);
  }

  // Find the flowField for the given adjacent zone.
  inline const std::vector<uint16_t> &getFlow(int adjZone) const {
    static const std::vector<uint16_t> emptyFlow = {};
    for (const auto &pair : costFlowFields)
      if (pair.first == adjZone)
        return pair.second;
    LOG_ERROR("getFlow zone:" << adjZone << " not an adjacent zone");
    return emptyFlow;
  }

  // Returns the map coordinates of the floor cell at index idx.
  // Returns {-1, -1} if idx is out of bounds.
  inline GridType::Point getFloor(int idx) const {
    if (idx >= 0 && idx < (int)floorCells.size()) {
      return floorCells[idx];
    }
    return {-1, -1};
  }

  // The flow field has the code of movement towards the nearest sink cell
  // If the SINK_BIT is set then the cell is the boundary cell with the
  // direction to cross into the next subgrid.
  //
  // get the cost and direction from the flow field:
  // 8bit cost> | <sinkBit> <direction Index>
  inline uint16_t getCostFlow(int x, int y,
                              const std::vector<uint16_t> &flow) const {
    return flow[indexForLocal(x - offsetX, y - offsetY, width)];
  }
  // Using a flat index for arrays. Assumed x,y are local coords.
  static inline int indexForLocal(int x, int y, int cols) {
    return y * cols + x;
  }
};

void generateFlowGrids(GridToGraph::Graph &graph);

} // namespace FlowField
} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_FLOWFIELD_HPP_ */
