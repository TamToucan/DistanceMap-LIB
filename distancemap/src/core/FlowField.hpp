#ifndef DISTANCEMAP_SRC_FLOWFIELD_HPP_
#define DISTANCEMAP_SRC_FLOWFIELD_HPP_

#include <cstdint>

#include "Debug.h"

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
  std::vector<int> grid;
  // List for { adjacent zone, flow field} pairs
  std::vector<std::pair<int, std::vector<uint16_t>>> costFlowFields;

  int width;
  int height;
  int offsetX; // Global x coordinate of the subgrids top-left cell.
  int offsetY; // Global y coordinate of the subgrids top-left cell.
  bool isInside(int x, int y) const {
    return (x >= 0 && x < width && y >= 0 && y < height);
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

  // The flow fieild has the code of movement towards the nearest sink cell
  // If the SINK_BIT is set then the cell is the boundary cell with the
  // direction to cross into the next subgrid.
  //
  // get the cost and direction from the flow field:
  // 8bit cost> | <sinkBit> <direction Index>
  inline uint16_t getCostFlow(int x, int y,
                              const std::vector<uint16_t> &flow) const {
    return flow[indexFor(x - offsetX, y - offsetY, width)];
  }
  // Using a flat index for arrays.
  static inline int indexFor(int x, int y, int cols) { return y * cols + x; }
};

void generateFlowGrids(GridToGraph::Graph &graph);

} // namespace FlowField
} // namespace DistanceMap

#endif /* DISTANCEMAP_SRC_FLOWFIELD_HPP_ */
