#ifndef SPARSE_NAV_GRAPH_HPP
#define SPARSE_NAV_GRAPH_HPP

#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "DistanceMapDLL.hpp"
#include "Grid2D.h"
#include "GridTypes.hpp"

// Forward declaration
namespace DistanceMap {
namespace Router {
struct RouteCtx;
}
}  // namespace DistanceMap

namespace DistanceMap {
namespace Routing {

class DISTANCEMAP_API SparseNavGraph {
 public:
  // Base graph adjacency (not deadEnds)
  std::vector<std::vector<std::pair<int, int>>>
      forwardConnections;  // node -> [ { otherNode, edge_idx },... ]
  std::vector<std::vector<std::pair<int, int>>>
      reverseConnections;  // otherNode -> [ { node, edge_idx },... ]
  std::unordered_map<int, std::pair<int, int>>
      deadendConnection;  // deadNode -> { node , edge_idx }

  // Edge-node mappings
  std::vector<std::vector<int>> nodeToEdgeIdxs;
  std::vector<int> edgeCosts;
  std::vector<std::pair<int, int>>
      edgeFromTos;  // { from, to (to == -1) for deadEnds }

  // Spatial information
  std::vector<GridType::Point> nodePoints;
  MathStuff::Grid2D<uint32_t> edgeGrid;  // <16 dst along path of closest edge>
                                         // |1=past halfway | <15 edge idx>

  // Hierarchical information (optional)
  std::vector<int> nodeToZone;                // baseNode -> zoneId
  std::vector<std::vector<int>> zoneToNodes;  // zoneId -> [baseNodes]
  std::vector<std::vector<int>> zoneToEdges;  // zoneId -> [baseEdges]

 public:
  SparseNavGraph();
  ~SparseNavGraph() = default;

  int getEdgeFromNodes(int n1, int n2) const;
  int getEdgeFromDead(int deadIdx) const;

  // Helper methods
  int getNodeZone(int nodeIdx) const;
  const std::vector<int>& getZoneNodes(int zoneId) const;
  const std::vector<int>& getZoneEdges(int zoneId) const;

  // Routing methods
  std::vector<int>
  findRoute(Router::RouteCtx* ctx, const std::vector<GridType::ZoneInfo>& zones,
            const std::vector<GridType::AbstractEdge>& abstractEdges,
            std::optional<int> sourceNodeIdx, std::optional<int> sourceEdgeIdx,
            int targetNodeIdx,
            int sourceZoneId,  // Precomputed source zone
            int targetZoneId,  // Precomputed target zone
            int zoneRelationship)
      const;  // -1 = same zone, >=0 = adjacent zone index, -2 = distant

 private:
  std::vector<int> bidirectionalAStarFlexible(
      std::optional<int> sourceNodeIdx, std::optional<int> sourceEdgeIdx,
      std::optional<int> targetNodeIdx, std::optional<int> targetEdgeIdx,
      const std::unordered_set<int>& allowedEdges,
      const std::unordered_set<int>& allowedNodes) const;

  std::vector<int>
  convertEdgesToNodePath(const std::vector<int>& edgePath,
                         std::optional<int> sourceEdgeIdx,
                         std::optional<int> sourceNodeIdx) const;

  std::vector<int> findZoneNodeToNodePath(Router::RouteCtx* ctx,
                                          const std::vector<int>& zoneBases,
                                          const std::vector<int>& zoneEdges,
                                          int sourceNodeIdx,
                                          int targetNodeIdx) const;

  std::vector<int> findZoneEdgeToNodePath(Router::RouteCtx* ctx,
                                          const std::vector<int>& zoneBases,
                                          const std::vector<int>& zoneEdges,
                                          int sourceEdgeIdx,
                                          int targetNodeIdx) const;

  std::vector<int> findRouteWithAllowedSets(
      Router::RouteCtx* ctx, const std::vector<int>& allowedNodes,
      const std::vector<int>& allowedEdges, std::optional<int> sourceNodeIdx,
      std::optional<int> sourceEdgeIdx, int targetNodeIdx) const;

  static std::vector<int>
  findZonePathBFS(const std::vector<GridType::ZoneInfo>& zones,
                  int sourceZoneId, int targetZoneId);
};

// Builder function
SparseNavGraph buildSparseGraph(const std::vector<GridType::Point>& baseNodes,
                                const std::vector<GridType::Point>& deadEnds,
                                const std::vector<GridType::Edge>& baseEdges,
                                const GridType::Grid& infoGrid);

}  // namespace Routing
}  // namespace DistanceMap

#endif  // SPARSE_NAV_GRAPH_HPP
