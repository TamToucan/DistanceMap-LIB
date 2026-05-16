#ifndef SPARSE_NAV_GRAPH_HPP
#define SPARSE_NAV_GRAPH_HPP

/**
 * @file SparseNavGraph.hpp
 * @brief Sparse base+abstract graph used for A* routing.
 * @details Stores adjacency in compact "node -> [(other, edgeIdx), ...]" form,
 * plus an edge-grid that maps any cell to the nearest base edge (for snapping
 * agent positions). Supports same-zone, adjacent-zone and distant (multi-zone
 * via boundary waypoints) queries. Build via buildSparseGraph(); attach
 * abstract level connectivity with buildAbstractConnectivity().
 */

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
namespace GridToGraph {
struct AbstractLevel;
}
} // namespace DistanceMap

namespace DistanceMap {
namespace Routing {

/**
 * @class SparseNavGraph
 * @brief Compact adjacency + grid lookup used by A* and zone-aware routing.
 */
class DISTANCEMAP_API SparseNavGraph {
public:
  // Base graph adjacency (not deadEnds)
  std::vector<std::vector<std::pair<int, int>>>
      forwardConnections; // node -> [ { otherNode, edge_idx },... ]
  std::vector<std::vector<std::pair<int, int>>>
      reverseConnections; // otherNode -> [ { node, edge_idx },... ]
  std::unordered_map<int, std::pair<int, int>>
      deadendConnection; // deadNode -> { node , edge_idx }

  // Edge-node mappings
  std::vector<std::vector<int>> nodeToEdgeIdxs;
  std::vector<int> edgeCosts;
  std::vector<std::pair<int, int>>
      edgeFromTos; // { from, to (to == -1) for deadEnds }

  // Spatial information
  std::vector<GridType::Point> nodePoints;
  MathStuff::Grid2D<uint32_t> edgeGrid; // <16 dst along path of closest edge>
                                        // |1=past halfway | <15 edge idx>

  // Hierarchical information (optional)
  std::vector<int> nodeToZone;               // baseNode -> zoneId
  std::vector<std::vector<int>> zoneToNodes; // zoneId -> [baseNodes]
  std::vector<std::vector<int>> zoneToEdges; // zoneId -> [baseEdges]

  // Abstract graph information
  std::vector<std::vector<std::vector<std::pair<int, int>>>>
      abstractForwardConnections; // levelIdx -> [ abstractNodeIdx -> [ {
                                  // otherAbstractNodeIdx, abstractEdgeIdx },...
                                  // ] ]
  std::vector<MathStuff::Grid2D<uint8_t>>
      abstractFlowGrids; // levelIdx -> distance map storing direction to
                         // nearest abstractNode

  int medianEdgeCost;

public:
  SparseNavGraph();
  ~SparseNavGraph() = default;

  /// Returns the edge index connecting n1 and n2, or -1 if none.
  int getEdgeFromNodes(int n1, int n2) const;
  /// Returns the edge index for the given dead-end index, or -1.
  int getEdgeFromDead(int deadIdx) const;

  // Helper methods
  /// Returns the zoneId a base node belongs to.
  int getNodeZone(int nodeIdx) const;
  /// Returns the list of baseNode indices that live inside the given zone.
  const std::vector<int> &getZoneNodes(int zoneId) const;
  /// Returns the list of baseEdge indices that live inside the given zone.
  const std::vector<int> &getZoneEdges(int zoneId) const;

  // Routing methods
  /**
   * @brief Compute a route from a source (node or edge) to a target node.
   * @param ctx               Per-agent route cache (last route / route-type
   *                          fingerprint) reused across calls.
   * @param sourceNodeIdx     Pass either this or sourceEdgeIdx.
   * @param sourceEdgeIdx     Pass either this or sourceNodeIdx.
   * @param targetNodeIdx     Destination base node.
   * @param sourceZoneId      Zone of the source (precomputed by caller).
   * @param targetZoneId      Zone of the target (precomputed by caller).
   * @param zoneRelationship  -1 = same zone, >=0 = adjacent (index into
   *                          source zone's adjacency list), -2 = distant
   *                          (route via boundaries).
   * @param costBias          Adds randomness to edge cost for path variety.
   * @param maxPerturbation   Caps the randomness added by costBias.
   * @return Path as a list of base-graph node indices (empty on failure).
   */
  std::vector<int>
  findRoute(Router::RouteCtx *ctx, const std::vector<GridType::ZoneInfo> &zones,
            const std::vector<GridType::AbstractEdge> &abstractEdges,
            std::optional<int> sourceNodeIdx, std::optional<int> sourceEdgeIdx,
            int targetNodeIdx,
            int sourceZoneId,     // Precomputed source zone
            int targetZoneId,     // Precomputed target zone
            int zoneRelationship, // -1 = same zone, >=0 = adjacent zone index, -2 = distant
            int costBias = 0,
            int maxPerturbation = 15)
      const;

private:
  /// Bidirectional A* over the sparse graph, restricted to allowed sets.
  /// Source and target can each be either a node or an edge.
  std::vector<int> bidirectionalAStarFlexible(
      std::optional<int> sourceNodeIdx, std::optional<int> sourceEdgeIdx,
      std::optional<int> targetNodeIdx, std::optional<int> targetEdgeIdx,
      const std::unordered_set<int> &allowedEdges,
      const std::unordered_set<int> &allowedNodes,
      int costBias = 0,
      int maxPerturbation = 15) const;

  /// Turn a list of edge indices into the corresponding node-index path,
  /// handling the special case where the start is an edge midpoint.
  std::vector<int>
  convertEdgesToNodePath(const std::vector<int> &edgePath,
                         std::optional<int> sourceEdgeIdx,
                         std::optional<int> sourceNodeIdx) const;

  /// Same-zone node->node A*, restricted to the zone's nodes/edges.
  std::vector<int> findZoneNodeToNodePath(Router::RouteCtx *ctx,
                                          const std::vector<int> &zoneBases,
                                          const std::vector<int> &zoneEdges,
                                          int sourceNodeIdx,
                                          int targetNodeIdx,
                                          int costBias = 0,
                                          int maxPerturbation = 15) const;

  /// Same-zone edge->node A*, restricted to the zone's nodes/edges.
  std::vector<int> findZoneEdgeToNodePath(Router::RouteCtx *ctx,
                                          const std::vector<int> &zoneBases,
                                          const std::vector<int> &zoneEdges,
                                          int sourceEdgeIdx,
                                          int targetNodeIdx,
                                          int costBias = 0,
                                          int maxPerturbation = 15) const;

  /// Shared A* helper that takes pre-built allowed-node/edge sets.
  std::vector<int> findRouteWithAllowedSets(
      Router::RouteCtx *ctx, const std::vector<int> &allowedNodes,
      const std::vector<int> &allowedEdges, std::optional<int> sourceNodeIdx,
      std::optional<int> sourceEdgeIdx, int targetNodeIdx,
      int costBias = 0,
      int maxPerturbation = 15) const;

  // Returns the node in candidateNodes closest (Manhattan) to boundarySink.
  int findClosestNodeToBoundary(
      const GridType::Point &boundarySink,
      const std::vector<int> &candidateNodes) const;

  // Routes src→tgt via zone-boundary waypoints using small per-zone A* calls.
  std::vector<int> findDistantRouteViaBoundaries(
      Router::RouteCtx *ctx,
      const std::vector<GridType::ZoneInfo> &zones,
      const std::vector<int> &zonePath,
      std::optional<int> sourceNodeIdx,
      std::optional<int> sourceEdgeIdx,
      int targetNodeIdx,
      int costBias,
      int maxPerturbation = 15) const;

  /// BFS over zone adjacency. Returns the sequence of zone ids from source
  /// to target (inclusive), or empty if unreachable.
  static std::vector<int>
  findZonePathBFS(const std::vector<GridType::ZoneInfo> &zones,
                  int sourceZoneId, int targetZoneId);
};

/// Construct the base SparseNavGraph from raw node/edge/grid data.
/// Populates forward/reverse connections, dead-end map, node->edge mapping,
/// edge costs, node points, and the edge grid.
SparseNavGraph buildSparseGraph(const std::vector<GridType::Point> &baseNodes,
                                const std::vector<GridType::Point> &deadEnds,
                                const std::vector<GridType::Edge> &baseEdges,
                                const GridType::Grid &infoGrid);

/// Populate the per-level abstract adjacency and flow grids on an existing
/// SparseNavGraph. Call after buildSparseGraph().
void buildAbstractConnectivity(
    SparseNavGraph &graph,
    const std::vector<GridToGraph::AbstractLevel> &abstractLevels,
    const GridType::Grid &infoGrid,
    const std::vector<GridType::Point> &baseNodes);

} // namespace Routing
} // namespace DistanceMap

#endif // SPARSE_NAV_GRAPH_HPP
