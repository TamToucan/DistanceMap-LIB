#include "NavigationGraph.hpp"

#include <algorithm>
#include <cmath>

#include "Debug.h"
#include "DistanceMapCore.hpp"
#include "MathUtils.h"
#include "NavigationAPI.hpp"

namespace DistanceMap {
namespace Routing {

// Frames at the same grid cell before the route is discarded and retried.
static constexpr int STUCK_THRESHOLD = 60;

// Consecutive frames off the skeleton (XPND/WALL/unknown — never touching
// EDGE/NODE/DEND) before Phase A is bypassed and a full re-route is forced.
// This catches oscillation between off-skeleton cells that STUCK_THRESHOLD
// misses because Phase A returns early (before Phase E can run).
static constexpr int NON_SKELETON_STUCK_THRESHOLD = 90;

// Clears the route plan AND the A* route-cache fields inside RouteCtx so that
// the next ensureRoute call actually re-runs A* instead of hitting SAME ROUTE.
static void resetRouteCache(Router::RouteCtx *ctx) {
  ctx->routeNodes.clear();
  ctx->graph.tgtDeadEndEdgeIdx = -1;
  ctx->routeSrcNodeIdx  = -1;
  ctx->routeTgtNodeIdx  = -1;
  ctx->routeSrcNodeIdx2 = -1;
  ctx->routeTgtEdgeIdx  = -1;
  ctx->routeSrcEdgeIdx  = -1;
  ctx->routeTgtNodeIdx2 = -1;
  ctx->routeSrcEdgeIdx2 = -1;
  ctx->routeTgtEdgeIdx2 = -1;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

NavigationGraph::NavigationGraph(const GridToGraph::Graph &graphData,
                                 const Router::Info &info)
    : NavigationAPI(graphData.infoGrid, info), m_baseGraph(graphData.baseGraph),
      m_pathCostMap(graphData.pathCostMap),
      m_routingGraph(graphData.routingGraph), m_baseEdges(graphData.baseEdges),
      m_baseNodes(graphData.baseNodes), m_deadEnds(graphData.deadEnds),
      m_abstractLevels(graphData.abstractLevels) {}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

float NavigationGraph::computeAngle(double dx, double dy) const {
  if (dx == 0 && dy == 0)
    return 0.0f;
  return static_cast<float>(
      std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}

NavigationGraph::CellKind NavigationGraph::classifyCell(int cell) {
  if (cell & GridType::NODE) return CellKind::NODE;
  if (cell & GridType::EDGE) return CellKind::EDGE;
  if (cell & GridType::DEND) return CellKind::DEND;
  if (cell & GridType::XPND) return CellKind::XPND;
  return CellKind::OTHER;
}

const std::vector<GridToGraph::AbstractLevel> *
NavigationGraph::getAbstractLevels() const {
  return &m_abstractLevels;
}

// ---------------------------------------------------------------------------
// getClosestNode
//
// Maps an arbitrary floor cell to the nearest graph node.
// Fixes the original bug where edgeIdx was read once from the initial position
// and then incorrectly reused after XPND traversal moved the working point.
// ---------------------------------------------------------------------------

NavigationGraph::ClosestNodeInfo
NavigationGraph::getClosestNode(const GridType::Point &pos) const {
  GridType::Point workPos = pos;
  int cell = m_infoGrid[workPos.second][workPos.first];

  // Sloped edges can leave the agent on a WALL|BOUNDARY cell.  The BOUNDARY
  // direction (low 3 bits) points one step toward the adjacent walkable cell,
  // so follow it before attempting any graph resolution.
  if ((cell & GridType::WALL) && (cell & GridType::BOUNDARY)) {
    int dirIdx = cell & GridType::DIR_MASK;
    workPos.first  += GridType::directions8[dirIdx].first;
    workPos.second += GridType::directions8[dirIdx].second;
    cell = m_infoGrid[workPos.second][workPos.first];
    LOG_DEBUG("GCN: BOUNDARY|WALL at " << pos.first << "," << pos.second
      << " stepped to " << workPos.first << "," << workPos.second);
  }

  // Already on a node
  if (cell & GridType::NODE) {
    return {cell & 0xffff, -1, false, workPos};
  }

  // Already on an edge
  if (cell & GridType::EDGE) {
    int eIdx = cell & GridType::EDGE_MASK;
    const auto &ft = m_routingGraph.edgeFromTos[eIdx];
    bool secondHalf = (cell & GridType::EDGE_HALF) && (ft.second >= 0);
    return {secondHalf ? ft.second : ft.first, eIdx, true, workPos};
  }

  // Follow XPND pointers until we land on an edge or node
  GridType::Point edgePnt = workPos;
  while (cell & GridType::XPND) {
    int dst = GridType::get_XPND_DIST(cell);
    int dir = GridType::get_XPND_DIR(cell);
    edgePnt.first  += GridType::directions8[dir].first  * dst;
    edgePnt.second += GridType::directions8[dir].second * dst;
    cell = m_infoGrid[edgePnt.second][edgePnt.first];
    LOG_DEBUG("GCN: XPND => " << edgePnt.first << "," << edgePnt.second
      << " dir: " << GridType::directions8[dir].first << "," << GridType::directions8[dir].second
      << " dst: " << dst << " cell: " << std::hex << cell << std::dec);
  }

  if (cell & GridType::EDGE) {
    int eIdx = cell & GridType::EDGE_MASK;
    const auto &ft = m_routingGraph.edgeFromTos[eIdx];
    bool secondHalf = (cell & GridType::EDGE_HALF) && (ft.second >= 0);
    return {secondHalf ? ft.second : ft.first, eIdx, true, edgePnt};
  }

  if (cell & GridType::NODE) {
    return {cell & 0xffff, -1, false, edgePnt};
  }

  if (cell & GridType::DEND) {
    int dendIdx = cell & 0xffff;
    // Re-read edgeIdx from the destination point (not the original pos)
    int eIdx = m_routingGraph.getEdgeFromDead(dendIdx);
    int fromNode = (eIdx >= 0) ? m_routingGraph.edgeFromTos[eIdx].first : -1;
    LOG_DEBUG("GCN: DEND " << dendIdx << " fromNode: " << fromNode);
    return {fromNode, dendIdx, false, edgePnt};
  }

  LOG_ERROR("GCN: Could not resolve pos " << pos.first << "," << pos.second
                                          << " cell: " << std::hex << cell
                                          << std::dec);
  return {-1, -1, false, pos};
}

// ---------------------------------------------------------------------------
// findZoneRelationship
// ---------------------------------------------------------------------------

NavigationGraph::ZoneInfo
NavigationGraph::findZoneRelationship(const GridType::Point &source,
                                      const GridType::Point &target) const {
  ZoneInfo zi;

  // Track the last level where BOTH zones were valid.
  // These are what get used for the distant-routing fallback so they must
  // never come from a level where either zone was -1.
  int lastValidLvl = -1;
  int lastValidSrc = -1;
  int lastValidTgt = -1;

  for (int lvl = 0; lvl < static_cast<int>(m_abstractLevels.size()); ++lvl) {
    const auto &ablv  = m_abstractLevels[lvl];
    int srcZone = ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
    const int tgtZone = ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
    const int nZones  = static_cast<int>(ablv.zones.size());

    // If the source cell has no zone assignment (e.g. a floor cell not covered by
    // XPND expansion, reached via collision/truncation), search adjacent cells for
    // a valid zone so the agent can still route rather than looping forever.
    if (srcZone < 0) {
      const int rows = static_cast<int>(ablv.zoneGrid.size());
      const int cols = (rows > 0) ? static_cast<int>(ablv.zoneGrid[0].size()) : 0;
      for (const auto &d : GridType::directions8) {
        const int nx = source.first  + d.first;
        const int ny = source.second + d.second;
        if (nx >= 0 && ny >= 0 && ny < rows && nx < cols) {
          const int z = ablv.zoneGrid[ny][nx].closestAbstractNodeIdx;
          if (z >= 0 && z < nZones) {
            LOG_DEBUG("NG: findZones lvl " << lvl << " src zone -1 at "
                      << source.first << "," << source.second
                      << "; using neighbor zone " << z
                      << " from " << nx << "," << ny);
            srcZone = z;
            break;
          }
        }
      }
    }

    if (srcZone < 0 || srcZone >= nZones || tgtZone < 0 || tgtZone >= nZones) {
      LOG_DEBUG("NG: findZones lvl " << lvl << " invalid src=" << srcZone
                                     << " tgt=" << tgtZone << "; skipping");
      continue;
    }

    // This level has both zones valid — remember it for the distant fallback
    lastValidLvl = lvl;
    lastValidSrc = srcZone;
    lastValidTgt = tgtZone;

    if (srcZone == tgtZone) {
      zi = {srcZone, tgtZone, lvl, -1};
      return zi;
    }

    for (int adj : ablv.zones[srcZone].adjacentZones) {
      if (adj == tgtZone) {
        zi = {srcZone, tgtZone, lvl, lvl};
        return zi;
      }
    }
  }

  if (lastValidLvl >= 0) {
    // Distant: zones are in different non-adjacent zones; route via highest valid level
    zi = {lastValidSrc, lastValidTgt, lastValidLvl, -2};
  } else {
    // No level had both src and tgt in a valid zone — unroutable
    LOG_INFO("NG: findZones: no valid zone for src="
             << source.first << "," << source.second
             << " tgt=" << target.first << "," << target.second);
    zi.ablvIdx = -1; // sentinel consumed by ensureRoute
  }
  return zi;
}

// ---------------------------------------------------------------------------
// ensureRoute
//
// Builds ctx->routeNodes if empty. Returns true when the route is ready.
// ---------------------------------------------------------------------------

bool NavigationGraph::ensureRoute(Router::RouteCtx *ctx,
                                  const ZoneInfo &zi,
                                  std::optional<int> srcNodeIdx,
                                  std::optional<int> srcEdgeIdx,
                                  int tgtNodeIdx) const {
  if (!ctx->routeNodes.empty())
    return true;

  if (zi.ablvIdx < 0) {
    LOG_INFO("NG: ensureRoute: unroutable (no valid zone); skipping");
    return false;
  }

  const auto &ablv = m_abstractLevels[zi.ablvIdx];
  ctx->routeNodes = m_routingGraph.findRoute(
      ctx, ablv.zones, ablv.abstractEdges,
      srcNodeIdx, srcEdgeIdx, tgtNodeIdx,
      zi.sourceZone, zi.targetZone, zi.relationship,
      ctx->graph.agentCostBias,
      ctx->graph.agentMaxPerturbation);

  // Empty-route fallback: A* returns an empty edge-path in two legitimate cases:
  //   1. Source IS the target node (0 edges to traverse).
  //   2. Source is ON an edge whose endpoint IS the target node — A* seeds both
  //      directions from the same endpoint so reconstruction has no steps, but
  //      the agent can simply follow the source edge to reach the target.
  // Do NOT apply on any other A* failure — a spurious [tgtNodeIdx] route when
  // there is no real connection causes Phase F to spin with "NODE forward edge
  // not found" and reset indefinitely.
  if (ctx->routeNodes.empty() && zi.ablvIdx >= 0) {
    const bool atTarget = srcNodeIdx.has_value() && (*srcNodeIdx == tgtNodeIdx);
    const bool onDirectEdge = srcEdgeIdx.has_value() && (
        m_routingGraph.edgeFromTos[*srcEdgeIdx].first  == tgtNodeIdx ||
        m_routingGraph.edgeFromTos[*srcEdgeIdx].second == tgtNodeIdx);
    if (atTarget || onDirectEdge) {
      ctx->routeNodes.push_back(tgtNodeIdx);
      LOG_INFO("NG: Empty-path fallback route -> [" << tgtNodeIdx << "]");
    }
  }

  if (!ctx->routeNodes.empty()) {
    LOG_INFO("NG: Route computed: " << ctx->routeNodes.size()
                                    << " nodes -> tgt " << tgtNodeIdx);
    LOG_DEBUG_CONT("RouteNodes; ");
    LOG_DEBUG_FOR(auto rn : ctx->routeNodes, " " << rn);
  }
  return !ctx->routeNodes.empty();
}

// ---------------------------------------------------------------------------
// stepFromNode
//
// Agent is sitting on a graph junction node. Advance one step along the edge
// toward routeNodes[0].
// ---------------------------------------------------------------------------

GridType::Point NavigationGraph::stepFromNode(Router::RouteCtx *ctx,
                                              int srcNode,
                                              GridType::Point source) const {
  // Pop the head of the route if it's the node we're already on
  if (!ctx->routeNodes.empty() && ctx->routeNodes[0] == srcNode) {
    ctx->routeNodes.erase(ctx->routeNodes.begin());
    LOG_DEBUG("NG: NODE " << srcNode << " popped from routeNodes");
  }

  if (ctx->routeNodes.empty()) {
    // If the target lies in a dead-end whose from-node is us, enter it
    const int deIdx = ctx->graph.tgtDeadEndEdgeIdx;
    if (deIdx >= 0) {
      const auto &deEdge = m_baseEdges[deIdx];
      if (deEdge.from == srcNode && deEdge.path.size() >= 2) {
        GridType::Point nxt = deEdge.path[1];
        ctx->graph.intendedNext = nxt;
        LOG_DEBUG("NG: NODE " << srcNode << " entering dead-end edge " << deIdx);
        return nxt;
      }
    }
    LOG_DEBUG("NG: NODE " << srcNode << " routeNodes empty, stay");
    ctx->graph.intendedNext = {-1, -1};
    return source;
  }

  int toNode = ctx->routeNodes[0];
  if (toNode < 0) {
    // A* reconstruction can emit -1 as a "to" index for dead-end edges.
    // Guard here before passing it to getEdgeFromNodes, which would crash.
    LOG_ERROR("NG: NODE " << srcNode << " routeNodes[0]=" << toNode
                          << " invalid; re-routing");
    resetRouteCache(ctx);
    ctx->graph.intendedNext    = {-1, -1};
    ctx->graph.cachedTgtNodeIdx  = -1;
    ctx->graph.cachedTgtZoneIdx  = -1;
    return source;
  }
  int edgeIdx = m_routingGraph.getEdgeFromNodes(srcNode, toNode);

  // Try reverse direction if forward lookup fails
  if (edgeIdx == -1) {
    edgeIdx = m_routingGraph.getEdgeFromNodes(toNode, srcNode);
    LOG_DEBUG("NG: NODE forward edge not found, tried reverse: " << edgeIdx);
  }

  if (edgeIdx == -1) {
    LOG_ERROR("NG: NODE " << srcNode << " -> " << toNode << " no edge found; re-routing");
    resetRouteCache(ctx);
    ctx->graph.intendedNext = {-1, -1};
    ctx->graph.cachedTgtNodeIdx  = -1;
    ctx->graph.cachedTgtZoneIdx  = -1;
    return source;
  }

  const auto &edge = m_baseEdges[edgeIdx];
  // path[0] = edge.from position, path[last] = edge.to position
  if (edge.path.size() < 2) {
    LOG_ERROR("NG: Edge " << edgeIdx << " path too short");
    return source;
  }

  GridType::Point nxt = (edge.from == srcNode)
                            ? edge.path[1]
                            : edge.path[edge.path.size() - 2];
  // Record the intended next skeleton cell so Phase A (XPND fast-path) can
  // continue toward it if the agent passes through off-skeleton cells while
  // walking to the first cell of the new edge.
  ctx->graph.intendedNext = nxt;
  LOG_DEBUG("NG: NODE " << srcNode << " -> " << toNode
                        << " via edge " << edgeIdx
                        << " step to " << nxt.first << "," << nxt.second);
  return nxt;
}

// ---------------------------------------------------------------------------
// stepFromEdge
//
// Agent is on an edge corridor. Walk one step toward the end of the path that
// leads to routeNodes[0].
// ---------------------------------------------------------------------------

GridType::Point NavigationGraph::stepFromEdge(Router::RouteCtx *ctx,
                                              int edgeIdx,
                                              GridType::Point source) const {
  const auto &edge = m_baseEdges[edgeIdx];
  auto it = std::find(edge.path.begin(), edge.path.end(), source);
  if (it == edge.path.end()) {
    LOG_ERROR("NG: EDGE " << edgeIdx << " source " << source.first << ","
                          << source.second << " not found in path");
    return source;
  }

  const int idx = static_cast<int>(std::distance(edge.path.begin(), it));

  // Determine direction: which endpoint of this edge is next in our route?
  // IMPORTANT: when edge.toDeadEnd == true, edge.to is a dead-end index, NOT a
  // base node index. routeNodes only contains base node indices, so comparing
  // edge.to against routeNodes[0] on a dead-end edge is a namespace collision
  // that would falsely match (e.g. dead-end 1 == base node 1) and send the
  // agent into the dead end. Dead-end tips are never route waypoints.
  bool goForward = !edge.toDeadEnd; // default: toward edge.to unless it's a dead end
  if (!ctx->routeNodes.empty()) {
    if (!edge.toDeadEnd && edge.to == ctx->routeNodes[0]) {
      goForward = true;
    } else if (edge.from == ctx->routeNodes[0]) {
      goForward = false;
    } else {
      // Neither endpoint of this edge matches the next route waypoint — the
      // agent was physically displaced (e.g. by separation forces) onto an
      // adjacent off-route edge. The planned route is still valid for the
      // target; throwing it away forces a needless A* recompute and, worse,
      // triggers the same situation again next frame.
      // Instead: keep the route, clear intendedNext (so the XPND fast-path
      // doesn't redirect back to the wrong skeleton cell), and step toward the
      // nearer endpoint. Once back on a node, stepFromNode will resume the
      // existing route correctly.
      LOG_INFO("NG: EDGE " << edgeIdx << " (" << edge.from << "->"
                           << edge.to << (edge.toDeadEnd ? "(DEND)" : "")
                           << ") off-route (want " << ctx->routeNodes[0]
                           << "); stepping to nearer end, keeping route");
      ctx->graph.intendedNext = {-1, -1};
      // Step toward whichever endpoint is nearer
      goForward = (idx > static_cast<int>(edge.path.size()) / 2);
    }
  } else if (edgeIdx == ctx->graph.tgtDeadEndEdgeIdx) {
    // routeNodes consumed; we are intentionally entering this dead-end toward the target
    goForward = true;
  }

  int nextIdx = goForward ? idx + 1 : idx - 1;
  nextIdx = std::max(0, std::min(static_cast<int>(edge.path.size()) - 1, nextIdx));

  if (nextIdx == idx) {
    // Already at an endpoint — let the NODE handler pick it up next frame
    LOG_DEBUG("NG: EDGE " << edgeIdx << " at endpoint idx " << idx);
    return source;
  }

  LOG_DEBUG("NG: EDGE " << edgeIdx << " idx " << idx << " -> " << nextIdx
                        << " (" << (goForward ? "fwd" : "bck") << ")");
  // Record the chosen skeleton cell so the XPND fast-path can continue toward
  // it if the agent steps off-skeleton during a diagonal traversal.
  ctx->graph.intendedNext = edge.path[nextIdx];
  return edge.path[nextIdx];
}

// ---------------------------------------------------------------------------
// stepFromDeadEnd
//
// Agent is in a dead-end pocket. Always exit toward the connecting junction.
// ---------------------------------------------------------------------------

GridType::Point NavigationGraph::stepFromDeadEnd(int dendIdx,
                                                 GridType::Point source) const {
  int edgeIdx = m_routingGraph.getEdgeFromDead(dendIdx);
  if (edgeIdx == -1) {
    LOG_ERROR("NG: DEND " << dendIdx << " no edge found");
    return source;
  }
  const auto &edge = m_baseEdges[edgeIdx];
  if (edge.path.size() < 2) {
    LOG_ERROR("NG: DEND edge " << edgeIdx << " path too short");
    return source;
  }
  // path goes from junction → dead-end tip; we want second-to-last = one step toward junction
  GridType::Point nxt = edge.path[edge.path.size() - 2];
  LOG_DEBUG("NG: DEND " << dendIdx << " exit via edge " << edgeIdx
                        << " to " << nxt.first << "," << nxt.second);
  return nxt;
}

// ---------------------------------------------------------------------------
// getNextMove  —  main per-frame decision
//
// Phases:
//   A. XPND fast-path  (off-skeleton: follow expansion pointer)
//   B. Locate target node
//   C. Validate route (invalidate if target changed)
//   D. Compute route if empty
//   E. Stuck detection
//   F. Step based on cell type
// ---------------------------------------------------------------------------

GridType::Point NavigationGraph::getNextMove(Router::RouteCtx *ctx,
                                             GridType::Point source,
                                             GridType::Point target) {
  const int srcCell = m_infoGrid[source.second][source.first];

  // ------------------------------------------------------------------
  // Pre-Phase: Non-skeleton accumulator
  // Counts frames the agent spends on off-skeleton cells (XPND, WALL,
  // BOUNDARY, unknown) without ever touching an EDGE/NODE/DEND cell.
  // When the threshold is exceeded the agent is genuinely lost in a bad
  // XPND region — Phase A is bypassed so Phase D can force a fresh A*
  // and Phase F's fallback scan can steer the agent back to the skeleton.
  // ------------------------------------------------------------------
  const bool onSkeleton = srcCell & (GridType::NODE | GridType::EDGE | GridType::DEND);
  if (onSkeleton) {
    ctx->graph.nonSkeletonFrames = 0;
  } else {
    ctx->graph.nonSkeletonFrames++;
  }
  const bool xpndStuck = ctx->graph.nonSkeletonFrames >= NON_SKELETON_STUCK_THRESHOLD;
  if (xpndStuck) {
    LOG_INFO("NG: Non-skeleton stuck (" << ctx->graph.nonSkeletonFrames
             << " frames) at " << source.first << "," << source.second
             << "; bypassing Phase A for re-route");
    ctx->graph.nonSkeletonFrames = 0;
    ctx->graph.intendedNext      = {-1, -1};
    resetRouteCache(ctx);
    ctx->graph.cachedTgtNodeIdx  = -1;
    ctx->graph.cachedTgtZoneIdx  = -1;
    // Fall through — Phase B-F will re-route and Phase F's fallback scan
    // will steer the agent toward the nearest skeleton cell.
  }

  // ------------------------------------------------------------------
  // Phase A: XPND fast-path
  // Off-skeleton cells store a direction pointer toward the nearest edge.
  // Normally we follow it immediately. BUT: skeleton edges can have diagonal
  // steps. With L-shaped (axis-aligned) movement the agent passes through
  // off-skeleton (XPND) cells on the way to the next skeleton cell. If we
  // blindly follow the XPND pointer it points at the WRONG part of the
  // skeleton and undoes the traversal every frame.
  // Fix: stepFromEdge records its chosen next skeleton cell as intendedNext.
  // While on XPND and that cell is still adjacent, continue toward it instead.
  // ------------------------------------------------------------------
  if (!xpndStuck && (srcCell & GridType::XPND)) {
    // If the agent is at or adjacent to the world target, navigate directly
    // rather than letting XPND redirect it back to the skeleton. This stops
    // oscillation when the world target itself lies in an XPND zone.
    {
      const int dx = std::abs(source.first  - target.first);
      const int dy = std::abs(source.second - target.second);
      if (dx <= 1 && dy <= 1) {
        LOG_DEBUG("NG: XPND adjacent to world target "
                  << target.first << "," << target.second << "; direct step");
        return target;
      }
    }
    // Route consumed (direct-nav mode): don't redirect back to the skeleton.
    // intendedNext from a previous skeleton step may still be adjacent,
    // making the adjacency check below permanently pass and creating an
    // oscillation: skeleton → XPND → intendedNext → skeleton → repeat.
    // When the route is gone and a target is cached the skeleton has done
    // its job; fall through to direct steering instead.
    //
    // Exception: if the agent is far from the world target the route was
    // consumed prematurely (e.g. physics pushed it after A* finished).
    // Clear the cache so Phase D re-routes; then fall through to raw XPND
    // steering which brings the agent back to the skeleton in a few frames.
    if (ctx->routeNodes.empty() && ctx->graph.cachedTgtNodeIdx >= 0) {
      const int tdx = std::abs(source.first  - target.first);
      const int tdy = std::abs(source.second - target.second);
      if (tdx + tdy <= 4) {
        ctx->graph.intendedNext = {-1, -1};
        LOG_DEBUG("NG: XPND direct-nav mode, skip skeleton redirect");
        return source;
      }
      // Far from world target — stale route. Reset so Phase D re-routes.
      LOG_INFO("NG: XPND direct-nav but far from target (" << (tdx + tdy)
               << " cells); clearing cache to re-route");
      ctx->graph.cachedTgtNodeIdx = -1;
      ctx->graph.cachedTgtZoneIdx = -1;
      ctx->graph.intendedNext     = {-1, -1};
      // Fall through: raw XPND pointer steers back to the skeleton.
    }

    if (ctx->graph.intendedNext.first >= 0) {
      const GridType::Point inxt = ctx->graph.intendedNext; // copy so we can clear
      const int dx = std::abs(source.first  - inxt.first);
      const int dy = std::abs(source.second - inxt.second);
      if (dx == 0 && dy == 0) {
        // Agent has arrived at intendedNext — consume it and fall through to
        // raw XPND so the cell's own pointer steers further into the skeleton.
        // Without this, returning inxt == source causes a "same-cell" result
        // which triggers direct-angle movement back into the wall every frame.
        ctx->graph.intendedNext = {-1, -1};
        LOG_DEBUG("NG: XPND intendedNext arrived, consuming");
      } else if (dx <= 1 && dy <= 1) {
        LOG_DEBUG("NG: XPND override intendedNext -> " << inxt.first << "," << inxt.second);
        return inxt;
      } else {
        // Too far from intended target — stale, fall through to raw XPND
        ctx->graph.intendedNext = {-1, -1};
        LOG_DEBUG("NG: XPND intendedNext stale, clearing");
      }
    }
    int dirIdx = GridType::get_XPND_DIR(srcCell);
    GridType::Point dir = GridType::directions8[dirIdx];
    GridType::Point nxt = {source.first + dir.first, source.second + dir.second};
    LOG_DEBUG("NG: XPND -> " << nxt.first << "," << nxt.second);
    return nxt;
  }

  // ------------------------------------------------------------------
  // Phase B: Locate target node
  // ------------------------------------------------------------------
  const ClosestNodeInfo tgtInfo = getClosestNode(target);
  const int tgtNodeIdx = tgtInfo.nodeIdx;
  if (tgtNodeIdx < 0) {
    LOG_ERROR("NG: Could not locate target node for " << target.first
                                                      << "," << target.second);
    return source;
  }

  // Cheap zone lookup (level 0 only; -1 if no levels built).
  const int tgtZoneIdx = (!m_abstractLevels.empty())
      ? m_abstractLevels[0].zoneGrid[target.second][target.first].closestAbstractNodeIdx
      : -1;

  // Determine if the target lies in/on a dead-end edge (updated every frame,
  // cheap: one unordered_map lookup at most).
  {
    int deIdx = -1;
    if (!tgtInfo.isEdge && tgtInfo.edgeOrDeadEndIdx >= 0) {
      // Target is at a DEND tip cell — find the connecting edge
      deIdx = m_routingGraph.getEdgeFromDead(tgtInfo.edgeOrDeadEndIdx);
    } else if (tgtInfo.isEdge && tgtInfo.edgeOrDeadEndIdx >= 0) {
      // Target is on an EDGE — check if it is a dead-end edge (to == -1)
      if (m_routingGraph.edgeFromTos[tgtInfo.edgeOrDeadEndIdx].second == -1)
        deIdx = tgtInfo.edgeOrDeadEndIdx;
    }
    ctx->graph.tgtDeadEndEdgeIdx = deIdx;
  }

  // ------------------------------------------------------------------
  // Phase C: Invalidate stale route when the target ZONE changed.
  //
  // Using zone rather than node avoids thrashing when a moving target
  // crosses node boundaries within the same zone — the boundary waypoints
  // that form the bulk of a distant route are zone-stable.  The agent
  // self-corrects on the final leg once it arrives in the target zone.
  // ------------------------------------------------------------------
  if (!ctx->routeNodes.empty() &&
      ctx->graph.cachedTgtNodeIdx != tgtNodeIdx) {
    const bool zoneChanged = (tgtZoneIdx < 0
                              || ctx->graph.cachedTgtZoneIdx < 0
                              || tgtZoneIdx != ctx->graph.cachedTgtZoneIdx);
    if (zoneChanged) {
      LOG_INFO("NG: Target zone changed " << ctx->graph.cachedTgtZoneIdx
                                         << " -> " << tgtZoneIdx
                                         << " (node " << ctx->graph.cachedTgtNodeIdx
                                         << " -> " << tgtNodeIdx << "); clearing route");
      resetRouteCache(ctx);
      ctx->graph.intendedNext = {-1, -1};
    }
    // Zone unchanged: keep existing route. Boundary waypoints are still
    // valid; the agent navigates to the old target node (same zone) and
    // Phase D recomputes same-zone A* once routeNodes is exhausted.
  }

  // ------------------------------------------------------------------
  // Phase D: Build route if empty
  // ------------------------------------------------------------------
  if (ctx->routeNodes.empty()) {
    // If the agent is at the junction node for the target dead-end, or is
    // already traversing that dead-end edge, skip A* entirely — Phase F handles
    // the stepping via the tgtDeadEndEdgeIdx branches.
    bool skipToPhaseF = false;
    if (ctx->graph.tgtDeadEndEdgeIdx >= 0) {
      const int deFromNode = m_baseEdges[ctx->graph.tgtDeadEndEdgeIdx].from;
      const bool atJunction    = (srcCell & GridType::NODE) &&
                                 ((srcCell & 0xffff) == deFromNode);
      const bool onDeadEndEdge = (srcCell & GridType::EDGE) &&
                                 ((srcCell & GridType::EDGE_MASK) ==
                                  ctx->graph.tgtDeadEndEdgeIdx);
      skipToPhaseF = (atJunction || onDeadEndEdge);
    }
    if (!skipToPhaseF) {
      // Route was already computed for this target and fully consumed.
      // Suppress A* only while the agent is close to the target node (final
      // approach / arrived). If the agent has drifted far away (e.g. pushed by
      // physics after route was consumed), clear the cache so A* re-routes.
      if (ctx->graph.cachedTgtNodeIdx == tgtNodeIdx) {
        const int dx = std::abs(source.first  - tgtInfo.closestGraphPoint.first);
        const int dy = std::abs(source.second - tgtInfo.closestGraphPoint.second);
        if (dx + dy <= 4) {
          LOG_DEBUG("NG: Route consumed for tgt node " << tgtNodeIdx << "; direct nav");
          return source;
        }
        LOG_INFO("NG: Route consumed but far from node " << tgtNodeIdx
                 << " (dist=" << (dx + dy) << "); re-routing");
        resetRouteCache(ctx);
        ctx->graph.cachedTgtNodeIdx  = -1;
    ctx->graph.cachedTgtZoneIdx  = -1;
      }
      // Resolve source to node/edge for the A* call
      std::optional<int> srcNodeIdx;
      std::optional<int> srcEdgeIdx;
      if (srcCell & GridType::NODE) {
        srcNodeIdx = srcCell & 0xffff;
      } else if (srcCell & GridType::EDGE) {
        srcEdgeIdx = srcCell & GridType::EDGE_MASK;
      } else {
        ClosestNodeInfo srcInfo = getClosestNode(source);
        if (srcInfo.nodeIdx >= 0) {
          srcNodeIdx = srcInfo.nodeIdx;
        } else if (srcInfo.isEdge && srcInfo.edgeOrDeadEndIdx >= 0) {
          srcEdgeIdx = srcInfo.edgeOrDeadEndIdx;
        } else {
          // getClosestNode could not resolve source to any graph element.
          // This happens when physics pushes the agent into a WALL|BOUNDARY cell
          // (not NODE/EDGE/DEND/XPND). Calling A* with null src/edge always fails,
          // so skip it and step toward the nearest skeleton cell immediately.
          LOG_INFO("NG: Phase D: unresolvable source " << source.first << ","
                   << source.second << " srcCell=0x" << std::hex << srcCell
                   << std::dec << "; escaping to nearest skeleton cell");
          const int rows = static_cast<int>(m_infoGrid.size());
          const int cols = (rows > 0) ? static_cast<int>(m_infoGrid[0].size()) : 0;
          for (const auto &d : GridType::directions8) {
            const int nx = source.first  + d.first;
            const int ny = source.second + d.second;
            if (nx >= 0 && ny >= 0 && ny < rows && nx < cols) {
              const int nc = m_infoGrid[ny][nx];
              if (nc & (GridType::NODE | GridType::EDGE |
                        GridType::XPND | GridType::DEND)) {
                ctx->graph.intendedNext = {nx, ny};
                return {nx, ny};
              }
            }
          }
          return source; // no navigable neighbour found
        }
      }

      ZoneInfo zi = findZoneRelationship(source, target);

      if (!ensureRoute(ctx, zi, srcNodeIdx, srcEdgeIdx, tgtNodeIdx)) {
        LOG_INFO("NG: No route found src=" << source.first << "," << source.second
                                           << " tgt=" << target.first << ","
                                           << target.second << "; staying");
        return source;
      }
      ctx->graph.cachedTgtNodeIdx  = tgtNodeIdx;
      ctx->graph.cachedTgtZoneIdx  = tgtZoneIdx;
    }
  }

  // ------------------------------------------------------------------
  // Phase E: Stuck detection
  // If the agent has been on the same grid cell for too long, the route
  // is discarded so it will be recomputed next frame, and we step toward
  // the nearest edge endpoint to break the deadlock (e.g. head-on agents).
  // ------------------------------------------------------------------
  if (source == ctx->graph.lastGridPos) {
    ctx->graph.stuckFrames++;
    if (ctx->graph.stuckFrames >= STUCK_THRESHOLD) {
      LOG_INFO("NG: STUCK " << ctx->graph.stuckFrames << " frames at "
                            << source.first << "," << source.second
                            << "; forcing escape");
      resetRouteCache(ctx);
      ctx->graph.cachedTgtNodeIdx  = -1;
    ctx->graph.cachedTgtZoneIdx  = -1;
      ctx->graph.stuckFrames      = 0;
      ctx->graph.intendedNext     = {-1, -1};

      // Escape: if on an edge, step toward its nearer endpoint
      if (srcCell & GridType::EDGE) {
        int eIdx = srcCell & GridType::EDGE_MASK;
        const auto &edge = m_baseEdges[eIdx];
        auto it = std::find(edge.path.begin(), edge.path.end(), source);
        if (it != edge.path.end()) {
          int idx = static_cast<int>(std::distance(edge.path.begin(), it));
          // Step toward whichever end is closer
          bool goFwd = idx > static_cast<int>(edge.path.size()) / 2;
          int ni = goFwd ? std::min(idx + 1, (int)edge.path.size() - 1)
                         : std::max(idx - 1, 0);
          if (ni != idx)
            return edge.path[ni];
        }
      }
      return source; // stay; re-route happens next frame
    }
  } else {
    ctx->graph.stuckFrames = 0;
  }
  ctx->graph.lastGridPos = source;

  // ------------------------------------------------------------------
  // Phase F: Step based on cell type
  // ------------------------------------------------------------------
  const CellKind kind = classifyCell(srcCell);

  if (kind == CellKind::NODE) {
    return stepFromNode(ctx, srcCell & 0xffff, source);
  }
  if (kind == CellKind::EDGE) {
    return stepFromEdge(ctx, srcCell & GridType::EDGE_MASK, source);
  }
  if (kind == CellKind::DEND) {
    GridType::Point nxt = stepFromDeadEnd(srcCell & 0xffff, source);
    ctx->graph.intendedNext = nxt; // guard XPND redirect on the way out of the dead end
    return nxt;
  }

  LOG_ERROR("NG: Unknown cell type at " << source.first << "," << source.second
                                        << " cell: " << std::hex << srcCell
                                        << std::dec);

  // The agent is on a cell with no recognized navigation type (not NODE/EDGE/
  // DEND/XPND). This can happen when physics pushes the agent into a floor cell
  // that the navmesh expansion didn't cover. If routeNodes is non-empty, Phase D
  // is skipped every frame so the route is never rebuilt — causing a permanent
  // silent "same-cell" loop. Reset the route and step toward the nearest
  // adjacent skeleton cell so the agent can re-enter the navmesh.
  resetRouteCache(ctx);
  ctx->graph.cachedTgtNodeIdx = -1;
  ctx->graph.intendedNext     = {-1, -1};
  {
    const int rows = static_cast<int>(m_infoGrid.size());
    const int cols = (rows > 0) ? static_cast<int>(m_infoGrid[0].size()) : 0;
    for (const auto &d : GridType::directions8) {
      const int nx = source.first  + d.first;
      const int ny = source.second + d.second;
      if (nx >= 0 && ny >= 0 && ny < rows && nx < cols) {
        const int nc = m_infoGrid[ny][nx];
        if (nc & (GridType::NODE | GridType::EDGE | GridType::XPND | GridType::DEND)) {
          ctx->graph.intendedNext = {nx, ny};
          return {nx, ny};
        }
      }
    }
  }
  return source;
}

// ---------------------------------------------------------------------------
// getMoveDirection  —  public API
//
// Converts world float coordinates to grid cells, delegates to getNextMove,
// then converts the returned next-cell centre back to a steering angle.
// ---------------------------------------------------------------------------

float NavigationGraph::getMoveDirection(Router::RouteCtx *ctx,
                                        GridType::Vec2 from, GridType::Vec2 to,
                                        int type, float /*dt*/) {
  const float cellSizeX = m_info.mCellWidth  * static_cast<float>(CELL_MULT);
  const float cellSizeY = m_info.mCellHeight * static_cast<float>(CELL_MULT);

  GridType::Point fromPnt = {static_cast<int>(from.x / cellSizeX),
                             static_cast<int>(from.y / cellSizeY)};
  GridType::Point toPnt   = {static_cast<int>(to.x   / cellSizeX),
                             static_cast<int>(to.y   / cellSizeY)};

  ctx->from = fromPnt;
  ctx->to   = toPnt;

  ctx->next = getNextMove(ctx, fromPnt, toPnt);

  // If the next cell is the same as the current cell we are essentially at the
  // target — steer directly toward the world target position.
  if (ctx->next == fromPnt) {
    ctx->curDir = computeAngle(to.x - from.x, to.y - from.y);
    LOG_DEBUG("NG: same-cell -> direct angle " << ctx->curDir);
    return ctx->curDir;
  }

  // Steer toward the centre of the next grid cell
  float nextCX = ctx->next.first  * cellSizeX + cellSizeX * 0.5f;
  float nextCY = ctx->next.second * cellSizeY + cellSizeY * 0.5f;
  ctx->curDir = computeAngle(nextCX - from.x, nextCY - from.y);

  LOG_DEBUG("NG: from " << fromPnt.first << "," << fromPnt.second
                        << " next " << ctx->next.first << "," << ctx->next.second
                        << " angle " << ctx->curDir);
  return ctx->curDir;
}

float NavigationGraph::stuck(Router::RouteCtx *ctx, GridType::Vec2 from,
                             GridType::Vec2 to, int type, float dt) {
  if (ctx->graph.stuckRecoveryFrames == 0) {
    // First frame of recovery: clear stale navigation state
    ctx->graph.intendedNext      = {-1, -1};
    ctx->graph.nonSkeletonFrames = 0;
    resetRouteCache(ctx);
    ctx->graph.cachedTgtNodeIdx = -1;
    ctx->graph.cachedTgtZoneIdx = -1;
    LOG_INFO("NG: stuck() recovery started at "
             << from.x << "," << from.y);
  }

  ctx->graph.stuckRecoveryFrames++;

  // Phase 1 (frames 1-30): settle — suppress movement so knockback/separation dissipates
  if (ctx->graph.stuckRecoveryFrames <= 30) {
    return NavigationAPI::NO_MOVE;
  }

  // Phase 2 (frames 31-60): steer directly toward target, bypassing graph
  if (ctx->graph.stuckRecoveryFrames <= 60) {
    float dx = to.x - from.x;
    float dy = to.y - from.y;
    float angle = std::atan2(dy, dx) * 180.0f / 3.14159265f;
    if (angle < 0.0f) angle += 360.0f;
    return angle;
  }

  // Recovery complete
  ctx->graph.stuckRecoveryFrames = 0;
  ctx->isStuck = false;
  return NavigationAPI::NO_MOVE;
}

} // namespace Routing
} // namespace DistanceMap
