#include "NavigationGraph.hpp"

#include <algorithm>
#include <cmath>

#include "Debug.h"
#include "DistanceMapCore.hpp"
#include "MathUtils.h"
#include "NavigationAPI.hpp"

namespace DistanceMap {
namespace Routing {

NavigationGraph::NavigationGraph(const GridToGraph::Graph& graphData,
                                 const Router::Info& info)
    : NavigationAPI(graphData.infoGrid, info),
      m_baseGraph(graphData.baseGraph),
      m_pathCostMap(graphData.pathCostMap),
      m_routingGraph(graphData.routingGraph),
      m_baseEdges(graphData.baseEdges),
      m_baseNodes(graphData.baseNodes),
      m_deadEnds(graphData.deadEnds),
      m_abstractLevels(graphData.abstractLevels) {}

float NavigationGraph::computeAngle(double dx, double dy) {
  if (dx == 0 && dy == 0) {
    return 0.0;  // No movement
  }
  return static_cast<float>(
      std::fmod(std::atan2(dy, dx) * (180.0 / MY_PI) + 360.0, 360.0));
}

NavigationGraph::ClosestNodeInfo
NavigationGraph::getClosestNode(const GridType::Point& pos) {
  const int edgeIdx =
      m_routingGraph.edgeGrid.get(pos.first, pos.second) & GridType::EDGE_MASK;
  int cell = m_infoGrid[pos.second][pos.first];
  LOG_DEBUG(
      "getClosest: " << pos.first << "," << pos.second
                     << " => edgeIdx: " << edgeIdx << " ("
                     << (m_routingGraph.edgeFromTos[edgeIdx].first) << "->"
                     << (m_routingGraph.edgeFromTos[edgeIdx].second) << ")"
                     << " cell: " << std::hex << cell << std::dec);
  if (cell & GridType::NODE) {
    LOG_DEBUG("getClosest: at NODE " << (cell & 0xffff));
    return {cell & 0xffff, -1, false, pos};
  }

  if (cell & GridType::EDGE) {
    int edgeIdx = cell & GridType::EDGE_MASK;
    const auto& fromto = m_routingGraph.edgeFromTos[edgeIdx];
    LOG_DEBUG("getClosest: at EDGE " << edgeIdx << " fromto: " << fromto.first
                                     << "->" << fromto.second);
    if (!(cell & GridType::EDGE_HALF) || fromto.second < 0) {
      return {fromto.first, edgeIdx, true, pos};
    } else {
      return {fromto.second, edgeIdx, true, pos};
    }
  }
  GridType::Point edgePnt = pos;
  while (cell & GridType::XPND) {
    int dst = GridType::get_XPND_DIST(cell);
    int dir = GridType::get_XPND_DIR(cell);
    edgePnt.first += GridType::directions8[dir].first * dst;
    edgePnt.second += GridType::directions8[dir].second * dst;
    cell = m_infoGrid[edgePnt.second][edgePnt.first];
    LOG_DEBUG("GNE: XPND => move in dir:"
              << dir << " by dst:" << dst << " => " << edgePnt.first << ","
              << edgePnt.second << " cell: " << std::hex << cell << std::dec);
  }
  if (cell & GridType::EDGE) {
    LOG_DEBUG("getClosest: At EDGE => " << (cell & GridType::EDGE_MASK));
    int edgeIdx = cell & GridType::EDGE_MASK;
    const auto fromto = m_routingGraph.edgeFromTos[edgeIdx];
    LOG_DEBUG("  => node: " << ((cell & GridType::EDGE_HALF) ? fromto.second
                                                             : fromto.first));
    if (!(cell & GridType::EDGE_HALF) || fromto.second < 0) {
      return {fromto.first, edgeIdx, true, edgePnt};
    } else {
      return {fromto.second, edgeIdx, true, edgePnt};
    }
  }
  if (cell & GridType::NODE) {
    int node = cell & 0xffff;
    LOG_DEBUG("getClosest: Moved to NODE " << node);
    return {node, -1, false, edgePnt};
  }
  if (cell & GridType::DEND) {
    int toDend = cell & 0xffff;
    int from = m_routingGraph.edgeFromTos[edgeIdx].first;
    LOG_DEBUG("getClosest: Moved to DEND " << toDend << " fromNode: " << from);
    return {from, toDend, false, edgePnt};
  } else {
    LOG_ERROR("Cell not moved to edge " << " (" << edgePnt.first << ","
                                        << edgePnt.second << ") => " << std::hex
                                        << cell << std::dec);
    return {-1, -1, false, edgePnt};
  }
}

GridType::Point NavigationGraph::nextPoint(const GridType::Point& from,
                                           const GridType::Point& dir) {
  return {from.first + dir.first, from.second + dir.second};
}

GridType::Point NavigationGraph::nextStep(const GridType::Point& from,
                                          const GridType::Point& to) {
  GridType::Point dir = {to.first - from.first, to.second - from.second};
  GridType::Point p = nextPoint(from, dir);
  LOG_DEBUG("  NXTSTP f: " << from.first << "," << from.second
                           << " t: " << to.first << "," << to.second
                           << " => d:" << dir.first << "," << dir.second
                           << " => p: " << p.first << "," << p.second);
  return p;
}

GridType::Point NavigationGraph::getNextMove(Router::RouteCtx* ctx,
                                             GridType::Point source,
                                             GridType::Point target) {
  // Check if source in a wall
  int srcCell = m_infoGrid[source.second][source.first];
  LOG_DEBUG("GetNextMove: " << source.first << "," << source.first
                            << " src: " << std::hex << srcCell << std::dec);
  if (srcCell & GridType::WALL) {
    // If boundary then use dir to move ouit of wall
    if (srcCell & GridType::BOUNDARY) {
      int dirIdx = srcCell & GridType::DIR_MASK;
      source.first += GridType::directions8[dirIdx].first;
      source.second += GridType::directions8[dirIdx].second;
      LOG_DEBUG("GM: SRC BOUNDARY => src: " << source.first << ","
                                            << source.second);
    }

    // If still in wall then error
    srcCell = m_infoGrid[source.second][source.first];
    if (srcCell & GridType::WALL) {
      LOG_ERROR(source.first << "," << source.second
                             << " is WALL: SRC ***************************");
    }
    return source;
  }

  // Check target in wall
  int tgtCell = m_infoGrid[target.second][target.first];
  if (tgtCell & GridType::WALL) {
    // If boundary then use dir to move ouit of wall
    if (tgtCell & GridType::BOUNDARY) {
      int dirIdx = tgtCell & GridType::DIR_MASK;
      target.first += GridType::directions8[dirIdx].first;
      target.second += GridType::directions8[dirIdx].second;
      LOG_DEBUG("GM: TGT BOUNDARY => tgt: " << target.first << ","
                                            << target.second);
    }

    // If still in wall then error
    if (m_infoGrid[target.second][target.first] & GridType::WALL) {
      LOG_ERROR(target.first << "," << target.second
                             << " is WALL: TARG ***************************");
      return source;
    }
  }

  //
  // If on XPND then move towards edge
  //
  if (srcCell & GridType::XPND) {
    int dirIdx = GridType::get_XPND_DIR(srcCell);
    GridType::Point dir = GridType::directions8[dirIdx];
    LOG_DEBUG("GM: At XPND move in dir: " << dir.first << "," << dir.second);
    return nextPoint(source, dir);
  }

  if (!ctx->routeNodes.empty()) {
    if (srcCell & GridType::NODE) {
      int from = srcCell & 0xffff;
      LOG_DEBUG("DBG: At node: " << from);
      if (ctx->routeNodes[0] == from) {
        LOG_DEBUG("DBG: which is 1st of routeNodes");
        ctx->routeNodes.erase(ctx->routeNodes.begin());
        if (!ctx->routeNodes.empty()) {
          int to = ctx->routeNodes[0];
          int edgeIdx = m_routingGraph.getEdgeFromNodes(from, to);
          const auto pnt = m_baseEdges[edgeIdx].path[1];
          LOG_DEBUG("DBG: N1: " << from << " N2:" << to << " E: " << edgeIdx
                                << " return path[1] " << pnt.first << ","
                                << pnt.second);
          return pnt;
        }
      }
    }

    if (srcCell & GridType::EDGE) {
      int edgeIdx = srcCell & GridType::EDGE_MASK;
      const auto& edge = m_baseEdges[edgeIdx];
      LOG_DEBUG("DBG: At EDGE " << edgeIdx << " (" << edge.from << "->"
                                << edge.to << ") => follow ");
      auto it = std::find(edge.path.begin(), edge.path.end(), source);
      if (it != edge.path.end()) {
        size_t index = std::distance(edge.path.begin(), it);
        int adj = (edge.from == ctx->routeNodes[0]) ? -1
                  : (edge.to == ctx->routeNodes[0]) ? 1
                                                    : 0;

        if (adj != 0) {
          LOG_DEBUG("DBG: Found src in path index:"
                    << index << " adj: " << adj << " move to "
                    << edge.path[index + adj].first << ","
                    << edge.path[index + adj].second);
          return edge.path[index + adj];
        } else {
          LOG_DEBUG(
              "DBG: Found src in path but adj is 0 (stuck). Clearing "
              "routeNodes.");
          ctx->routeNodes.clear();
        }
      } else {
        LOG_ERROR("DBG: Source not found in path");
      }
    }
  }

  // Check all the levels and find the level at which the zones are adjacent
  int ablvIdx = -1;
  int targetZone = -1;
  int sourceZone = -1;
  for (int levelIdx = 0;
       (ablvIdx == -1) && (levelIdx < m_abstractLevels.size()); ++levelIdx) {
    const auto& ablv = m_abstractLevels[levelIdx];
    targetZone =
        ablv.zoneGrid[target.second][target.first].closestAbstractNodeIdx;
    sourceZone =
        ablv.zoneGrid[source.second][source.first].closestAbstractNodeIdx;
    // Check if in same zone for this level
    if (targetZone == sourceZone) {
      ablvIdx = levelIdx;
      break;
    }

    // Check if target is one of the source neighbors
    for (int adjZone : ablv.zones[sourceZone].adjacentZones) {
      if (adjZone == targetZone) {
        ablvIdx = levelIdx;
        break;
      }
    }
  }
  LOG_DEBUG("GM: Found: srcZ: " << sourceZone << " tgtZ: " << targetZone
                                << " idx: " << ablvIdx);

  // Get the ablv. Use top level if more than 1 zone apart
  const auto& ablv = (ablvIdx != -1)
                         ? m_abstractLevels[ablvIdx]
                         : m_abstractLevels[m_abstractLevels.size() - 1];

  // Route through the appropriate level based on zone distance
  // If at Node use that
  std::optional<int> srcNodeIdx = (srcCell & GridType::NODE)
                                      ? std::optional<int>(srcCell & 0xffff)
                                      : std::nullopt;
  // If at Edge use that
  std::optional<int> srcEdgeIdx =
      (srcCell & GridType::EDGE)
          ? std::optional<int>(srcCell & GridType::EDGE_MASK)
          : std::nullopt;
  // Otherwise find the closest node
  if (!srcNodeIdx && !srcEdgeIdx) {
    LOG_DEBUG("GM: Not N/E FindCloset to source " << source.first << ","
                                                  << source.second);
    const auto& srcEdgeOrDed = getClosestNode(source);
    srcNodeIdx = srcEdgeOrDed.nodeIdx;
  }

  LOG_DEBUG("GM: source node/edge found. Find target");

  // If same zone then use the node
  // want to have "find closest node using XPND edges NODE"
  ClosestNodeInfo tgtEdgeOrDed =
      (sourceZone == targetZone)
          ? getClosestNode(target)
          : ClosestNodeInfo{ablv.abstractNodes[targetZone].baseCenterNode, -1,
                            false, target};
  LOG_DEBUG("GM: Got TGT closest node: " << tgtEdgeOrDed.nodeIdx
                                         << " edge: " << tgtEdgeOrDed.edgeOrDeadEndIdx << " isEdge: " << tgtEdgeOrDed.isEdge);

  int tgtNodeIdx = tgtEdgeOrDed.nodeIdx;
  // -1 = same zone, >=0 = adjacent zone index, -2 = distant
  int zoneRelationship = (ablvIdx == -1)              ? -2
                         : (sourceZone == targetZone) ? -1
                                                      : ablvIdx;

  // If in the same zone, the src and tgt are the same node
  // then use the edge connecting to the other node
  if (zoneRelationship == -1 && srcNodeIdx && *srcNodeIdx == tgtNodeIdx &&
      tgtEdgeOrDed.edgeOrDeadEndIdx >= 0) {
    int edgeIdx = (tgtEdgeOrDed.isEdge)
                      ? tgtEdgeOrDed.edgeOrDeadEndIdx
                      : m_routingGraph.getEdgeFromDead(tgtEdgeOrDed.edgeOrDeadEndIdx);
    if (edgeIdx != -1) {
      const auto& edge = m_baseEdges[edgeIdx];
      const auto& path = edge.path;
      const auto it = std::find(path.begin(), path.end(), source);
      if (it != path.end()) {
        int idx = std::distance(path.begin(), it);
        LOG_DEBUG("DBG=== Found Source "
                  << source.first << "," << source.second << " idx: " << idx
                  << " tgtNode: " << tgtNodeIdx << " (" << edge.from << "->"
                  << edge.to << (edge.toDeadEnd ? " (DED)" : ""));
        if (tgtNodeIdx != edge.from) {
          idx = std::max(0, idx - 1);
          LOG_DEBUG("DBG=== Move BCK to " << path[idx].first << ","
                                          << path[idx].second);
          return nextStep(source, path[idx]);
        } else {
          idx = std::min(static_cast<int>(path.size()) - 1, idx + 1);
          LOG_DEBUG("DBG=== Move FWD to " << path[idx].first << ","
                                          << path[idx].second);
          return nextStep(source, path[idx]);
        }
      } else {
        LOG_DEBUG("DBG=== Source " << source.first << "," << source.second
                                   << " Not found in Edge: " << edgeIdx << " ("
                                   << edge.from << "->" << edge.to
                                   << (edge.toDeadEnd ? " (DED)" : ""));
      }
    } else {
      LOG_DEBUG("DBG=== No edge found");
    }
  }
  LOG_DEBUG("GM: tgtNode: " << tgtNodeIdx << " => Find routeNodes");

  // Special case: Source is on an edge.
  if (srcCell & GridType::EDGE) {
    int currentEdgeIdx = srcCell & GridType::EDGE_MASK;

    // Check if target is on the same edge or in a dead end connected to this edge.
    int targetEdgeIdx = -1;
    bool targetIsDeadEnd = false;

    if (tgtEdgeOrDed.isEdge) {
      targetEdgeIdx = tgtEdgeOrDed.edgeOrDeadEndIdx;
    } else if (tgtEdgeOrDed.edgeOrDeadEndIdx >= 0) {
      // Target is in a dead end.
      targetEdgeIdx = m_routingGraph.getEdgeFromDead(tgtEdgeOrDed.edgeOrDeadEndIdx);
      targetIsDeadEnd = true;
      LOG_DEBUG("GM: tgt edge: " << targetEdgeIdx << " from DeadIdx:" << tgtEdgeOrDed.edgeOrDeadEndIdx);
    }
    LOG_DEBUG("GM: curEdge:" << currentEdgeIdx << " tgtEdge:" << targetEdgeIdx << " dend:" << targetIsDeadEnd);

    if (currentEdgeIdx != -1 && currentEdgeIdx == targetEdgeIdx) {
      // We are on the correct edge.
      // Determine direction.
      const auto& edge = m_baseEdges[currentEdgeIdx];
      auto itSrc = std::find(edge.path.begin(), edge.path.end(), source);
      if (itSrc != edge.path.end()) {
        int srcIdx = std::distance(edge.path.begin(), itSrc);

        int tgtPathIdx = -1;
        if (targetIsDeadEnd) {
          // Dead End: Target is effectively at the end of the path.
          tgtPathIdx = edge.path.size() - 1;
          LOG_DEBUG("GM: put tgt at end of path: " << tgtPathIdx);
        } else {
          // Target is on the edge. Search for it on the path.
          // Note: target point might not be exactly on the path if it's
          // floating point, but here we are dealing with integer grid points.
          // If target is not found, we might need to find the closest point on
          // path.
          auto itTgt = std::find(edge.path.begin(), edge.path.end(),
                                 tgtEdgeOrDed.closestGraphPoint);
          if (itTgt != edge.path.end()) {
            tgtPathIdx = std::distance(edge.path.begin(), itTgt);
            LOG_DEBUG("GM: target (closestGraphPoint):"
                      << tgtEdgeOrDed.closestGraphPoint.first << ","
                      << tgtEdgeOrDed.closestGraphPoint.second
                      << " found on path idx: " << tgtPathIdx);
          } else {
            LOG_DEBUG("GM: target (closestGraphPoint):"
                      << tgtEdgeOrDed.closestGraphPoint.first << ","
                      << tgtEdgeOrDed.closestGraphPoint.second
                      << " not found on path. Search path for closest");
            // Fallback: Find closest point on path to target
            // Use closestGraphPoint as the target reference if available
            GridType::Point refTgt = tgtEdgeOrDed.closestGraphPoint;

            int minDist = std::numeric_limits<int>::max();
            for (int i = 0; i < edge.path.size(); ++i) {
              int d = std::abs(edge.path[i].first - refTgt.first) +
                      std::abs(edge.path[i].second - refTgt.second);
              if (d < minDist) {
                LOG_DEBUG("GM: losest pnt:" << i << " " << edge.path[i].first
                                            << "," << edge.path[i].second
                                            << " dist:" << d);
                minDist = d;
                tgtPathIdx = i;
              }
            }
          }
        }

        if (tgtPathIdx != -1) {
          LOG_DEBUG("GM: On same edge as target. srcIdx:"
                    << srcIdx << " tgtIdx:" << tgtPathIdx);
          if (srcIdx < tgtPathIdx) {
            return nextStep(source, edge.path[srcIdx + 1]);
          } else if (srcIdx > tgtPathIdx) {
            return nextStep(source, edge.path[srcIdx - 1]);
          } else {
            return nextStep(source, target);
          }
        }
      }
    }
  }

  ctx->routeNodes = m_routingGraph.findRoute(
      ctx, ablv.zones, ablv.abstractEdges, srcNodeIdx, srcEdgeIdx, tgtNodeIdx,
      sourceZone, targetZone, zoneRelationship);

  // Route using the list of nodes for the path
  LOG_DEBUG_CONT("GM: Use routeNodes: ");
  LOG_DEBUG_FOR(int n : ctx->routeNodes, n << " ");
  if (ctx->routeNodes.empty()) {
    if (zoneRelationship == -1) {
      LOG_DEBUG(
          "GM: No routeNodes, but same zone => use tgtNode: " << tgtNodeIdx);
      ctx->routeNodes.push_back(tgtNodeIdx);
    } else {
      LOG_DEBUG("ERROR: No routeNodes srcZ:" << sourceZone
                                             << " tgtZ:" << targetZone);
      return nextStep(source, target);
    }
  }

  // If at DeadEnd then move back towards the connected node
  LOG_DEBUG("GM: Check DEND srcCell:" << std::hex << srcCell << " DEND:"
                                      << GridType::DEND << std::dec);
  if (srcCell & GridType::DEND) {
    int dendIdx = srcCell & 0xffff;
    LOG_DEBUG("GM: src at DEND: " << dendIdx);
    int edgeIdx = m_routingGraph.getEdgeFromDead(dendIdx);
    if (edgeIdx != -1) {
      const auto& edge = m_baseEdges[edgeIdx];
      // Edge goes From -> DeadEnd
      // We are at DeadEnd, so move back towards From
      if (edge.path.size() >= 2) {
        const auto& nxt = edge.path[edge.path.size() - 2];
        LOG_DEBUG("GM:  DEND edge: " << edgeIdx << " " << edge.from << "->"
                                     << edge.to << " move back to " << nxt.first
                                     << "," << nxt.second);
        return nextStep(source, nxt);
      }
    }
    LOG_ERROR("Failed to find edge for DEND:" << dendIdx);
  }

  // If at a Node then search edges to find one going from node
  if (srcCell & GridType::NODE) {
    int srcNode = srcCell & 0xffff;
    LOG_DEBUG("GM: src at Node: " << srcNode);
    int toIdx = (srcNode == ctx->routeNodes[0]) ? ctx->routeNodes[1]
                                                : ctx->routeNodes[0];
    LOG_DEBUG("GM: src: " << srcNode << " to:" << toIdx);

    // Find edge connecting nodeA to nodeB
    for (const auto& [neighbor, edgeIdx] :
         m_routingGraph.forwardConnections[srcNode]) {
      LOG_DEBUG("GM:   Check edge:" << edgeIdx << " neigh: " << neighbor
                                    << " vs to: " << toIdx);
      if (neighbor == toIdx) {
        const auto& edge = m_baseEdges[edgeIdx];
        const auto& nxt = (edge.from == srcNode)
                              ? edge.path[1]
                              : edge.path[edge.path.size() - 2];
        LOG_DEBUG("GM:  edge: " << edgeIdx << " " << edge.from << "->"
                                << edge.to << " nxtPnt:" << nxt.first << ","
                                << nxt.second);
        return nextStep(source, nxt);
      }
    }
    LOG_DEBUG("ERROR: Failed to find edge for node:" << srcNode << "->"
                                                     << toIdx);
  }

  // If at Edge then find the source on the path and move to the next/prev
  if (srcCell & GridType::EDGE) {
    int edgeIdx = srcCell & GridType::EDGE_MASK;
    LOG_DEBUG("GM: src at Edge: " << edgeIdx);
    const auto& edge = m_baseEdges[edgeIdx];

    int index = std::find(edge.path.begin(), edge.path.end(), source) -
                edge.path.begin();
    if (index != edge.path.size()) {
      LOG_DEBUG("GM:   found src: " << source.first << "," << source.second
                                    << " in " << edge.from << "->" << edge.to
                                    << " path sz:" << edge.path.size()
                                    << " at idx: " << index);
      GridType::Point nxt;
      index += (edge.from == ctx->routeNodes[0]) ? -1 : 1;
      if (index < 0) {
        nxt = m_baseNodes[edge.from];
        LOG_DEBUG_CONT("GM:  At path start use fromNode: " << edge.from);
      } else if (index >= edge.path.size()) {
        nxt = edge.toDeadEnd ? m_deadEnds[edge.to] : m_baseNodes[edge.to];
        LOG_DEBUG_CONT("GM:  At path end use toNode: " << edge.to);
      } else {
        nxt = edge.path[index];
        LOG_DEBUG_CONT("GM:  move along path to idx:" << index);
      }
      LOG_DEBUG(" nxtPnt:" << nxt.first << "," << nxt.second);

      // DEBUG: Inspection for 28,16 -> 29,17
      if (source.first == 28 && source.second == 16 && nxt.first == 29 && nxt.second == 17) {
        int c28_17 = m_infoGrid[17][28];
        int c29_16 = m_infoGrid[16][29];
        LOG_DEBUG("DEBUG_INSPECT: 28,17: " << std::hex << c28_17 << " IS_WALL:" << (c28_17 & GridType::WALL));
        LOG_DEBUG("DEBUG_INSPECT: 29,16: " << std::hex << c29_16 << " IS_WALL:" << (c29_16 & GridType::WALL));
      }

      return nextStep(source, nxt);
    }
    LOG_ERROR("Failed to src: " << source.first << "," << source.second
                                << " in path for edge: " << edgeIdx);
    return nextStep(source, target);
  }
  //
  // If on XPND then move towards edge
  // Removed from here to be placed at end of function to allow routeNodes to be populated
  //

  LOG_ERROR("Not on EDGE or NODE");
  return nextStep(source, target);
}

float NavigationGraph::getMoveDirection(Router::RouteCtx* ctx,
                                        GridType::Vec2 from, GridType::Vec2 to,
                                        int type) {
  GridType::Point fromPnt = {from.x / (m_info.mCellWidth * CELL_MULT),
                             from.y / (m_info.mCellHeight * CELL_MULT)};
  GridType::Point toPnt = {to.x / (m_info.mCellWidth * CELL_MULT),
                           to.y / (m_info.mCellHeight * CELL_MULT)};
  LOG_DEBUG("---GETANGLE: from:"
            << from.x << "," << from.y << " to:" << to.x << "," << to.y
            << "  cell: " << m_info.mCellWidth << "x" << m_info.mCellHeight
            << " => " << fromPnt.first << "," << fromPnt.second
            << " to:" << toPnt.first << "," << toPnt.second);

  LOG_DEBUG("---------------to:" << to.x << "," << to.y << " => " << toPnt.first
                                 << "," << toPnt.second);
  LOG_DEBUG("      CTX: F: "
            << ctx->from.first << "," << ctx->from.second
            << " CtxTo: " << ctx->to.first << "," << ctx->to.second
            << " CtxNxt: " << ctx->next.first << "," << ctx->next.second
            << " CtxtDir: " << ctx->curDir << " FPNT: " << fromPnt.first << ","
            << fromPnt.second << " TPNT: " << toPnt.first << "," << toPnt.second
            << " last: " << ((int)ctx->lastRouteType)
            << " ctxType: " << ctx->type << " type: " << type);

  if (ctx->type == type) {
    auto dx = ctx->next.first - fromPnt.first;
    auto dy = ctx->next.second - fromPnt.second;
    if (dx || dy) {
      // Only allow reuse if the next point is adjacent.
      // If it's far away, it implies we are in a fallback mode (e.g. moving to
      // target after error) and we should re-evaluate getNextMove frequently to
      // try to get back on the graph.
      bool isAdjacent = (std::abs(dx) <= 1 && std::abs(dy) <= 1);

      bool allowReuse =
          isAdjacent && ((ctx->reuseInit) ? --ctx->reuseCnt : true);

      if (allowReuse) {
        LOG_DEBUG_CONT("---REUSE (" << ctx->reuseCnt << ") PNT(dxdy: " << dx
                                    << ", " << dy << ") "
                                    << " frm: " << fromPnt.first << ","
                                    << fromPnt.second
                                    << " nxt: " << ctx->next.first << ","
                                    << ctx->next.second);
        ctx->didReuse = true;
        auto nxtDir = computeAngle(dx, dy);
        if (nxtDir != ctx->curDir) {
          LOG_DEBUG("  CHANGE dir " << ctx->curDir << " => " << nxtDir);
          ctx->curDir = nxtDir;
        } else {
          LOG_DEBUG("  SAME dir " << ctx->curDir);
        }
        return ctx->curDir;
      } else {
        LOG_DEBUG("---STOP: REUSE (adj:" << isAdjacent << ")");
        ctx->reuseCnt = ctx->reuseInit;
      }
    }
  }

  ctx->from = fromPnt;
  ctx->to = toPnt;
  ctx->type = type;

  LOG_DEBUG("===GETNEXTMOVE: " << fromPnt.first << "," << fromPnt.second
                               << " TO " << toPnt.first << "," << toPnt.second
                               << " ruse:" << ctx->reuseCnt);
  ctx->next = getNextMove(ctx, fromPnt, toPnt);

  // If next point is same as current, we are at the target cell approx.
  // just return direction to target
  if (ctx->next == fromPnt) {
    ctx->curDir = computeAngle(to.x - from.x, to.y - from.y);
    LOG_DEBUG("==>ANG (SameCell) from:" << from.x << "," << from.y << " TO: " << to.x << "," << to.y << " = " << ctx->curDir);
    return ctx->curDir;
  }

  // Calculate center of next cell in world coords
  // Assuming 8x8 blocks as per getClosest/m_info usages essentially mean 1 "Navigation Gird Cell" = width/height * CELL_MULT world units?
  // Wait, in testLott:
  // info.mCellWidth = 8;
  // fromPnt = {from.x / (m_info.mCellWidth * CELL_MULT), ...}
  // So Cell Size = mCellWidth * CELL_MULT = 64.
  // Center = index * 64 + 32.
  float cellSizeX = m_info.mCellWidth * CELL_MULT;
  float cellSizeY = m_info.mCellHeight * CELL_MULT;
  GridType::Point tgtCell = ctx->next;

#if 0
  // Steering Corner Avoidance:
  // If moving diagonally, check if we are clipping a wall corner.
  // If so, steer towards the SAFE neighbor first to clear the corner.
  int dx = ctx->next.first - fromPnt.first;
  int dy = ctx->next.second - fromPnt.second;
  if (std::abs(dx) == 1 && std::abs(dy) == 1) {
    // Check Horizontal Neighbor
    GridType::Point hNeighbor = {fromPnt.first + dx, fromPnt.second};
    bool hWall = false;
    if (hNeighbor.second >= 0 && hNeighbor.second < m_infoGrid.size() &&
        hNeighbor.first >= 0 && hNeighbor.first < m_infoGrid[0].size()) {
      hWall = (m_infoGrid[hNeighbor.second][hNeighbor.first] & GridType::WALL);
    }

    // Check Vertical Neighbor
    GridType::Point vNeighbor = {fromPnt.first, fromPnt.second + dy};
    bool vWall = false;
    if (vNeighbor.second >= 0 && vNeighbor.second < m_infoGrid.size() &&
        vNeighbor.first >= 0 && vNeighbor.first < m_infoGrid[0].size()) {
      vWall = (m_infoGrid[vNeighbor.second][vNeighbor.first] & GridType::WALL);
    }

    if (hWall && !vWall) {
      // Horizontal is blocked, steer Vertical (to vNeighbor)
      tgtCell = vNeighbor;
      LOG_DEBUG("  STEER AVOID: Horiz blocked, steering to " << tgtCell.first << "," << tgtCell.second);
    } else if (vWall && !hWall) {
      // Vertical is blocked, steer Horizontal (to hNeighbor)
      tgtCell = hNeighbor;
      LOG_DEBUG("  STEER AVOID: Vert blocked, steering to " << tgtCell.first << "," << tgtCell.second);
    }
  }
#endif

  float nextCenterX = (tgtCell.first * cellSizeX) + (cellSizeX * 0.5f);
  float nextCenterY = (tgtCell.second * cellSizeY) + (cellSizeY * 0.5f);

  ctx->curDir = computeAngle(nextCenterX - from.x,
                             nextCenterY - from.y);
  LOG_DEBUG("==>ANG from:" << from.x << "," << from.y
                           << " NXT_CELL: " << ctx->next.first << "," << ctx->next.second
                           << " STEER_CELL: " << tgtCell.first << "," << tgtCell.second
                           << " NXT_CENTER: " << nextCenterX << "," << nextCenterY
                           << " = " << ctx->curDir);
  return ctx->curDir;
}

}  // namespace Routing
}  // namespace DistanceMap
