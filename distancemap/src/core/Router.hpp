#pragma once

#include <vector>

#include "GridTypes.hpp"
#include <set>

namespace DistanceMap {
namespace Router {

struct RouteCtx {
  int type = -1;
  float curDir;
  GridType::Point from;
  GridType::Point to;
  GridType::Point next;
  std::vector<int> routeNodes;

  // Limit number of times can keep reusing previous direction
  // before re-routing
  // NOTE: For GRAPH routing 0 = infinite. FLOW doesn't allow
  // 0 and resets it to 20
  int reuseInit = 0;
  int reuseCnt = reuseInit;
  bool didReuse = false;

  // Cache for each type
  int routeSrcNodeIdx = -1;
  int routeTgtNodeIdx = -1;

  int routeSrcNodeIdx2 = -1;
  int routeTgtEdgeIdx = -1;

  int routeSrcEdgeIdx = -1;
  int routeTgtNodeIdx2 = -1;

  int routeSrcEdgeIdx2 = -1;
  int routeTgtEdgeIdx2 = -1;

  enum class RouteType { None, NodeToNode, NodeToEdge, EdgeToNode, EdgeToEdge };
  RouteType lastRouteType = RouteType::None;

  // Walkabout Specific
  struct WalkaboutState {
    int levelIdx = -1;
    int zoneIdx = -1;
    std::set<int> visitedBaseNodes;
    int currentBaseTargetNode = -1;
    GridType::Point lockedTarget = {-1, -1};
    bool hasLockedTarget = false;

    // DeadEnd Support
    int currentEdgeIdx = -1;
    bool allowDeadEnds = true;
    bool targetIsDeadEnd = false;
    std::set<int> visitedDeadEnds;

    // Edge Traversal Logic
    std::set<int> visitedEdges;
    int currentPathIdx = -1;

    // Wander state
    bool allowWander = true;
    float wanderOffset = 0.0f;
    float wanderTarget = 0.0f;
    float wanderTimer = 0.0f;

    // Tube Logic
    float laneBias = 0.0f; // -1.0 to 1.0 (Left to Right)
    GridType::Vec2 debugTubeTarget = {-1.0f, -1.0f};
    GridType::Vec2 smoothedTarget = {-1.0f, -1.0f};
  };
  WalkaboutState walkabout;

  // Perlin Wander Specific
  struct PerlinState {
    float lastAngle = -1.0f;
    float perlinTime = 0.0f;
    float noiseSeed = -1.0f; // -1.0f means uninitialized

    int state = 0; // 0 = WANDERING, 1 = ESCAPING
    int currentZoneId = -1;
    float boredomTimer = 0.0f;
    int targetZoneId = -1;

    // Agent-specific randomization
    float boredomThreshold = 15.0f;
    float perlinSpeed = 0.5f;

    // Dead-End Escape Lock
    bool isLocked = false;
    float lockedAngle = -1.0f;
  };
  PerlinState perlin;
};

struct Info {
  int mCaveWidth = 2;
  int mCaveHeight = 2;
  int mBorderWidth = 1;
  int mBorderHeight = 1;
  int mCellWidth = 1;
  int mCellHeight = 1;
  int mStartCellX = 0;
  int mStartCellY = 0;
};

} // namespace Router
} // namespace DistanceMap
