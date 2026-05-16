#pragma once

/**
 * @file CuteDistanceMap.hpp
 * @brief Cute Framework wrapper around DistanceMap::DistanceMapCore.
 * @details Adds a Cute-friendly facade so the game can construct a navigation
 * graph in one call and access the underlying core via core(). No Cute-specific
 * behaviour beyond the include — kept thin to mirror the CuteCave wrapper.
 */

#include <cute.h>

#include <vector>

#include "DistanceMapCore.hpp"

/**
 * @class CuteDistanceMap
 * @brief Cute wrapper: owns one DistanceMapCore for a level.
 */
class CuteDistanceMap {
 public:
  CuteDistanceMap() = default;
  ~CuteDistanceMap() = default;

  /// Build the navigation graph from an occupancy grid (0=empty, 1=solid).
  void initialize(const std::vector<std::vector<int>>& grid);

  /// Mutable access to the underlying core (Graph, routing, etc.).
  DistanceMap::DistanceMapCore& core() { return mCore; };

 private:
  DistanceMap::DistanceMapCore mCore;
};
