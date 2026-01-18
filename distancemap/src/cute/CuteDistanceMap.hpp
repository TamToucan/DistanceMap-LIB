#pragma once

#include <cute.h>

#include <vector>

#include "DistanceMapCore.hpp"
#include "Router.hpp"

class CuteDistanceMap {
 public:
  CuteDistanceMap() = default;
  ~CuteDistanceMap() = default;

  // Grid is 0 = empty, 1 = solid
  void initialize(const std::vector<std::vector<int>>& grid,
                  const DistanceMap::Router::Info& info,
                  DistanceMap::NavigatorType type);

  DistanceMap::DistanceMapCore& core() { return mCore; };

 private:
  DistanceMap::DistanceMapCore mCore;
};
