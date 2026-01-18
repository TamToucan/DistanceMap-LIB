#include "CuteDistanceMap.hpp"

#include <cute.h>

#include "CuteDistanceMap.hpp"

void CuteDistanceMap::CuteDistanceMap::initialize(const std::vector<std::vector<int>>& grid,
                                                  const DistanceMap::Router::Info& info,
                                                  DistanceMap::NavigatorType type) {
  mCore.initialize(grid, info, type);
}
