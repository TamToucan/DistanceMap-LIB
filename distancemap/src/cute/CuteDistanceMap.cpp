#include "CuteDistanceMap.hpp"

#include <cute.h>

#include "CuteDistanceMap.hpp"

void CuteDistanceMap::CuteDistanceMap::initialize(const std::vector<std::vector<int>>& grid,
                                                  const DistanceMap::Router::Info& info) {
  mCore.initialize(grid, info);
}
