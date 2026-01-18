#pragma once

#include "DistanceMapDLL.hpp"
#include "GridTypes.hpp"
#include "Router.hpp"

namespace DistanceMap {

// In Godot I have 64x64 tiles, but cut up into 8x8 cells for collision
// detection and when setting a tile map cell it uses that 8x8 cut up
// graphic to set the cell.
// However this meant I have a bunch of *8 in the code. This is just
// so I can see where I am using it.
constexpr int CELL_MULT = 8;

class DISTANCEMAP_API NavigationAPI {
 public:
  virtual ~NavigationAPI() = default;

  virtual float getMoveDirection(Router::RouteCtx* ctx, GridType::Vec2 from,
                                 GridType::Vec2 to, int type) = 0;

  // Calculates new position with wall sliding
  virtual GridType::Vec2 resolveMove(const GridType::Vec2& currentPos, float angle, float distance);

  // Checks if a world position is valid (not inside a wall)
  virtual bool validateMove(const GridType::Vec2& pos);

 protected:
  // Info grid must be kept around. It will probably be from Graph so that's ok
  const GridType::Grid& m_infoGrid;
  // Router::Info is often temp, so copy it
  const Router::Info m_info;
  const int m_gridWidth = 0;
  const int m_gridHeight = 0;
  const int m_cellWidth = 1;
  const int m_cellHeight = 1;
  NavigationAPI(const GridType::Grid& infoGrid, const Router::Info& info);
};

}  // namespace DistanceMap
