#include "NavigationAPI.hpp"

#include <cmath>

namespace DistanceMap {

NavigationAPI::NavigationAPI(const GridType::Grid &infoGrid,
                             const Router::Info &info)
    : m_infoGrid(infoGrid), m_info(info), m_gridWidth(infoGrid[0].size()),
      m_gridHeight(infoGrid.size()), m_cellWidth(info.mCellWidth),
      m_cellHeight(info.mCellHeight) {}

bool NavigationAPI::validateMove(const GridType::Vec2 &pos) {
  int cellX = (int)(pos.x / (m_cellWidth * CELL_MULT));
  int cellY = (int)(pos.y / (m_cellHeight * CELL_MULT));
  if (cellY < 0 || cellY >= m_gridHeight || cellX < 0 || cellX >= m_gridWidth) {
    return false; // Out of bounds is invalid
  }

  // Check if Wall
  if (m_infoGrid[cellY][cellX] & GridType::WALL) {
    return false;
  }
  return true;
}

GridType::Vec2 NavigationAPI::resolveMove(const GridType::Vec2 &currentPos,
                                          float angle, float distance) {
  float rad = angle * 3.14159265f / 180.0f;
  float dx = std::cos(rad) * distance;
  float dy = std::sin(rad) * distance;
  GridType::Vec2 nextPos;
  nextPos.x = currentPos.x + dx;
  nextPos.y = currentPos.y + dy;

  // Bypass the abstract, restrictive 64x64 binary grid collision.
  // Game.cpp will apply this full move vector, and Level::resolveCollision (the
  // physical simulator) will perfectly handle tangent pushing and proper agent
  // cornering on sloped polygons.
  return nextPos;
}

bool NavigationAPI::checkBoundary(GridType::Point &pos) const {
  // Check if in a wall
  int srcCell = m_infoGrid[pos.second][pos.first];

  if (srcCell & GridType::WALL) {
    // If boundary then use dir to move ouit of wall
    if (srcCell & GridType::BOUNDARY) {
      int dirIdx = srcCell & GridType::DIR_MASK;
      pos.first += GridType::directions8[dirIdx].first;
      pos.second += GridType::directions8[dirIdx].second;
    }

    // If still in wall then error
    srcCell = m_infoGrid[pos.second][pos.first];
    if (srcCell & GridType::WALL) {

      return false;
    }
  }
  return true;
}

} // namespace DistanceMap
