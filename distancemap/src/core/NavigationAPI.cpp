#include "NavigationAPI.hpp"

#include <cmath>

namespace DistanceMap {

NavigationAPI::NavigationAPI(const GridType::Grid& infoGrid, const Router::Info& info)
    : m_infoGrid(infoGrid),
      m_info(info),
      m_gridWidth(infoGrid.size()),
      m_gridHeight(infoGrid[0].size()),
      m_cellWidth(info.mCellWidth),
      m_cellHeight(info.mCellHeight) {
}

bool NavigationAPI::validateMove(const GridType::Vec2& pos) {
  int cellX = (int)(pos.x / (m_cellWidth * CELL_MULT));
  int cellY = (int)(pos.y / (m_cellHeight * CELL_MULT));
  if (cellY < 0 || cellY >= m_gridHeight ||
      cellX < 0 || cellX >= m_gridWidth) {
    return false;  // Out of bounds is invalid
  }

  // Check if Wall
  if (m_infoGrid[cellY][cellX] & GridType::WALL) {
    return false;
  }
  return true;
}

GridType::Vec2 NavigationAPI::resolveMove(const GridType::Vec2& currentPos, float angle, float distance) {
  float rad = angle * 3.14159265f / 180.0f;
  float dx = std::cos(rad) * distance;
  float dy = std::sin(rad) * distance;

  GridType::Vec2 nextPos;
  nextPos.x = currentPos.x + dx;
  nextPos.y = currentPos.y + dy;

  // Try full move
  if (validateMove(nextPos)) {
    return nextPos;
  }

  // Try Slide X
  GridType::Vec2 slideX = currentPos;
  slideX.x += dx;
  if (std::abs(dx) > 0.001f && validateMove(slideX)) {
    return slideX;
  }

  // Try Slide Y
  GridType::Vec2 slideY = currentPos;
  slideY.y += dy;
  if (std::abs(dy) > 0.001f && validateMove(slideY)) {
    return slideY;
  }

  // Blocked
  return currentPos;
}

}  // namespace DistanceMap
