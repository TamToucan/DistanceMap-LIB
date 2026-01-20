#include "GDDistanceMap.hpp"

#include <gdextension_interface.h>

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/godot.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <vector>

#include "Cave.h"
#include "Debug.h"
#include "GDCave.hpp"
#include "GDTracker.hpp"
#include "TileTypes.h"

using namespace godot;
using namespace DistanceMap;

void GDDistanceMap::_bind_methods() {
  ClassDB::bind_method(D_METHOD("set_floor", "floorTileCoords"),
                       &GDDistanceMap::setFloor);
  ClassDB::bind_method(D_METHOD("set_border_size", "borderSize"),
                       &GDDistanceMap::setBorderSize);
  ClassDB::bind_method(D_METHOD("set_cave_size", "caveSize"),
                       &GDDistanceMap::setCaveSize);
  ClassDB::bind_method(D_METHOD("set_cell_size", "cellSize"),
                       &GDDistanceMap::setCellSize);
  ClassDB::bind_method(D_METHOD("make_distance_map", "pTileMap", "layer"),
                       &GDDistanceMap::make_it);
  ClassDB::bind_method(D_METHOD("get_move", "node", "from", "to", "type"),
                       &GDDistanceMap::getMoveAngle);
  ClassDB::bind_method(D_METHOD("set_tracker"), &GDDistanceMap::setTracker);
}

GDDistanceMap::GDDistanceMap() {
  mFloor = Vector2i(0, 0);
  mWall = Vector2i(0, 1);
  // This is a pain. Godot has TileMapLayer so don't have the original "tiles"
  // just the coords. So create a reverse mapping of coords to tiles
  for (int t = 0; t < Cave::TileName::TILE_COUNT; ++t) {
    auto coords = GDCave::GDCave::getAtlasCoords(t);
    mCoordsToTile[coords] = t;
  }
}

GDDistanceMap::~GDDistanceMap() {}

GDDistanceMap* GDDistanceMap::setBorderSize(godot::Vector2i sz) {
  info.mBorderWidth = sz.width;
  info.mBorderHeight = sz.height;
  LOG_INFO("SET BORDER: " << info.mBorderWidth << "x" << info.mBorderHeight);
  return this;
}

GDDistanceMap* GDDistanceMap::setCaveSize(godot::Vector2i sz) {
  info.mCaveWidth = sz.width;
  info.mCaveHeight = sz.height;
  LOG_INFO("SET CAVE: " << info.mCaveWidth << "x" << info.mCaveHeight);
  return this;
}

GDDistanceMap* GDDistanceMap::setCellSize(godot::Vector2i sz) {
  info.mCellWidth = sz.width;
  info.mCellHeight = sz.height;
  LOG_INFO("SET CELL: " << info.mCellWidth << "x" << info.mCellHeight);
  return this;
}

GDDistanceMap* GDDistanceMap::setFloor(godot::Vector2i floor) {
  mFloor = floor;
  LOG_INFO("SET FLOOR: " << mFloor.x << "x" << mFloor.y);
  return this;
}

void GDDistanceMap::make_it(TileMapLayer* pTileMap, int layer) {
  LOG_INFO("====================================================="
           << std::endl
           << "##Make DistanceMap tileMap: " << info.mCaveWidth << "x" << info.mCaveHeight
           << " cell:" << info.mCaveWidth << "," << info.mCellHeight
           << " border:" << info.mBorderWidth << "," << info.mBorderHeight);

  const int COPY_W = info.mCaveWidth + 2;
  const int COPY_H = info.mCaveHeight + 2;
  // Initialize floorGrid with solid wall so [][] works and have 1x1 border
  std::vector<std::vector<int>> grid(COPY_H, std::vector<int>(COPY_W, 1));
  for (int y = 0; y < info.mCaveHeight; ++y) {
    for (int x = 0; x < info.mCaveWidth; ++x) {
      Vector2i coords = getMapPos(x, y);
      coords = pTileMap->get_cell_atlas_coords(coords);
      int v = Cave::Cave::isEmpty(mCoordsToTile[coords]) ? 0 : 1;
      // This never writes the 1x1 border edge which has been init'ed to 1
      grid[y + 1][x + 1] = v;
    }
  }

  core.initialize(grid, info);
  mpNavigator = core.makeNavigator(NavigatorType::GRAPH);
}

// ===========================================================================

Vector2i GDDistanceMap::getMapPos(int x, int y) {
  return Vector2i(info.mBorderWidth + x * info.mCellWidth,
                  info.mBorderHeight + y * info.mCellHeight);
}

/////////////////////////////////////////////////////////

static void untrack_cb(godot::Node* id, void* ctx) {
  if (ctx) {
    LOG_DEBUG("===UNTRACK id:" << id << " CTX: " << ctx);
    DistanceMap::Router::RouteCtx* routeCtx =
        static_cast<DistanceMap::Router::RouteCtx*>(ctx);
    delete routeCtx;
  }
}

float GDDistanceMap::getMoveAngle(godot::Node* id, godot::Vector2 from,
                                  godot::Vector2 to, int type) {
  DistanceMap::Router::RouteCtx* ctx =
      pTracker ? pTracker->getContext<DistanceMap::Router::RouteCtx>(id)
               : nullptr;

  if (!ctx) {
    LOG_DEBUG("===TRACK => CREATE CONTEXT");
    ctx = new DistanceMap::Router::RouteCtx();
    pTracker->setContext<DistanceMap::Router::RouteCtx>(id, ctx);
    pTracker->set_untrack_callback(untrack_cb);
  }

  GridType::Vec2 fromV(from.x, from.y);
  GridType::Vec2 toV(to.x, to.y);

  return core.getMoveAngle(mpNavigator, ctx, fromV, toV, type);
}
