#ifndef GD_DISTANCE_MAP_H
#define GD_DISTANCE_MAP_H

#include <godot_cpp/classes/object.hpp>
#include <godot_cpp/classes/tile_map_layer.hpp>
#include <map>
#include <memory>

#include "Debug.h"
#include "DistanceMapCore.hpp"
#include "GDDistanceMapDLL.hpp"
#include "GDTracker.hpp"
#include "NavigationAPI.hpp"
#include "Router.hpp"

namespace godot {
class GDDISTANCE_MAP_API GDDistanceMap : public RefCounted {
  GDCLASS(GDDistanceMap, RefCounted)

 protected:
  static void _bind_methods();

  DistanceMap::Router::Info info;
  godot::Vector2i mFloor;
  godot::Vector2i mWall;

  DistanceMap::DistanceMapCore core;
  GDTracker* pTracker = nullptr;

 public:
  GDDistanceMap();
  ~GDDistanceMap();

  GDDistanceMap* setBorderSize(godot::Vector2i sz);
  GDDistanceMap* setCaveSize(godot::Vector2i sz);
  GDDistanceMap* setCellSize(godot::Vector2i sz);
  GDDistanceMap* setFloor(godot::Vector2i floor);
  GDDistanceMap* setTracker() {
    pTracker = GDTracker::getInstance();
    LOG_INFO("## SET TRACKER " << pTracker);
    return this;
  }

  void make_it(TileMapLayer* pTileMap, int layer);

  float getMoveAngle(godot::Node* node, godot::Vector2 from, godot::Vector2 to,
                     int type);

 private:
  godot::Vector2i getMapPos(int x, int y);
  std::map<godot::Vector2i, int> mCoordsToTile;
  std::unique_ptr<DistanceMap::NavigationAPI> mpNavigator;
};

}  // namespace godot

#endif
