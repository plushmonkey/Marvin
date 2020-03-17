#pragma once

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <unordered_map>

#include "Hash.h"
#include "Vector2f.h"

namespace marvin {

class Map;

using RegionIndex = std::size_t;

struct MapCoord {
  uint16_t x;
  uint16_t y;

  MapCoord(uint16_t x, uint16_t y) : x(x), y(y) {}
  MapCoord(Vector2f vec) : x((uint16_t)vec.x), y((uint16_t)vec.y) {}

  bool operator==(const MapCoord& other) const {
    return x == other.x && y == other.y;
  }
};

}  // namespace marvin

MAKE_HASHABLE(marvin::MapCoord, t.x, t.y);

namespace marvin {

class RegionRegistry {
 public:
  static std::unique_ptr<RegionRegistry> Create(const Map& map);

  bool IsConnected(MapCoord a, MapCoord b) const;

  bool IsRegistered(MapCoord coord) const;
  void Insert(MapCoord coord, RegionIndex index);

  RegionIndex CreateRegion();

 private:
  RegionRegistry();

  std::unordered_map<MapCoord, RegionIndex> coord_regions_;
  RegionIndex region_count_;
};

}  // namespace marvin
