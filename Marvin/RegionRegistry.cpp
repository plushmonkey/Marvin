#include "RegionRegistry.h"

#include <vector>

#include "Map.h"

namespace marvin {

void FloodFillEmptyRegion(const Map& map, const MapCoord& coord,
                          RegionRegistry& registry, RegionIndex region_index) {
  if (map.IsSolid(coord.x, coord.y)) return;

  registry.Insert(coord, region_index);

  std::vector<MapCoord> stack;

  stack.push_back(coord);

  while (!stack.empty()) {
    MapCoord current = stack.back();
    stack.pop_back();

    const MapCoord west(current.x - 1, current.y);
    const MapCoord east(current.x + 1, current.y);
    const MapCoord north(current.x, current.y - 1);
    const MapCoord south(current.x, current.y + 1);

    if (!map.IsSolid(west.x, west.y)) {
      if (!registry.IsRegistered(west)) {
        registry.Insert(west, region_index);
        stack.push_back(west);
      }
    }

    if (!map.IsSolid(east.x, east.y)) {
      if (!registry.IsRegistered(east)) {
        registry.Insert(east, region_index);
        stack.push_back(east);
      }
    }

    if (!map.IsSolid(north.x, north.y)) {
      if (!registry.IsRegistered(north)) {
        registry.Insert(north, region_index);
        stack.push_back(north);
      }
    }

    if (!map.IsSolid(south.x, south.y)) {
      if (!registry.IsRegistered(south)) {
        registry.Insert(south, region_index);
        stack.push_back(south);
      }
    }
  }
}

std::unique_ptr<RegionRegistry> RegionRegistry::Create(const Map& map) {
  // make_unique doesn't have access to the private constructor so inherit
  // RegionRegistry and construct it with the default generated constructor
  struct make_unique_enabler : public RegionRegistry {};
  auto registry = std::make_unique<make_unique_enabler>();

  for (uint16_t y = 0; y < 1024; ++y) {
    for (uint16_t x = 0; x < 1024; ++x) {
      MapCoord coord(x, y);

      if (!map.IsSolid(x, y)) {
        // If the current coord is empty and hasn't been inserted into region
        // map then create a new region and flood fill it
        if (!registry->IsRegistered(coord)) {
          auto region_index = registry->CreateRegion();

          FloodFillEmptyRegion(map, coord, *registry, region_index);
        }
      }
    }
  }

  return registry;
}

RegionRegistry::RegionRegistry() : region_count_(0) {}

bool RegionRegistry::IsRegistered(MapCoord coord) const {
  return coord_regions_.find(coord) != coord_regions_.end();
}

void RegionRegistry::Insert(MapCoord coord, RegionIndex index) {
  coord_regions_[coord] = index;
}

RegionIndex RegionRegistry::CreateRegion() { return region_count_++; }

bool RegionRegistry::IsConnected(MapCoord a, MapCoord b) const {
  auto first = coord_regions_.find(a);
  auto second = coord_regions_.find(b);

  // Only one needs to be checked for invalid because the second line will
  // fail if one only one is invalid
  if (first == coord_regions_.end()) return false;

  return first->second == second->second;
}

}  // namespace marvin
