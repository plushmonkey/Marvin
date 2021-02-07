#pragma once

#include <memory>
#include <string>
#include <vector>

#include "Types.h"
#include "Vector2f.h"

namespace marvin {

constexpr std::size_t kMapExtent = 1024;

using TileId = u8;
using TileData = std::vector<TileId>;

constexpr TileId kSafeTileId = 171;

class Map {
public:
  Map(const TileData& tile_data);

  bool IsSolid(TileId id) const;
  bool IsSolid(u16 x, u16 y) const;
  bool IsSolid(const Vector2f& position) const;
  TileId GetTileId(u16 x, u16 y) const;
  TileId GetTileId(const Vector2f& position) const;

  bool CanOccupy(const Vector2f& position, float radius) const;

  static std::unique_ptr<Map> Load(const char* filename);
  static std::unique_ptr<Map> Load(const std::string& filename);

private:
  TileData tile_data_;
};

} // namespace marvin
