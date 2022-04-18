#pragma once

#include "GameProxy.h"

namespace marvin {

class Vector2f;
class Bot;

class InfluenceMap {
public:
  InfluenceMap() {
    tiles = new float[1024 * 1024];
  }

  float GetValue(uint16_t x, uint16_t y);
  float GetValue(Vector2f v);

  void AddValue(uint16_t x, uint16_t y, float value);
  void SetValue(uint16_t x, uint16_t y, float value);

  void Clear();
  void Decay(float dt);

  void Update(GameProxy& game, std::vector<Player> enemy_list);

  void CastWeapon(const Map& map, Vector2f from, Vector2f direction, float max_length, float value, Weapon* weapon);
  void CastInfluence(const Map& map, Vector2f from, Vector2f direction, float max_length, float value);

private:
  float* tiles;
};

} // namespace marvin
