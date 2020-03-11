#pragma once

#include "Vector2f.h"

namespace marvin {

class Map;

struct CastResult {
  bool hit;
  Vector2f position;
  Vector2f normal;
};

bool RayBoxIntersect(Vector2f origin, Vector2f direction, Vector2f box_pos, Vector2f box_extent, float* dist, Vector2f* norm);
double BoxPointDistance(Vector2f box_pos, Vector2f box_extent, Vector2f point);

CastResult RayCast(const Map& map, Vector2f from, Vector2f direction, double max_length);

} // namespace marvin
