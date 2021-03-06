#include "RayCaster.h"

#include "Bot.h"
#include "Debug.h"
#include "Map.h"
#include <algorithm>

extern std::unique_ptr<marvin::Bot> g_Bot;

namespace marvin {

float BoxPointDistance(Vector2f box_pos, Vector2f box_extent, Vector2f p) {
  Vector2f bmin = box_pos;
  Vector2f bmax = box_pos + box_extent;

  float dx = std::max(std::max(bmin.x - p.x, 0.0f), p.x - bmax.x);
  float dy = std::max(std::max(bmin.y - p.y, 0.0f), p.y - bmax.y);

  return std::sqrt(dx * dx + dy * dy);
}

bool LineBoxIntersect(Vector2f point, Vector2f direction, Vector2f box_pos, Vector2f box_extent, float* dist,
                      Vector2f* norm) {
  if (RayBoxIntersect(point, direction, box_pos, box_extent, dist, norm)) {
    return true;
  }

  return RayBoxIntersect(point, -direction, box_pos, box_extent, dist, norm);
}

bool RayBoxIntersect(Vector2f origin, Vector2f direction, Vector2f box_pos, Vector2f box_extent, float* dist,
                     Vector2f* norm) {
  Vector2f recip(1.0f / direction.x, 1.0f / direction.y);
  Vector2f lb = box_pos + Vector2f(0, box_extent.y);
  Vector2f rt = box_pos + Vector2f(box_extent.x, 0);

  float t1 = (float)((lb.x - origin.x) * recip.x);
  float t2 = (float)((rt.x - origin.x) * recip.x);
  float t3 = (float)((lb.y - origin.y) * recip.y);
  float t4 = (float)((rt.y - origin.y) * recip.y);

  using std::max;
  using std::min;

  float tmin = max(min(t1, t2), min(t3, t4));
  float tmax = min(max(t1, t2), max(t3, t4));

  bool intersected = false;
  float t;

  if (tmax < 0) {
    t = tmax;
  } else if (tmin > tmax) {
    t = tmax;
  } else {
    intersected = true;
    t = tmin;

    if (norm) {
      if (t == t1) {
        *norm = Vector2f(-1, 0);
      } else if (t == t2) {
        *norm = Vector2f(1, 0);
      } else if (t == t3) {
        *norm = Vector2f(0, 1);
      } else if (t == t4) {
        *norm = Vector2f(0, -1);
      } else {
        *norm = Vector2f(0, 0);
      }
    }
  }

  if (dist) {
    *dist = t;
  }

  return intersected;
}

CastResult RayCast(const Map& map, Vector2f from, Vector2f direction, float max_length, bool debug) {
  static const Vector2f kDirections[] = {Vector2f(0, 0), Vector2f(1, 0), Vector2f(-1, 0), Vector2f(0, 1),
                                         Vector2f(0, -1)};

  CastResult result = {0};
  float closest_distance = std::numeric_limits<float>::max();
  Vector2f closest_normal;

  result.distance = max_length;

  for (float i = 0; i < max_length + 3; ++i) {
    Vector2f current = from + direction * i;

    current.x = std::floor(current.x);
    current.y = std::floor(current.y);

    for (Vector2f check_direction : kDirections) {
      Vector2f check = current + check_direction;

      if (!map.IsSolid((unsigned short)check.x, (unsigned short)check.y))
        continue;

      if (debug) {
        RenderWorldLine(g_Bot->GetGame().GetPosition(), check, check + Vector2f(1, 1), RGB(0, 255, 255));
        RenderWorldLine(g_Bot->GetGame().GetPosition(), check + Vector2f(0, 1), check + Vector2f(1, 0), RGB(0, 255, 255));
      }

      float dist;
      Vector2f normal;

      if (RayBoxIntersect(from, direction, check, Vector2f(1, 1), &dist, &normal)) {
        if (dist < closest_distance && dist >= 0) {
          closest_distance = dist;
          closest_normal = normal;
        }
      }
    }

    // Break out once the iteration loops beyond possibly closer tiles
    if (i > std::floor(closest_distance) + 2) {
      result.hit = true;
      result.normal = closest_normal;
      result.distance = closest_distance;
      result.position = from + direction * closest_distance;

      if (debug) {
        RenderWorldLine(g_Bot->GetGame().GetPosition(), result.position, result.position + result.normal, RGB(200, 0, 150));
      }
      break;
    }
  }

  return result;
}

} // namespace marvin
