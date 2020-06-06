#include "Pathfinder.h"

#include <algorithm>
#include <cmath>

#include "../RayCaster.h"
#include "../Bot.h"

namespace marvin {
namespace path {

Vector2f ClosestWall(const Map& map, Vector2f pos, int search) {
  float closest_dist = std::numeric_limits<float>::max();
  Vector2f closest;

  Vector2f base(std::floor(pos.x), std::floor(pos.y));
  for (int y = -search; y <= search; ++y) {
    for (int x = -search; x <= search; ++x) {
      Vector2f current = base + Vector2f((float)x, (float)y);

      if (!map.IsSolid((unsigned short)current.x, (unsigned short)current.y)) {
        continue;
      }

      float dist = BoxPointDistance(current, Vector2f(1, 1), pos);

      if (dist < closest_dist) {
        closest_dist = dist;
        closest = current;
      }
    }
  }
  
  return closest;
}

bool IsPassablePath(const Map& map, Vector2f from, Vector2f to, float radius) {
  const Vector2f direction = Normalize(to - from);
  const Vector2f side = Perpendicular(direction) * radius;
  const float distance = from.Distance(to);

  CastResult cast_center = RayCast(map, from, direction, distance);
  CastResult cast_side1 = RayCast(map, from + side, direction, distance);
  CastResult cast_side2 = RayCast(map, from - side, direction, distance);

  return !cast_center.hit && !cast_side1.hit && !cast_side2.hit;
}

NodePoint ToNodePoint(const Vector2f v) {
  NodePoint np;

  np.x = (uint16_t)v.x;
  np.y = (uint16_t)v.y;

  return np;
}

float Euclidean(const Node* from, const Node* to) {
  float dx = static_cast<float>(from->point.x - to->point.x);
  float dy = static_cast<float>(from->point.y - to->point.y);

  return std::sqrt(dx * dx + dy * dy);
}

Pathfinder::Pathfinder(std::unique_ptr<NodeProcessor> processor)
    : processor_(std::move(processor)) {}

std::vector<Vector2f> Pathfinder::FindPath(const Vector2f& from,
                                           const Vector2f& to, float radius) {
  std::vector<Vector2f> path;

  processor_->ResetNodes();

  Node* start = processor_->GetNode(ToNodePoint(from));
  Node* goal = processor_->GetNode(ToNodePoint(to));

  if (start == nullptr || goal == nullptr) {
    return path;
  }

  openset_.Clear();
  openset_.Push(start);

  while (!openset_.Empty()) {
    Node* node = openset_.Pop();

    if (node == goal) {
      break;
    }

    node->closed = true;

    NodeConnections connections = processor_->FindEdges(node, start, goal, radius);

    for (std::size_t i = 0; i < connections.count; ++i) {
      Node* edge = connections.neighbors[i];
      float cost = node->g + edge->weight;

      if (edge->closed && cost < edge->g) {
        edge->closed = false;
      }

      if (edge->openset && cost < edge->g) {
        edge->g = cost;
        edge->f = edge->g + edge->h;
        edge->parent = node;
        openset_.Update();
      } else if (!edge->openset && !edge->closed) {
        edge->g = cost;
        edge->h = Euclidean(goal, edge);
        edge->f = edge->g + edge->h;
        edge->parent = node;
        openset_.Push(edge);
        edge->openset = true;
      }
    }
  }

  // Construct path backwards from goal node
  std::vector<NodePoint> points;
  Node* current = goal;
  while (current != nullptr && current != start) {
    points.push_back(current->point);
    current = current->parent;
  }

  // Reverse and store as vector
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::size_t index = points.size() - i - 1;
    Vector2f pos(points[index].x, points[index].y);

    path.push_back(pos);
  }

  return path;
}

std::vector<Vector2f> Pathfinder::SmoothPath(const std::vector<Vector2f>& path,
                                             const Map& map,
                                             float ship_radius) {
  std::vector<Vector2f> result;

  // How far away it should try to push the path from walls
  float push_distance = ship_radius * 1.5f;

  result.resize(path.size());

  for (std::size_t i = 0; i < path.size(); ++i) {
    Vector2f current = path[i] + Vector2f(0.5, 0.5);
    Vector2f closest =
        ClosestWall(map, current, (int)std::ceil(push_distance + 1));
    Vector2f new_pos = current;

    if (closest != Vector2f(0, 0)) {
      // Attempt to push the path outward from the wall
      // TODO: iterative box penetration push

      Vector2f center = closest + Vector2f(0.5, 0.5);
      Vector2f direction = Normalize(center - current);
      CastResult cast_result = RayCast(map, current, direction, push_distance);

      if (cast_result.hit) {
        Vector2f hit = cast_result.position;
        float dist = hit.Distance(current);
        float force = push_distance - dist;

        new_pos = current + Normalize(current - hit) * force;
      }
    }

    if (current != new_pos) {
      // Make sure the new node is in line of sight
      if (!IsPassablePath(map, current, new_pos, ship_radius)) {
        new_pos = current;
      }
    }

    result[i] = new_pos;
  }

  if (result.size() <= 2) return result;

  std::vector<Vector2f> minimum;
  minimum.reserve(result.size());

  Vector2f prev = result[0];
  for (std::size_t i = 1; i < result.size(); ++i) {
    Vector2f curr = result[i];
    Vector2f direction = Normalize(curr - prev);
    Vector2f side = Perpendicular(direction) * ship_radius;
    float dist = prev.Distance(curr);

    CastResult cast_center = RayCast(map, prev, direction, dist);
    CastResult cast_side1 = RayCast(map, prev + side, direction, dist);
    CastResult cast_side2 = RayCast(map, prev - side, direction, dist);

    if (cast_center.hit || cast_side1.hit || cast_side2.hit) {
      if (minimum.size() > result.size()) {
        minimum = result;
        break;
      }

      if (!minimum.empty() && result[i - 1] != minimum.back()) {
        minimum.push_back(result[i - 1]);
        prev = minimum.back();
        i--;
      } else {
        minimum.push_back(result[i]);
        prev = minimum.back();
      }
    }
  }

  minimum.push_back(result.back());

  result = minimum;
  return result;
}

}  // namespace path
}  // namespace marvin
