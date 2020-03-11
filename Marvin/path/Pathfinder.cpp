#include "Pathfinder.h"

#include <algorithm>
#include <cmath>

namespace marvin {
namespace path {

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
                                           const Vector2f& to) {
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

    NodeConnections connections = processor_->FindEdges(node, start, goal);

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

}  // namespace path
}  // namespace marvin
