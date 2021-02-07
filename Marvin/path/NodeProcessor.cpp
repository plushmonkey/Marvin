#include "NodeProcessor.h"

namespace marvin {
namespace path {

NodeConnections NodeProcessor::FindEdges(Node* node, Node* start, Node* goal, float radius) {
  NodeConnections connections;

  connections.count = 0;

  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      if (x == 0 && y == 0)
        continue;
      uint16_t world_x = node->point.x + x;
      uint16_t world_y = node->point.y + y;

      if (map_.IsSolid(world_x, world_y))
        continue;

      Vector2f check_pos(world_x + 0.5f, world_y + 0.5f);

      if (!map_.CanOccupy(check_pos, radius))
        continue;

      NodePoint point(world_x, world_y);
      Node* current = GetNode(point);

      if (current != nullptr) {
        if (map_.GetTileId(point.x, point.y) == kSafeTileId) {
          current->weight = 10.0f;
        }
        connections.neighbors[connections.count++] = current;

        if (connections.count >= 8) {
          return connections;
        }
      }
    }
  }

  return connections;
}

void NodeProcessor::ResetNodes() {
  for (std::size_t i = 0; i < 1024 * 1024; ++i) {
    Node* node = &nodes_[i];

    node->closed = false;
    node->openset = false;
    node->g = node->h = node->f = 0;
    node->parent = nullptr;
    node->rotations = 0;
  }
}

Node* NodeProcessor::GetNode(NodePoint point) {
  if (point.x >= 1024 || point.y >= 1024) {
    return nullptr;
  }

  std::size_t index = point.y * 1024 + point.x;
  Node* node = &nodes_[index];

  if (node->point.x != point.x) {
    node->point = point;
    node->g = node->h = node->f = 0;
    node->closed = false;
    node->openset = false;
    node->parent = nullptr;
  }

  return &nodes_[index];
}

} // namespace path
} // namespace marvin
