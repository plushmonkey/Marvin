#include "NodeProcessor.h"

namespace marvin {
namespace path {

NodeConnections NodeProcessor::FindEdges(Node* node, Node* start, Node* goal, float radius) {
  NodeConnections connections;

  connections.count = 0;

  NodePoint base_point = GetPoint(node);

  for (int y = -1; y <= 1; ++y) {
    for (int x = -1; x <= 1; ++x) {
      if (x == 0 && y == 0) {
        continue;
      }

      uint16_t world_x = base_point.x + x;
      uint16_t world_y = base_point.y + y;

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

Node* NodeProcessor::GetNode(NodePoint point) {
  if (point.x >= 1024 || point.y >= 1024) {
    return nullptr;
  }

  std::size_t index = point.y * 1024 + point.x;
  Node* node = &nodes_[index];

  if (!(node->flags & NodeFlag_Initialized)) {
    node->g = node->f = 0.0f;
    node->flags = NodeFlag_Initialized;
    node->parent = nullptr;
  }

  return &nodes_[index];
}

} // namespace path
} // namespace marvin
