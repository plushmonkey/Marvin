#pragma once

#include <unordered_map>
#include <vector>

#include "../GameProxy.h"
#include "../Map.h"
#include "Node.h"

namespace marvin {
namespace path {

constexpr std::size_t kMaxNodes = 1024 * 1024;

struct NodeConnections {
  Node* neighbors[8];
  std::size_t count;
};

// Determines the node edges when using A*.
class NodeProcessor {
public:
  NodeProcessor(const Map& map) : map_(map) {
    nodes_.resize(kMaxNodes);
  }

  NodeConnections FindEdges(Node* node, Node* start, Node* goal, float radius);
  Node* GetNode(NodePoint point);

  // Calculate the node from the index.
  // This lets the node exist without storing its position so it fits in cache better.
  inline NodePoint GetPoint(const Node* node) const {
    size_t index = (node - &nodes_[0]);

    uint16_t world_y = (uint16_t)(index / 1024);
    uint16_t world_x = (uint16_t)(index % 1024);

    return NodePoint(world_x, world_y);
  }

private:
  std::vector<Node> nodes_;
  const Map& map_;
};

} // namespace path
} // namespace marvin
