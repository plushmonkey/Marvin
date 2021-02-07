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

  void ResetNodes();

  NodeConnections FindEdges(Node* node, Node* start, Node* goal, float radius);
  Node* GetNode(NodePoint point);

private:
  std::vector<Node> nodes_;
  const Map& map_;
};

} // namespace path
} // namespace marvin
