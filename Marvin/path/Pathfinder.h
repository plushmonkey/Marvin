#pragma once

#include <algorithm>
#include <memory>
#include <vector>

#include "../Vector2f.h"
#include "NodeProcessor.h"
#include "Path.h"

namespace marvin {
namespace path {

template <typename T, typename Compare, typename Container = std::vector<T>>
class PriorityQueue {
 public:
  using const_iterator = typename Container::const_iterator;

  const_iterator begin() const { return container_.cbegin(); }
  const_iterator end() const { return container_.cend(); }

  void Push(T item) {
    container_.push_back(item);
    std::push_heap(container_.begin(), container_.end(), comparator_);
  }

  T Pop() {
    T item = container_.front();
    std::pop_heap(container_.begin(), container_.end(), comparator_);
    container_.pop_back();
    return item;
  }

  void Update() {
    std::make_heap(container_.begin(), container_.end(), comparator_);
  }

  void Clear() { container_.clear(); }
  std::size_t Size() const { return container_.size(); }
  bool Empty() const { return container_.empty(); }

 private:
  Container container_;
  Compare comparator_;
};

struct Pathfinder {
 public:
  Pathfinder(std::unique_ptr<NodeProcessor> processor);
  std::vector<Vector2f> FindPath(const Vector2f& from, const Vector2f& to, float radius);

  std::vector<Vector2f> SmoothPath(const std::vector<Vector2f>& path,
                                   const Map& map, float ship_radius);

  void CreateMapWeights(const Map& map);

 private:
  struct NodeCompare {
    bool operator()(const Node* lhs, const Node* rhs) const {
      if (lhs->f == rhs->f) return lhs->rotations > rhs->rotations;
      return lhs->f > rhs->f;
    }
  };

  std::unique_ptr<NodeProcessor> processor_;
  PriorityQueue<Node*, NodeCompare> openset_;
};

}  // namespace path
}  // namespace marvin
