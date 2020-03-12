#include "BehaviorTree.h"

namespace marvin {
namespace behavior {

SequenceNode::SequenceNode(std::vector<BehaviorNode*> children)
    : children_(std::move(children)), running_node_index_(-1) {}

ExecuteResult SequenceNode::Execute(ExecuteContext& ctx) {
  std::size_t index = 0;

  if (running_node_index_ < children_.size()) {
    index = running_node_index_;
  }

  for (; index < children_.size(); ++index) {
    auto node = children_[index];
    ExecuteResult result = node->Execute(ctx);

    if (result == ExecuteResult::Failure) {
      return result;
    } else if (result == ExecuteResult::Running) {
      this->running_node_index_ = index;
      return result;
    }
  }

  return ExecuteResult::Success;
}

ParallelNode::ParallelNode(std::vector<BehaviorNode*> children)
    : children_(std::move(children)) {}

ExecuteResult ParallelNode::Execute(ExecuteContext& ctx) {
  ExecuteResult result = ExecuteResult::Success;

  for (auto& child : children_) {
    ExecuteResult child_result = child->Execute(ctx);

    if (result == ExecuteResult::Success &&
        child_result != ExecuteResult::Success) {
      result = child_result;
    }
  }

  return result;
}

SelectorNode::SelectorNode(std::vector<BehaviorNode*> children)
    : children_(std::move(children)) {}

ExecuteResult SelectorNode::Execute(ExecuteContext& ctx) {
  ExecuteResult result = ExecuteResult::Failure;

  for (auto& child : children_) {
    ExecuteResult child_result = child->Execute(ctx);

    if (child_result == ExecuteResult::Running ||
        child_result == ExecuteResult::Success) {
      return child_result;
    }
  }

  return result;
}

}  // namespace behavior
}  // namespace marvin
