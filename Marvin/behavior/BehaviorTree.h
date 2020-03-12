#pragma once

#include <cstdlib>
#include <vector>

#include "../Player.h"
#include "../Vector2f.h"

namespace marvin {

class Bot;

namespace behavior {

enum class ExecuteResult { Success, Failure, Running };

struct ExecuteContext {
  // TODO: Probably some blackboard-type structure
  // Just storing random variables that are used in here.

  Bot* bot;
  float dt;
  Vector2f target_position;
  const Player* target_player;
  std::vector<Vector2f> path;
};

class BehaviorNode {
 public:
  virtual ExecuteResult Execute(ExecuteContext& ctx) = 0;
};

class SequenceNode : public BehaviorNode {
 public:
  SequenceNode(std::vector<BehaviorNode*> children);
  template <typename... Args>
  SequenceNode(Args... children)
      : SequenceNode(
            std::vector<BehaviorNode*>({std::forward<Args>(children)...})) {}

  ExecuteResult Execute(ExecuteContext& ctx) override;

 private:
  std::vector<BehaviorNode*> children_;
  std::size_t running_node_index_;
};

class ParallelNode : public BehaviorNode {
public:
  ParallelNode(std::vector<BehaviorNode*> children);
  template <typename... Args>
  ParallelNode(Args... children)
    : ParallelNode(
      std::vector<BehaviorNode*>({ std::forward<Args>(children)... })) {}

  ExecuteResult Execute(ExecuteContext& ctx) override;

private:
  std::vector<BehaviorNode*> children_;
};

class SelectorNode : public BehaviorNode {
public:
  SelectorNode(std::vector<BehaviorNode*> children);
  template <typename... Args>
  SelectorNode(Args... children)
    : SelectorNode(
      std::vector<BehaviorNode*>({ std::forward<Args>(children)... })) {}

  ExecuteResult Execute(ExecuteContext& ctx) override;

private:
  std::vector<BehaviorNode*> children_;
};

}  // namespace behavior
}  // namespace marvin
