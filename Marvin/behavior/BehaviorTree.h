#pragma once

#include <cstdlib>
#include <vector>

#include "../Player.h"
#include "../Vector2f.h"
#include "Blackboard.h"

namespace marvin {

class Bot;

namespace behavior {

enum class ExecuteResult { Success, Failure, Running };

struct ExecuteContext {
  Blackboard blackboard;
  Bot* bot;
  float dt;

  ExecuteContext() : bot(nullptr), dt(0) {}
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
            std::vector<BehaviorNode*>({std::forward<Args>(children)...})) {}

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
            std::vector<BehaviorNode*>({std::forward<Args>(children)...})) {}

  ExecuteResult Execute(ExecuteContext& ctx) override;

 protected:
  std::vector<BehaviorNode*> children_;
};

class SuccessNode : public BehaviorNode {
public:
  SuccessNode(BehaviorNode* child);

  ExecuteResult Execute(ExecuteContext& ctx) override;
protected:
  BehaviorNode* child_;
};

class InvertNode : public BehaviorNode {
public:
  InvertNode(BehaviorNode* child);

  ExecuteResult Execute(ExecuteContext& ctx) override;
protected:
  BehaviorNode* child_;
};

}  // namespace behavior
}  // namespace marvin
