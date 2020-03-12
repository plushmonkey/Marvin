#pragma once

#include "BehaviorTree.h"

namespace marvin {
namespace behavior {

class BehaviorEngine {
public:
  BehaviorEngine(BehaviorNode* behavior_tree);

  void Update(ExecuteContext& ctx);

private:
  BehaviorNode* behavior_tree_;
};

}  // namespace behavior
}  // namespace marvin
