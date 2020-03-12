#include "BehaviorEngine.h"
#include "../Bot.h"

namespace marvin {
namespace behavior {

BehaviorEngine::BehaviorEngine(BehaviorNode* behavior_tree)
    : behavior_tree_(behavior_tree) {}

void BehaviorEngine::Update(ExecuteContext& ctx) {
  behavior_tree_->Execute(ctx);
}

}  // namespace behavior
}  // namespace marvin
