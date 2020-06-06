#pragma once

#include <string>

#include "../BehaviorTree.h"

namespace marvin {

class GameProxy;

namespace behavior {

class FollowPathNode : public BehaviorNode {
 public:
  FollowPathNode() : path_name_("path") {}
  FollowPathNode(const std::string& path_name) : path_name_(path_name) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx);

 private:
  bool CanMoveBetween(GameProxy& game, Vector2f from, Vector2f to);

  std::string path_name_;
};

}  // namespace behavior
}  // namespace marvin
