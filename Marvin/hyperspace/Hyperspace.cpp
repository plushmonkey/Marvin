#include "Hyperspace.h"

#include "../behavior/nodes/FollowPathNode.h"
#include "../platform/Platform.h"

namespace marvin {
namespace hs {

#if 1
const Vector2f kFlagRoom(820, 225);
const Vector2f kBaseEntrance(827, 280);
const Vector2f kSectorWarp(960, 63);
const Vector2f kTunnelWarp(570, 675);
#else
const Vector2f kFlagRoom(205, 205);
const Vector2f kBaseEntrance(180, 310);
const Vector2f kSectorWarp(60, 63);
const Vector2f kTunnelWarp(390, 395);
#endif

float GetPathDistance(std::vector<Vector2f> path) {
  if (path.size() <= 1)
    return 0.0f;

  float distance = 0.0f;

  for (std::size_t i = 0; i < path.size() - 1; ++i) {
    Vector2f current = path[i];
    Vector2f next = path[i + 1];

    distance += current.Distance(next);
  }

  return distance;
}

struct MoveToNode : public PathingNode {
  MoveToNode(Vector2f target) : target_(target), follow_node_("nav_path") {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto& game = ctx.bot->GetGame();
    Vector2f position = game.GetPosition();

    if (position.DistanceSq(target_) <= 2.0f * 2.0f) {
      return behavior::ExecuteResult::Success;
    }

    if (!ctx.bot->GetRegions().IsConnected(MapCoord(game.GetPosition()), MapCoord(target_))) {
      return behavior::ExecuteResult::Failure;
    }

    Path path = CreatePath(ctx, "nav_path", position, target_, game.GetShipSettings().GetRadius());
    ctx.blackboard.Set("nav_path", path);

    if (follow_node_.Execute(ctx) == behavior::ExecuteResult::Failure) {
      return behavior::ExecuteResult::Failure;
    }

    return behavior::ExecuteResult::Running;
  }

  Vector2f target_;
  behavior::FollowPathNode follow_node_;
};

struct BaseDefenseBehavior : public PathingNode {
public:
  BaseDefenseBehavior() : follow_node_("path") {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    const float kEnemyNearDistance = 40.0f;

    auto& game = ctx.bot->GetGame();
    auto target = ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    // Create the base path from entrance to flag room.
    if (base_path_.empty()) {
      ctx.blackboard.Erase("base_path");

      base_path_ = this->CreatePath(ctx, "base_path", kBaseEntrance, kFlagRoom, game.GetShipSettings().GetRadius());
      ctx.blackboard.Set("base_path", base_path_);

#if 0
      debug_log << "Creating base path through base" << std::endl;
      for (auto node : base_path_) {
        debug_log << "Node: " << node << std::endl;
      }
#endif
    }

    if (target != nullptr) {
      // Construct a path to the enemy to see how far away they are
      ctx.blackboard.Erase("target_path");
      Path target_path = this->CreatePath(ctx, "target_path", base_path_[current_index_], target->position,
                                          game.GetShipSettings(target->ship).GetRadius());
      float path_distance = GetPathDistance(target_path);

      if (path_distance < kEnemyNearDistance) {
        // Enemy is near bot, so move back to next node
        current_index_++;
      } else if (current_index_ > 0 && path_distance > kEnemyNearDistance * 1.2f) {
        // Enemy is far away from bot, so move up
        --current_index_;
      }

      if (current_index_ > base_path_.size()) {
        current_index_ = base_path_.size() - 1;
      }
    } else {
      current_index_ = 0;
    }

#if 0 // This is a test for gunning
    if (current_index_ > 0 && target) {
      float energy_percent = (game.GetPlayer().energy /
                              (float)game.GetShipSettings().InitialEnergy);

      Vector2f node_position = base_path_[current_index_ - 1];

      ctx.bot->GetSteering().Face(node_position);

      if (energy_percent > 0.7) {
        Vector2f to_target = target->position - game.GetPosition();
        // if (to_target.Dot(game.GetPlayer().GetHeading()) >= 0.7f) {
        if (node_position.Dot(game.GetPlayer().GetHeading()) >= 0.7f) {
          ctx.bot->GetKeys().Press(VK_CONTROL);
        }
      }
    }
#endif

    // Construct a micro-level path to the current node from the larger base
    // nodes.
    Path micro_path = this->CreatePath(ctx, "path", game.GetPosition(), base_path_[current_index_],
                                       game.GetShipSettings().GetRadius());

    ctx.blackboard.Set("path", micro_path);

    if (follow_node_.Execute(ctx) == behavior::ExecuteResult::Failure) {
      return behavior::ExecuteResult::Failure;
    }

    return behavior::ExecuteResult::Success;
  }

private:
  std::size_t current_index_ = 0;
  Path base_path_;
  behavior::FollowPathNode follow_node_;
};

struct HyperspaceFlagger : public behavior::SelectorNode {
  HyperspaceFlagger(Bot& bot) {
    auto move_fr_node = std::make_unique<MoveToNode>(kFlagRoom);
    auto warp_to_sector_node = std::make_unique<MoveToNode>(kSectorWarp);
    auto warp_to_tunnel_node = std::make_unique<MoveToNode>(kTunnelWarp);

    auto in_center_node = std::make_unique<InRegionNode>(Vector2f(512, 512));
    auto in_tunnel_node = std::make_unique<InRegionNode>(Vector2f(30, 30));
    auto in_sector_node = std::make_unique<InRegionNode>(kFlagRoom);

    auto move_to_tunnel = std::make_unique<behavior::SequenceNode>(in_center_node.get(), warp_to_tunnel_node.get());
    auto move_to_sector = std::make_unique<behavior::SequenceNode>(in_tunnel_node.get(), warp_to_sector_node.get());

    auto find_enemy = std::make_unique<FindEnemyNode>();
    auto success_find_enemy = std::make_unique<behavior::SuccessNode>(find_enemy.get());

    auto defense = std::make_unique<BaseDefenseBehavior>();

    auto defend_sequence = std::make_unique<behavior::SequenceNode>(success_find_enemy.get(), defense.get());

    auto selector =
        std::make_unique<behavior::SelectorNode>(move_to_tunnel.get(), move_to_sector.get(), defend_sequence.get());

    bot.SetBehavior(selector.get());

    bot.AddBehaviorNode(std::move(move_fr_node));
    bot.AddBehaviorNode(std::move(warp_to_sector_node));
    bot.AddBehaviorNode(std::move(warp_to_tunnel_node));

    bot.AddBehaviorNode(std::move(in_center_node));
    bot.AddBehaviorNode(std::move(in_tunnel_node));
    bot.AddBehaviorNode(std::move(in_sector_node));

    bot.AddBehaviorNode(std::move(move_to_tunnel));
    bot.AddBehaviorNode(std::move(move_to_sector));

    bot.AddBehaviorNode(std::move(find_enemy));
    bot.AddBehaviorNode(std::move(success_find_enemy));
    bot.AddBehaviorNode(std::move(defense));
    bot.AddBehaviorNode(std::move(defend_sequence));

    bot.AddBehaviorNode(std::move(selector));
  }
};

bool SetHyperspaceBehavior(Bot& bot) {
  auto hs = std::make_unique<HyperspaceFlagger>(bot);

  bot.AddBehaviorNode(std::move(hs));

  return true;
}

} // namespace hs
} // namespace marvin
