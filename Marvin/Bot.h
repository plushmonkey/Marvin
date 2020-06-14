#pragma once

#include <fstream>
#include <memory>

#include "KeyController.h"
#include "RayCaster.h"
#include "RegionRegistry.h"
#include "Steering.h"
#include "behavior/BehaviorEngine.h"
#include "path/Pathfinder.h"

namespace marvin {

extern std::ofstream debug_log;

class GameProxy;
struct Player;

class Bot {
 public:
  Bot(std::unique_ptr<GameProxy> game);

  void Update(float dt);

  KeyController& GetKeys() { return keys_; }
  GameProxy& GetGame() { return *game_; }

  void Move(const Vector2f& target, float target_distance);
  path::Pathfinder& GetPathfinder() { return *pathfinder_; }

  const RegionRegistry& GetRegions() const { return *regions_; }

  SteeringBehavior& GetSteering() { return steering_; }

  uint64_t GetTime() const;

  void AddBehaviorNode(std::unique_ptr<behavior::BehaviorNode> node) {
    behavior_nodes_.push_back(std::move(node));
  }

  void SetBehavior(behavior::BehaviorNode* behavior) {
    behavior_ = std::make_unique<behavior::BehaviorEngine>(behavior);
  }

 private:
  void Steer();

  std::unique_ptr<GameProxy> game_;
  std::unique_ptr<path::Pathfinder> pathfinder_;
  std::unique_ptr<behavior::BehaviorEngine> behavior_;
  std::unique_ptr<RegionRegistry> regions_;
  behavior::ExecuteContext behavior_ctx_;
  int ship_;
  uint64_t last_ship_change_;
  SteeringBehavior steering_;

  std::vector<std::unique_ptr<behavior::BehaviorNode>> behavior_nodes_;

  // TODO: Action-key map would be more versatile
  KeyController keys_;
};

bool InRect(marvin::Vector2f pos, marvin::Vector2f min_rect,
            marvin::Vector2f max_rect);

bool IsValidPosition(marvin::Vector2f position);

bool CanShoot(const marvin::Map& map, const marvin::Player& bot_player,
              const marvin::Player& target);

marvin::Vector2f CalculateShot(const marvin::Vector2f& pShooter,
                               const marvin::Vector2f& pTarget,
                               const marvin::Vector2f& vShooter,
                               const marvin::Vector2f& vTarget,
                               float sProjectile);

class PathingNode : public behavior::BehaviorNode {
 public:
  using Path = std::vector<Vector2f>;

  virtual behavior::ExecuteResult Execute(
      behavior::ExecuteContext& ctx) override = 0;

 protected:
  Path CreatePath(behavior::ExecuteContext& ctx, const std::string& pathname,
                  Vector2f from, Vector2f to, float radius) {
    bool build = true;
    auto& game = ctx.bot->GetGame();

    Path path = ctx.blackboard.ValueOr(pathname, Path());

    if (!path.empty()) {
      // Check if the current destination is the same as the requested one.
      if (path.back().DistanceSq(to) < 3 * 3) {
        Vector2f pos = game.GetPosition();
        Vector2f next = path.front();
        Vector2f direction = Normalize(next - pos);
        Vector2f side = Perpendicular(direction);
        float radius = game.GetShipSettings().GetRadius();

        float distance = next.Distance(pos);

        // Rebuild the path if the bot isn't in line of sight of its next node.
        CastResult center = RayCast(game.GetMap(), pos, direction, distance);
        CastResult side1 =
            RayCast(game.GetMap(), pos + side * radius, direction, distance);
        CastResult side2 =
            RayCast(game.GetMap(), pos - side * radius, direction, distance);

        if (!center.hit && !side1.hit && !side2.hit) {
          build = false;
        }
      }
    }

    if (build) {
      path = ctx.bot->GetPathfinder().FindPath(from, to, radius);
      path = ctx.bot->GetPathfinder().SmoothPath(path, game.GetMap(), radius);
    }

    return path;
  }
};

class FindEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    behavior::ExecuteResult result = behavior::ExecuteResult::Failure;
    float closest_cost = std::numeric_limits<float>::max();
    auto& game = ctx.bot->GetGame();
    const Player* target = nullptr;
    const Player& bot_player = ctx.bot->GetGame().GetPlayer();

    Vector2f resolution(1920, 1080);
    view_min_ = bot_player.position - resolution / 2.0f / 16.0f;
    view_max_ = bot_player.position + resolution / 2.0f / 16.0f;

    for (std::size_t i = 0; i < game.GetPlayers().size(); ++i) {
      const marvin::Player& player = game.GetPlayers()[i];

      if (!IsValidTarget(ctx, player)) continue;

      float cost = CalculateCost(game, bot_player, player);

      if (cost < closest_cost) {
        closest_cost = cost;
        target = &game.GetPlayers()[i];
        result = behavior::ExecuteResult::Success;
      }
    }

    const Player* current_target =
        ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    if (current_target && IsValidTarget(ctx, *current_target)) {
      // Calculate the cost to the current target so there's some stickiness
      // between close targets.
      const float kStickiness = 2.0f;
      float cost = CalculateCost(game, bot_player, *current_target);

      if (cost * kStickiness < closest_cost) {
        target = current_target;
      }
    }

    if (current_target != target) {
      ctx.blackboard.Set("aggression", 0.0f);
    }

    ctx.blackboard.Set("target_player", target);
    const Player* r =
        ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    return result;
  }

 private:
  float CalculateCost(GameProxy& game, const Player& bot_player,
                      const Player& target) {
    float dist = bot_player.position.Distance(target.position);
    // How many seconds it takes to rotate 180 degrees
    float rotate_speed = game.GetShipSettings().MaximumRotation / 200.0f;
    float move_cost =
        dist / (game.GetShipSettings().MaximumSpeed / 10.0f / 16.0f);
    Vector2f direction = Normalize(target.position - bot_player.position);
    float dot = std::abs(bot_player.GetHeading().Dot(direction) - 1.0f) / 2.0f;
    float rotate_cost = std::abs(dot) * rotate_speed;

    return move_cost + rotate_cost;
  }

  bool IsValidTarget(behavior::ExecuteContext& ctx, const Player& target) {
    const auto& game = ctx.bot->GetGame();
    const Player& bot_player = game.GetPlayer();

    if (target.id == game.GetPlayer().id) return false;
    if (target.ship > 7) return false;
    if (target.frequency == game.GetPlayer().frequency) return false;
    if (target.name[0] == '<') return false;

    if (game.GetMap().GetTileId(target.position) == marvin::kSafeTileId) {
      return false;
    }

    if (!IsValidPosition(target.position)) {
      return false;
    }

    MapCoord bot_coord(bot_player.position);
    MapCoord target_coord(target.position);

    if (!ctx.bot->GetRegions().IsConnected(bot_coord, target_coord)) {
      return false;
    }

    bool stealthing = (target.status & 1) != 0;
    bool cloaking = (target.status & 2) != 0;

    if (!(game.GetPlayer().status & 4)) {
      if (stealthing && cloaking) return false;

      bool visible = InRect(target.position, view_min_, view_max_);

      if (stealthing && !visible) return false;
    }

    return true;
  }

  Vector2f view_min_;
  Vector2f view_max_;
};

struct InRegionNode : public behavior::BehaviorNode {
  InRegionNode(Vector2f position) : target_(position) {}
  InRegionNode(MapCoord position) : target_(position) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    marvin::MapCoord coord(ctx.bot->GetGame().GetPosition());

    bool connected = ctx.bot->GetRegions().IsConnected(coord, target_);

    return connected ? behavior::ExecuteResult::Success
      : behavior::ExecuteResult::Failure;
  }

  marvin::MapCoord target_;
};

struct OnFrequencyNode : public behavior::BehaviorNode {
  OnFrequencyNode(int frequency) : frequency(frequency) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    if (ctx.bot->GetGame().GetPlayer().frequency == frequency) {
      return behavior::ExecuteResult::Success;
    }

    return behavior::ExecuteResult::Failure;
  }

  int frequency;
};


}  // namespace marvin
