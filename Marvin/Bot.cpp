#include "Bot.h"

#include <cstring>
#include <limits>

#include "GameProxy.h"
#include "Map.h"
#include "RayCaster.h"
#include "platform/Platform.h"

namespace {

bool IsValidPosition(marvin::Vector2f position) {
  return position.x >= 0 && position.x < 1024 && position.y >= 0 &&
         position.y < 1024;
}

bool CanShoot(const marvin::Map& map, const marvin::Player& bot_player,
              const marvin::Player& target) {
  if (bot_player.position.DistanceSq(target.position) > 25 * 25) return false;
  if (map.GetTileId(bot_player.position) == marvin::kSafeTileId) return false;

  return true;
}

marvin::Vector2f CalculateShot(const marvin::Vector2f& pShooter,
                               const marvin::Vector2f& pTarget,
                               const marvin::Vector2f& vShooter,
                               const marvin::Vector2f& vTarget,
                               float sProjectile) {
  marvin::Vector2f totarget = pTarget - pShooter;
  marvin::Vector2f v = vTarget - vShooter;

  float a = v.Dot(v) - sProjectile * sProjectile;
  float b = 2 * v.Dot(totarget);
  float c = totarget.Dot(totarget);

  marvin::Vector2f solution;

  float disc = (b * b) - 4 * a * c;
  float t = -1.0;

  if (disc >= 0.0) {
    float t1 = (-b + std::sqrt(disc)) / (2 * a);
    float t2 = (-b - std::sqrt(disc)) / (2 * a);
    if (t1 < t2 && t1 >= 0)
      t = t1;
    else
      t = t2;
  }

  // Only use the calculated shot if its collision is within acceptable
  // timeframe
  if (t < 0 || t > 5) {
    marvin::Vector2f hShooter = marvin::Normalize(vShooter);
    marvin::Vector2f hTarget = marvin::Normalize(vTarget);

    int sign = hShooter.Dot(hTarget) < 0.0 ? -1 : 1;

    float speed = vShooter.Length() + (sign * vTarget.Length()) + sProjectile;
    float look_ahead = totarget.Length() / speed;
    return marvin::Vector2f(pTarget + vTarget * look_ahead);
  }

  solution = pTarget + (v * t);

  return solution;
}

}  // namespace

namespace marvin {

class PatrolNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    // TODO: Implement

    return behavior::ExecuteResult::Success;
  }
};

class TargetInLineOfSightNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    if (ctx.target_player == nullptr) return behavior::ExecuteResult::Failure;

    auto& game = ctx.bot->GetGame();
    const auto& target = *ctx.target_player;

    auto to_target = target.position - game.GetPosition();

    CastResult result = RayCast(game.GetMap(), game.GetPosition(),
                                Normalize(to_target), to_target.Length());

    return result.hit ? behavior::ExecuteResult::Failure
                      : behavior::ExecuteResult::Success;
  }
};

class FindEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    behavior::ExecuteResult result = behavior::ExecuteResult::Failure;
    float closest_distance = std::numeric_limits<float>::max();
    auto& game = ctx.bot->GetGame();
    const Player* target = nullptr;

    for (std::size_t i = 0; i < game.GetPlayers().size(); ++i) {
      const marvin::Player& player = game.GetPlayers()[i];

      if (player.id == game.GetPlayer().id) continue;
      if (player.ship > 7) continue;
      if (player.frequency == game.GetPlayer().frequency) continue;
      if (game.GetMap().GetTileId(player.position) == marvin::kSafeTileId)
        continue;

      if (!IsValidPosition(player.position)) continue;

      float dist_sq = game.GetPlayer().position.DistanceSq(player.position);

      if (dist_sq < closest_distance) {
        closest_distance = dist_sq;
        target = &game.GetPlayers()[i];
        result = behavior::ExecuteResult::Success;
      }
    }

    ctx.target_player = target;

    return result;
  }
};

class LookingAtEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    if (ctx.target_player == nullptr) return behavior::ExecuteResult::Failure;
    const Player& target = *ctx.target_player;
    auto& game = ctx.bot->GetGame();
    const Player& bot_player = game.GetPlayer();

    float proj_speed =
        game.GetSettings().ShipSettings[bot_player.ship].BulletSpeed / 10.0f /
        16.0f;

    Vector2f seek_position =
        CalculateShot(game.GetPosition(), target.position, bot_player.velocity,
                      target.velocity, proj_speed);

    Vector2f direction = Normalize(seek_position - bot_player.position);
    float dot = bot_player.GetHeading().Dot(direction);

    if (dot >= 0.95) {
      if (CanShoot(game.GetMap(), bot_player, target)) {
        ctx.target_position = seek_position;
        return behavior::ExecuteResult::Success;
      }
    }

    return behavior::ExecuteResult::Failure;
  }
};

class ShootEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    ctx.bot->GetKeys().Press(VK_CONTROL);

    return behavior::ExecuteResult::Success;
  }
};

class MoveToEnemyNode : public behavior::BehaviorNode {
public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    ctx.bot->Move(ctx.target_position, 15.0f);

    return behavior::ExecuteResult::Success;
  }
};

class PathToEnemyNode : public behavior::BehaviorNode {
public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    bool rebuild_path = true;
    auto& game = ctx.bot->GetGame();

    auto from = game.GetPosition();
    auto to = ctx.target_player->position;

    const auto& bot_path = ctx.bot->GetPath();

    if (!bot_path.empty()) {
      if (bot_path.back().DistanceSq(to) < 3 * 3) {
        rebuild_path = false;
      }
    }

    if (rebuild_path) {
      ctx.path = ctx.bot->GetPathfinder().FindPath(from, to);

      int ship = game.GetPlayer().ship;
      float ship_radius = game.GetSettings().ShipSettings[ship].Radius;

      ctx.path = ctx.bot->GetPathfinder().SmoothPath(ctx.path, game.GetMap(), ship_radius);
    }

    return behavior::ExecuteResult::Success;
  }
};

class FollowPathNode : public behavior::BehaviorNode {
public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    if (ctx.path.empty()) return behavior::ExecuteResult::Failure;
    
    auto& path = ctx.path;

    auto position = ctx.bot->GetGame().GetPosition();

    Vector2f current = path.front();

    while (current.DistanceSq(position) < 2 && !path.empty()) {
      path.erase(path.begin());
      current = path.front();
    }

    ctx.bot->Move(current, 0.0f);

    return behavior::ExecuteResult::Success;
  }
};

Bot::Bot(std::unique_ptr<GameProxy> game) : game_(std::move(game)) {
  auto processor = std::make_unique<path::NodeProcessor>(game_->GetMap());

  pathfinder_ = std::make_unique<path::Pathfinder>(std::move(processor));

  auto find_enemy = std::make_unique<FindEnemyNode>();
  auto looking_at_enemy = std::make_unique<LookingAtEnemyNode>();
  auto target_in_los = std::make_unique<TargetInLineOfSightNode>();
  auto shoot_enemy = std::make_unique<ShootEnemyNode>();
  auto path_to_enemy = std::make_unique<PathToEnemyNode>();
  auto move_to_enemy = std::make_unique<MoveToEnemyNode>();
  auto follow_path = std::make_unique<FollowPathNode>();

  auto shoot_sequence = std::make_unique<behavior::SequenceNode>(looking_at_enemy.get(), shoot_enemy.get());
  auto parallel_shoot_enemy = std::make_unique<behavior::SequenceNode>(shoot_sequence.get(), move_to_enemy.get());
  auto los_shoot_conditional = std::make_unique<behavior::SequenceNode>(target_in_los.get(), parallel_shoot_enemy.get());

  auto path_sequence = std::make_unique<behavior::SequenceNode>(path_to_enemy.get(), follow_path.get());

  auto path_or_shoot_selector = std::make_unique<behavior::SelectorNode>(los_shoot_conditional.get(), path_sequence.get());

  auto handle_enemy = std::make_unique<behavior::SequenceNode>(find_enemy.get(), path_or_shoot_selector.get());

  auto root_selector = std::make_unique<behavior::SelectorNode>(
    handle_enemy.get()
    // Patrol
    );

  behavior_nodes_.push_back(std::move(find_enemy));
  behavior_nodes_.push_back(std::move(looking_at_enemy));
  behavior_nodes_.push_back(std::move(target_in_los));
  behavior_nodes_.push_back(std::move(shoot_enemy));
  behavior_nodes_.push_back(std::move(path_to_enemy));
  behavior_nodes_.push_back(std::move(move_to_enemy));
  behavior_nodes_.push_back(std::move(follow_path ));

  behavior_nodes_.push_back(std::move(shoot_sequence));
  behavior_nodes_.push_back(std::move(parallel_shoot_enemy));
  behavior_nodes_.push_back(std::move(los_shoot_conditional));
  behavior_nodes_.push_back(std::move(path_sequence));
  behavior_nodes_.push_back(std::move(path_or_shoot_selector));
  behavior_nodes_.push_back(std::move(handle_enemy));
  behavior_nodes_.push_back(std::move(root_selector));

  behavior_ = std::make_unique<behavior::BehaviorEngine>(behavior_nodes_.back().get());
}

void Bot::Update(float dt) {
  keys_.ReleaseAll();
  game_->Update(dt);

  behavior::ExecuteContext ctx;

  ctx.bot = this;
  ctx.dt = dt;
  ctx.path = path_;

  behavior_->Update(ctx);

  path_ = ctx.path;
}

void Bot::Move(const Vector2f& target, float target_distance) {
  const Player& bot_player = game_->GetPlayer();
  Vector2f seek = target - game_->GetPosition();
  Vector2f direction = marvin::Normalize(seek);
  Vector2f heading = bot_player.GetHeading();

  // Simple movement controller
  auto perp = marvin::Perpendicular(heading);
  float dot = heading.Dot(direction);
  bool clockwise = perp.Dot(direction) >= 0.0;

  float threshold = 0.15f;

  float distance = bot_player.position.Distance(target);

  if (distance < target_distance) {
    keys_.Press(VK_DOWN);
  } else {
    if (dot < -threshold) {
      keys_.Press(VK_DOWN);
    } else if (dot > threshold) {
      keys_.Press(VK_UP);
    }
  }

  if (dot < 0.985f) {
    keys_.Set(VK_RIGHT, clockwise);
    keys_.Set(VK_LEFT, !clockwise);
  }
}

}  // namespace marvin
