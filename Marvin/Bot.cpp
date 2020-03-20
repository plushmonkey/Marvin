#include "Bot.h"

#include <chrono>
#include <cstring>
#include <limits>

#include "GameProxy.h"
#include "Map.h"
#include "RayCaster.h"
#include "RegionRegistry.h"
#include "platform/Platform.h"

namespace {

bool InRect(marvin::Vector2f pos, marvin::Vector2f min_rect,
            marvin::Vector2f max_rect) {
  return ((pos.x >= min_rect.x && pos.y >= min_rect.y) &&
          (pos.x <= max_rect.x && pos.y <= max_rect.y));
}

bool IsValidPosition(marvin::Vector2f position) {
  return position.x >= 0 && position.x < 1024 && position.y >= 0 &&
         position.y < 1024;
}

bool CanShoot(const marvin::Map& map, const marvin::Player& bot_player,
              const marvin::Player& target) {
  if (bot_player.position.DistanceSq(target.position) > 60 * 60) return false;
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

  solution = pTarget + (v * t);

  return solution;
}

}  // namespace

namespace marvin {

class PathingNode : public behavior::BehaviorNode {
 public:
  using Path = std::vector<Vector2f>;

  virtual behavior::ExecuteResult Execute(
      behavior::ExecuteContext& ctx) override = 0;

 protected:
  Path CreatePath(behavior::ExecuteContext& ctx, const std::string& pathname,
                  Vector2f from, Vector2f to) {
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
      path = ctx.bot->GetPathfinder().FindPath(from, to);

      int ship = game.GetPlayer().ship;
      float ship_radius = game.GetSettings().ShipSettings[ship].GetRadius();

      path =
          ctx.bot->GetPathfinder().SmoothPath(path, game.GetMap(), ship_radius);
    }

    return path;
  }
};

class PatrolNode : public PathingNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    auto& game = ctx.bot->GetGame();

    Vector2f from = game.GetPosition();

    auto nodes =
        ctx.blackboard.ValueOr("patrol_nodes", std::vector<Vector2f>());
    auto index = ctx.blackboard.ValueOr("patrol_index", 0);

    if (nodes.empty()) {
      return behavior::ExecuteResult::Failure;
    }

    Vector2f to = nodes.at(index);

    if (game.GetPosition().DistanceSq(to) < 3.0f * 3.0f) {
      index = (index + 1) % nodes.size();
      ctx.blackboard.Set("patrol_index", index);
      to = nodes.at(index);
    }

    Path path = ctx.blackboard.ValueOr("path", Path());

    path = CreatePath(ctx, "path", from, to);

    ctx.blackboard.Set("path", path);

    return behavior::ExecuteResult::Success;
  }
};

class InLineOfSightNode : public behavior::BehaviorNode {
 public:
  using VectorSelector =
      std::function<const Vector2f*(marvin::behavior::ExecuteContext&)>;
  InLineOfSightNode(VectorSelector selector) : selector_(selector) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    auto target = selector_(ctx);

    if (target == nullptr) return behavior::ExecuteResult::Failure;

    auto& game = ctx.bot->GetGame();

    auto to_target = *target - game.GetPosition();

    CastResult result = RayCast(game.GetMap(), game.GetPosition(),
                                Normalize(to_target), to_target.Length());

    return result.hit ? behavior::ExecuteResult::Failure
                      : behavior::ExecuteResult::Success;
  }

 private:
  VectorSelector selector_;
};

class FindEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    const float kRotationMultiplier = 2.0f;

    behavior::ExecuteResult result = behavior::ExecuteResult::Failure;
    float closest_distance = std::numeric_limits<float>::max();
    auto& game = ctx.bot->GetGame();
    const Player* target = nullptr;
    const Player& bot_player = ctx.bot->GetGame().GetPlayer();

    Vector2f resolution(1920, 1080);
    view_min_ = bot_player.position - resolution / 2.0f / 16.0f;
    view_max_ = bot_player.position + resolution / 2.0f / 16.0f;

    for (std::size_t i = 0; i < game.GetPlayers().size(); ++i) {
      const marvin::Player& player = game.GetPlayers()[i];

      if (!IsValidTarget(ctx, player)) continue;

      Vector2f direction = Normalize(player.position - bot_player.position);
      float dot = bot_player.GetHeading().Dot(direction);
      float dist = game.GetPlayer().position.Distance(player.position);
      float multiplier = 1.0f + ((1.0f - dot) / kRotationMultiplier);
      float calc = dist * multiplier;

      if (calc < closest_distance) {
        closest_distance = calc;
        target = &game.GetPlayers()[i];
        result = behavior::ExecuteResult::Success;
      }
    }

    const Player* current_target =
        ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    if (current_target != target) {
      ctx.blackboard.Set("aggression", 0.0f);
    }

    ctx.blackboard.Set("target_player", target);
    const Player* r =
        ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    return result;
  }

 private:
  bool IsValidTarget(behavior::ExecuteContext& ctx, const Player& target) {
    const auto& game = ctx.bot->GetGame();
    const Player& bot_player = game.GetPlayer();

    if (target.id == game.GetPlayer().id) return false;
    if (target.ship > 7) return false;
    if (target.frequency == game.GetPlayer().frequency) return false;

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

    bool stealthing = (target.status & 2) != 0;
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

class LookingAtEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    const auto target_player =
        ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    if (target_player == nullptr) return behavior::ExecuteResult::Failure;

    const Player& target = *target_player;
    auto& game = ctx.bot->GetGame();
    const Player& bot_player = game.GetPlayer();

    float proj_speed =
        game.GetSettings().ShipSettings[bot_player.ship].BulletSpeed / 10.0f /
        16.0f;

    Vector2f target_pos = target.position;

    Vector2f seek_position =
        CalculateShot(game.GetPosition(), target_pos, bot_player.velocity,
                      target.velocity, proj_speed);

    Vector2f projectile_trajectory =
        (bot_player.GetHeading() * proj_speed) + bot_player.velocity;

    Vector2f projectile_direction = Normalize(projectile_trajectory);
    float target_radius =
        game.GetSettings().ShipSettings[target.ship].GetRadius();

    float aggression = ctx.blackboard.ValueOr("aggression", 0.0f);
    float radius_multiplier = 1.0f + aggression * 1.0f;
    float nearby_radius = target_radius * radius_multiplier;

    Vector2f box_min = target.position - Vector2f(nearby_radius, nearby_radius);
    Vector2f box_extent(nearby_radius * 2, nearby_radius * 2);
    float dist;
    Vector2f norm;

    bool hit = RayBoxIntersect(bot_player.position, projectile_direction,
                               box_min, box_extent, &dist, &norm);

    if (!hit) {
      box_min = seek_position - Vector2f(nearby_radius, nearby_radius);
      hit = RayBoxIntersect(bot_player.position, bot_player.GetHeading(),
                            box_min, box_extent, &dist, &norm);
    }

    ctx.blackboard.Set("target_position", seek_position);

    if (hit) {
      if (CanShoot(game.GetMap(), bot_player, target)) {
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
    float hover_distance = 30.0f;
    float aggression_min = 15.0f;

    float aggression = ctx.blackboard.ValueOr("aggression", 0.0f);
    aggression += ctx.dt / 15.0f;

    ctx.blackboard.Set("aggression", aggression);
    Vector2f target_position =
        ctx.blackboard.ValueOr("target_position", Vector2f());

    float target_dist =
        hover_distance - (aggression * (hover_distance - aggression_min));

    ctx.bot->Move(target_position, target_dist);

    auto& game = ctx.bot->GetGame();

    float proj_speed = game.GetShipSettings().BulletSpeed / 10.0f / 16.0f;
    float radius = game.GetShipSettings().GetRadius() * 1.5f;
    float max_speed = game.GetShipSettings().MaximumSpeed / 10.0f / 16.0f;

    Vector2f box_pos = game.GetPosition() - Vector2f(radius, radius);

    const Player& player =
        *ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);
    Vector2f shoot_direction =
        Normalize(player.velocity + (player.GetHeading() * proj_speed));
    // Vector2f shoot_direction = player.GetHeading();

    float dist;
    if (LineBoxIntersect(player.position, shoot_direction, box_pos,
                         Vector2f(radius * 2, radius * 2), &dist, nullptr)) {
      Vector2f shoot_perp = Perpendicular(shoot_direction);
      Vector2f box_hit = player.position + shoot_direction * dist;
#if 0
        // Move in the opposite direction of the hit on the hitbox.
        Vector2f box_dodge_dir = Normalize(player.position - box_hit);
        Vector2f dodge_dir = Normalize(shoot_perp * box_dodge_dir.Dot(shoot_perp));
#else
      Vector2f dodge_dir = Perpendicular(shoot_direction);
#endif
      // debug_log << "Dodging" << std::endl;

      ctx.bot->GetSteering() += (dodge_dir * max_speed * 100.0f);
      // ctx.bot->Move(game.GetPosition() + dodge_dir, 0.0f);
    }

    return behavior::ExecuteResult::Success;
  }
};

class PathToEnemyNode : public PathingNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    auto& game = ctx.bot->GetGame();

    auto from = game.GetPosition();
    auto to = ctx.blackboard.ValueOr<const Player*>("target_player", nullptr)
                  ->position;

    Path path = CreatePath(ctx, "path", from, to);

    ctx.blackboard.Set("path", path);

    return behavior::ExecuteResult::Success;
  }
};

class FollowPathNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    auto path = ctx.blackboard.ValueOr("path", std::vector<Vector2f>());
    size_t path_size = path.size();

    if (path.empty()) return behavior::ExecuteResult::Failure;

    auto& game = ctx.bot->GetGame();
    Vector2f current = path.front();

    while (path.size() > 1 &&
           CanMoveBetween(game, game.GetPosition(), path.at(1))) {
      path.erase(path.begin());
      current = path.front();
    }

    if (path.size() == 1 &&
        path.front().DistanceSq(game.GetPosition()) < 2 * 2) {
      path.clear();
    }

    if (path.size() != path_size) {
      ctx.blackboard.Set("path", path);
    }

    ctx.bot->Move(current, 0.0f);

    return behavior::ExecuteResult::Success;
  }

 private:
  bool CanMoveBetween(GameProxy& game, Vector2f from, Vector2f to) {
    Vector2f trajectory = to - from;
    Vector2f direction = Normalize(trajectory);
    Vector2f side = Perpendicular(direction);

    float distance = from.Distance(to);
    float radius = game.GetShipSettings().GetRadius();

    CastResult center = RayCast(game.GetMap(), from, direction, distance);
    CastResult side1 =
        RayCast(game.GetMap(), from + side * radius, direction, distance);
    CastResult side2 =
        RayCast(game.GetMap(), from - side * radius, direction, distance);

    return !center.hit && !side1.hit && !side2.hit;
  }
};

Bot::Bot(std::unique_ptr<GameProxy> game) : game_(std::move(game)) {
  auto processor = std::make_unique<path::NodeProcessor>(*game_);

  last_ship_change_ = 0;
  ship_ = game_->GetPlayer().ship;

  if (ship_ > 7) {
    ship_ = 0;
  }

  pathfinder_ = std::make_unique<path::Pathfinder>(std::move(processor));

  auto find_enemy = std::make_unique<FindEnemyNode>();
  auto looking_at_enemy = std::make_unique<LookingAtEnemyNode>();
  auto target_in_los = std::make_unique<InLineOfSightNode>(
      [](marvin::behavior::ExecuteContext& ctx) {
        const Vector2f* result = nullptr;

        const Player* target_player =
            ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

        if (target_player) {
          result = &target_player->position;
        }
        return result;
      });

  auto shoot_enemy = std::make_unique<ShootEnemyNode>();
  auto path_to_enemy = std::make_unique<PathToEnemyNode>();
  auto move_to_enemy = std::make_unique<MoveToEnemyNode>();
  auto follow_path = std::make_unique<FollowPathNode>();
  auto patrol = std::make_unique<PatrolNode>();

  auto shoot_sequence = std::make_unique<behavior::SequenceNode>(
      looking_at_enemy.get(), shoot_enemy.get());
  auto parallel_shoot_enemy = std::make_unique<behavior::ParallelNode>(
      shoot_sequence.get(), move_to_enemy.get());
  auto los_shoot_conditional = std::make_unique<behavior::SequenceNode>(
      target_in_los.get(), parallel_shoot_enemy.get());

  auto enemy_path_sequence = std::make_unique<behavior::SequenceNode>(
      path_to_enemy.get(), follow_path.get());

  auto patrol_path_sequence =
      std::make_unique<behavior::SequenceNode>(patrol.get(), follow_path.get());

  auto path_or_shoot_selector = std::make_unique<behavior::SelectorNode>(
      los_shoot_conditional.get(), enemy_path_sequence.get());

  auto handle_enemy = std::make_unique<behavior::SequenceNode>(
      find_enemy.get(), path_or_shoot_selector.get());

  auto root_selector = std::make_unique<behavior::SelectorNode>(
      handle_enemy.get(), patrol_path_sequence.get());

  behavior_nodes_.push_back(std::move(find_enemy));
  behavior_nodes_.push_back(std::move(looking_at_enemy));
  behavior_nodes_.push_back(std::move(target_in_los));
  behavior_nodes_.push_back(std::move(shoot_enemy));
  behavior_nodes_.push_back(std::move(path_to_enemy));
  behavior_nodes_.push_back(std::move(move_to_enemy));
  behavior_nodes_.push_back(std::move(follow_path));
  behavior_nodes_.push_back(std::move(patrol));

  behavior_nodes_.push_back(std::move(shoot_sequence));
  behavior_nodes_.push_back(std::move(parallel_shoot_enemy));
  behavior_nodes_.push_back(std::move(los_shoot_conditional));
  behavior_nodes_.push_back(std::move(enemy_path_sequence));
  behavior_nodes_.push_back(std::move(patrol_path_sequence));
  behavior_nodes_.push_back(std::move(path_or_shoot_selector));
  behavior_nodes_.push_back(std::move(handle_enemy));
  behavior_nodes_.push_back(std::move(root_selector));

  behavior_ =
      std::make_unique<behavior::BehaviorEngine>(behavior_nodes_.back().get());

  regions_ = RegionRegistry::Create(game_->GetMap());

  behavior_ctx_.blackboard.Set(
      "patrol_nodes",
      std::vector<Vector2f>({Vector2f(585, 540), Vector2f(400, 570)}));

  behavior_ctx_.blackboard.Set("patrol_index", 0);
}

void Bot::Update(float dt) {
  keys_.ReleaseAll();
  game_->Update(dt);

  if (game_->GetPlayer().ship > 7) {
    uint64_t timestamp =
        std::chrono::time_point_cast<std::chrono::duration<uint64_t>>(
            std::chrono::high_resolution_clock::now())
            .time_since_epoch()
            .count();

    if (timestamp - last_ship_change_ > 10) {
      if (game_->SetShip(ship_)) {
        last_ship_change_ = timestamp;
      }
    }
    return;
  }

  MapCoord spawn = game_->GetSettings().SpawnSettings[0].GetCoord();
  MapCoord current_coord((uint16_t)game_->GetPosition().x,
                         (uint16_t)game_->GetPosition().y);

  if (!regions_->IsConnected(current_coord, spawn)) {
    game_->Warp();
    return;
  }

  this->steering_ = Vector2f();

  behavior_ctx_.bot = this;
  behavior_ctx_.dt = dt;
  behavior_ctx_.blackboard.Set("path", path_);

  behavior_->Update(behavior_ctx_);

  path_ = behavior_ctx_.blackboard.ValueOr("path", path_);

  Steer();
}

void Bot::Steer() {
  if (steering_.LengthSq() <= 0.0f) return;
  // debug_log << "Steering: " << steering_ << std::endl;

  Vector2f trajectory = steering_ - game_->GetPlayer().velocity;
  Vector2f direction = marvin::Normalize(trajectory);
  Vector2f heading = game_->GetPlayer().GetHeading();

  // Simple movement controller
  auto perp = marvin::Perpendicular(heading);
  float dot = heading.Dot(direction);
  bool clockwise = perp.Dot(direction) >= 0.0;
  bool target_behind = steering_.Dot(Normalize(trajectory)) < 0.0f;

  float threshold = 0.15f;

  if (target_behind) {
    clockwise = !clockwise;
  }

  if (dot < -threshold) {
    keys_.Press(VK_DOWN);
  } else if (dot > threshold) {
    keys_.Press(VK_UP);
  }

  if (dot < 1.0f) {
    keys_.Set(VK_RIGHT, clockwise);
    keys_.Set(VK_LEFT, !clockwise);
  }
}

void Bot::Move(const Vector2f& target, float target_distance) {
  const Player& bot_player = game_->GetPlayer();
  Vector2f seek = target - game_->GetPosition();
  float distance = bot_player.position.Distance(target);
  float maxspeed = game_->GetShipSettings().InitialSpeed / 10.0f / 16.0f;

  if (distance > target_distance) {
    this->steering_ += Normalize(seek) * maxspeed;
  } else {
    this->steering_ -= Normalize(bot_player.GetHeading()) * maxspeed;
  }
}

}  // namespace marvin
