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

    if (current_target) {
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

class BundleShots : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    const uint64_t cooldown = 5000;
    const uint64_t duration = 1500;

    uint64_t bundle_expire =
        ctx.blackboard.ValueOr<uint64_t>("BundleCooldownExpire", 0);
    uint64_t current_time = ctx.bot->GetTime();

    if (current_time < bundle_expire) return behavior::ExecuteResult::Failure;

    if (running_ && current_time >= start_time_ + duration) {
      ctx.blackboard.Set("BundleCooldownExpire",
                         current_time + cooldown + (rand() % 1000));
      running_ = false;
      return behavior::ExecuteResult::Success;
    }

    auto& game = ctx.bot->GetGame();

    Vector2f target_position =
        ctx.blackboard.ValueOr("target_position", Vector2f());
    const Player& target =
        *ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    if (!running_) {
      if (!ShouldActivate(ctx, target)) {
        return behavior::ExecuteResult::Failure;
      }

      start_time_ = current_time;
      running_ = true;
    }

    ctx.bot->Move(target_position, 0.0f);
    ctx.bot->GetSteering().Face(target_position);

    ctx.bot->GetKeys().Press(VK_UP);
    ctx.bot->GetKeys().Press(VK_CONTROL);

    return behavior::ExecuteResult::Running;
  }

 private:
  bool ShouldActivate(behavior::ExecuteContext& ctx, const Player& target) {
    auto& game = ctx.bot->GetGame();
    Vector2f position = game.GetPosition();
    Vector2f velocity = game.GetPlayer().velocity;
    Vector2f relative_velocity = velocity - target.velocity;
    float dot = game.GetPlayer().GetHeading().Dot(
        Normalize(target.position - position));

    if (game.GetMap().GetTileId(game.GetPosition()) == kSafeTileId) {
      return false;
    }

    if (dot <= 0.7f) {
      return false;
    }

    if (relative_velocity.LengthSq() < 1.0f * 1.0f) {
      return true;
    }

    // Activate this if pointing near the target and moving away from them.
    if (game.GetPlayer().velocity.Dot(target.position - position) >= 0.0f) {
      return false;
    }

    return true;
  }

  uint64_t start_time_;
  bool running_;
};

class MoveToEnemyNode : public behavior::BehaviorNode {
 public:
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    const float hover_distance = 45.0f;
    const float aggression_min = 15.0f;

    auto& game = ctx.bot->GetGame();

    float aggression = ctx.blackboard.ValueOr("aggression", 0.0f);
    aggression += ctx.dt / 15.0f;

    if (aggression > 1.0f) {
      aggression = 1.0f;
    }

    ctx.blackboard.Set("aggression", aggression);
    Vector2f target_position =
        ctx.blackboard.ValueOr("target_position", Vector2f());

    float distance_min =
        hover_distance - (aggression * (hover_distance - aggression_min));
    float distance_max = hover_distance * 1.2f;

    MapCoord spawn =
        ctx.bot->GetGame().GetSettings().SpawnSettings[0].GetCoord();

    if (Vector2f(spawn.x, spawn.y)
            .DistanceSq(ctx.bot->GetGame().GetPosition()) < 35.0f * 35.0f) {
      distance_min = 0.0f;
      distance_max = 0.0f;
    }

    float distance = target_position.Distance(game.GetPlayer().position);

    if (distance < distance_min) {
      ctx.bot->Move(target_position, distance_min);
    } else if (distance > distance_max) {
      ctx.bot->Move(target_position, hover_distance);
    }

    float max_speed = game.GetShipSettings().MaximumSpeed / 10.0f / 16.0f;

    const Player& shooter =
        *ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    float dodge_dist_sq = (distance_min * 0.35f) * (distance_min * 0.35f);

    if (game.GetPlayer().position.DistanceSq(shooter.position) >
        dodge_dist_sq) {
      Vector2f dodge;

      if (IsAimingAt(game, shooter, game.GetPlayer(), &dodge)) {
        ctx.bot->GetSteering().Seek(game.GetPosition() + dodge, 100.0f);
        return behavior::ExecuteResult::Success;
      }
    }

    Vector2f heading = game.GetPlayer().GetHeading();

    float dot = heading.Dot(Normalize(target_position - game.GetPosition()));

    if (dot < 0.35f) {
      ctx.bot->GetSteering().Face(target_position);
    }

    return behavior::ExecuteResult::Success;
  }

 private:
  bool IsAimingAt(GameProxy& game, const Player& shooter, const Player& target,
                  Vector2f* dodge) {
    float proj_speed =
        game.GetShipSettings(shooter.ship).BulletSpeed / 10.0f / 16.0f;
    float radius = game.GetShipSettings(target.ship).GetRadius() * 1.5f;
    Vector2f box_pos = target.position - Vector2f(radius, radius);

    Vector2f shoot_direction =
        Normalize(shooter.velocity + (shooter.GetHeading() * proj_speed));

    if (shoot_direction.Dot(shooter.GetHeading()) < 0) {
      shoot_direction = shooter.GetHeading();
    }

    Vector2f extent(radius * 2, radius * 2);

    float shooter_radius = game.GetShipSettings(shooter.ship).GetRadius();
    Vector2f side = Perpendicular(shooter.GetHeading()) * shooter_radius * 1.5f;

    float distance;

    Vector2f directions[2] = {shoot_direction, shooter.GetHeading()};

    for (Vector2f direction : directions) {
      if (RayBoxIntersect(shooter.position, direction, box_pos, extent,
                          &distance, nullptr) ||
          RayBoxIntersect(shooter.position + side, direction, box_pos, extent,
                          &distance, nullptr) ||
          RayBoxIntersect(shooter.position - side, direction, box_pos, extent,
                          &distance, nullptr)) {
#if 0
        Vector2f hit = shooter.position + shoot_direction * distance;

        *dodge = Normalize(side * side.Dot(Normalize(hit - target.position)));
#else
        *dodge = Perpendicular(direction);
#endif

        return true;
      }
    }

    return false;
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

Bot::Bot(std::unique_ptr<GameProxy> game)
    : game_(std::move(game)), steering_(this) {
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
  auto bundle_shots = std::make_unique<BundleShots>();

  auto move_method_selector = std::make_unique<behavior::SelectorNode>(
      bundle_shots.get(), move_to_enemy.get());

  auto shoot_sequence = std::make_unique<behavior::SequenceNode>(
      looking_at_enemy.get(), shoot_enemy.get());
  auto parallel_shoot_enemy = std::make_unique<behavior::ParallelNode>(
      shoot_sequence.get(), move_method_selector.get());
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
  behavior_nodes_.push_back(std::move(bundle_shots));

  behavior_nodes_.push_back(std::move(move_method_selector));
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
    uint64_t timestamp = GetTime();

    if (timestamp - last_ship_change_ > 10000) {
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

  steering_.Reset();

  behavior_ctx_.bot = this;
  behavior_ctx_.dt = dt;
  behavior_ctx_.blackboard.Set("path", path_);

  behavior_->Update(behavior_ctx_);

  path_ = behavior_ctx_.blackboard.ValueOr("path", path_);

  Steer();
}

void Bot::Steer() {
  Vector2f force = steering_.GetSteering();
  float rotation = steering_.GetRotation();

  Vector2f heading = game_->GetPlayer().GetHeading();

  if (rotation != 0.0f) {
    float speed = game_->GetShipSettings().MaximumSpeed / 10.0f / 16.0f;
    force += Rotate(heading, rotation) * speed * 0.2f;
  }

  if (force.LengthSq() > 0.0f) {
    Vector2f direction = marvin::Normalize(force);

    // Simple movement controller
    auto perp = marvin::Perpendicular(heading);
    float dot = heading.Dot(direction);
    bool clockwise = perp.Dot(direction) >= 0.0;
    bool target_behind = force.Dot(direction) < 0.0f;

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
}

void Bot::Move(const Vector2f& target, float target_distance) {
  const Player& bot_player = game_->GetPlayer();
  float distance = bot_player.position.Distance(target);

  if (distance > target_distance) {
    steering_.Seek(target);
  } else {
    Vector2f to_target = target - bot_player.position;

    steering_.Seek(target -
                   Normalize(to_target) * (target_distance - distance));
  }
}

uint64_t Bot::GetTime() const {
  return std::chrono::time_point_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now())
      .time_since_epoch()
      .count();
}

}  // namespace marvin
