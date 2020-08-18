#include "Bot.h"

#include <chrono>
#include <cstring>
#include <limits>

#include "Debug.h"
#include "GameProxy.h"
#include "Map.h"
#include "RayCaster.h"
#include "RegionRegistry.h"
#include "behavior/nodes/FollowPathNode.h"
#include "platform/Platform.h"

namespace marvin {

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

    path =
        CreatePath(ctx, "path", from, to, game.GetShipSettings().GetRadius());

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

    if (seek_position.DistanceSq(target_player->position) < 15 * 15) {
      ctx.blackboard.Set("target_position", seek_position);
    } else {
      ctx.blackboard.Set("target_position", target.position);
    }

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
      float energy_pct =
          ctx.bot->GetGame().GetEnergy() /
          (float)ctx.bot->GetGame().GetShipSettings().InitialEnergy;
      uint64_t expiration = current_time + cooldown + (rand() % 1000);

      expiration -= (uint64_t)((cooldown / 2ULL) * energy_pct);

      ctx.blackboard.Set("BundleCooldownExpire", expiration);
      running_ = false;
      return behavior::ExecuteResult::Success;
    }

    auto& game = ctx.bot->GetGame();

    const Player& target =
        *ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

    if (!running_) {
      if (!ShouldActivate(ctx, target)) {
        return behavior::ExecuteResult::Failure;
      }

      start_time_ = current_time;
      running_ = true;
    }

    RenderText("BundleShots", GetWindowCenter() + Vector2f(0, 100),
               RGB(100, 100, 100), RenderText_Centered);

    ctx.bot->Move(target.position, 0.0f);
    ctx.bot->GetSteering().Face(target.position);

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
    const float hover_distance = 25.0f;
    const float aggression_min = 10.0f;

    auto& game = ctx.bot->GetGame();

    float aggression = ctx.blackboard.ValueOr("aggression", 0.0f);

    float energy_pct =
        game.GetEnergy() / (float)game.GetShipSettings().InitialEnergy;

    if (energy_pct < 0.5f) {
      aggression -= ctx.dt / 15.0f;

      if (aggression < -0.5f) {
        aggression = -0.5f;
      }
    } else {
      aggression += ctx.dt / 15.0f;

      if (aggression > 1.0f) {
        aggression = 1.0f;
      }
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
      ctx.bot->Move(target_position, distance_max);
    }

    const Player& shooter =
        *ctx.blackboard.ValueOr<const Player*>("target_player", nullptr);

#if 0 // Simple weapon avoidance but doesn't work great
    bool weapon_dodged = false;
    for (Weapon* weapon : game.GetWeapons()) {
      const Player* weapon_player = game.GetPlayerById(weapon->GetPlayerId());

      if (weapon_player == nullptr) continue;

      if (weapon_player->frequency == game.GetPlayer().frequency) continue;

      const auto& player = game.GetPlayer();

      if (weapon->GetType() & 0x8F00 && weapon->GetPosition().DistanceSq(player.position) < 20 * 20) {
        Vector2f direction = Normalize(player.position - weapon->GetPosition());
        Vector2f perp = Perpendicular(direction);

        if (perp.Dot(player.velocity) < 0) {
          perp = -perp;
        }

        ctx.bot->GetSteering().Seek(player.position + perp, 100.0f);
        weapon_dodged = true;
      } else if (weapon->GetPosition().DistanceSq(player.position) < 50 * 50) {
        Vector2f direction = Normalize(weapon->GetVelocity());

        float radius = game.GetShipSettings(player.ship).GetRadius() * 6.0f;
        Vector2f box_pos = player.position - Vector2f(radius, radius);
        Vector2f extent(radius * 2, radius * 2);

        float dist;
        Vector2f norm;

        if (RayBoxIntersect(weapon->GetPosition(), direction, box_pos, extent, &dist, &norm)) {
          Vector2f perp = Perpendicular(direction);

          if (perp.Dot(player.velocity) < 0) {
            perp = perp * -1.0f;
          }

          ctx.bot->GetSteering().Seek(player.position + perp, 2.0f);
          weapon_dodged = true;
        }
      }
    }

    if (weapon_dodged) {
      return behavior::ExecuteResult::Success;
    }
#endif

#if 0
    Vector2f dodge;
    for (auto& player : game.GetPlayers()) {
      if (player.position.DistanceSq(game.GetPlayer().position) > 25 * 25) continue;
      if (player.id == game.GetPlayer().id) continue;
      if (player.frequency == game.GetPlayer().frequency) continue;
      if (player.name[0] == '<') continue;
      if (player.ship > 7) continue;

      Vector2f delta;

      if (energy_pct < 0.75f && IsAimingAt(game, player, game.GetPlayer(), &delta)) {
        dodge += delta;
      }
    }

    if (dodge.LengthSq() > 0) {
      ctx.bot->GetSteering().Seek(game.GetPosition() + dodge, 100.0f);
      return behavior::ExecuteResult::Success;
    }
#else
    float dodge_dist_sq = (distance_min * 0.35f) * (distance_min * 0.35f);

    if (game.GetPlayer().position.DistanceSq(shooter.position) >
        dodge_dist_sq) {
      Vector2f dodge;

      if (energy_pct < 0.75f &&
          IsAimingAt(game, shooter, game.GetPlayer(), &dodge)) {
        ctx.bot->GetSteering().Seek(game.GetPosition() + dodge, 100.0f);
        return behavior::ExecuteResult::Success;
      }
    }
#endif

    if (energy_pct > 0.35f) {
      ctx.bot->GetSteering().Face(target_position);
    }

    return behavior::ExecuteResult::Success;
  }

 private:
  bool IsAimingAt(GameProxy& game, const Player& shooter, const Player& target,
                  Vector2f* dodge) {
    const float kRadiusMultiplier = 1.5f;

    float proj_speed =
        game.GetShipSettings(shooter.ship).BulletSpeed / 10.0f / 16.0f;
    float radius =
        game.GetShipSettings(target.ship).GetRadius() * kRadiusMultiplier;
    Vector2f box_pos = target.position - Vector2f(radius, radius);

    Vector2f shoot_direction =
        Normalize(shooter.velocity + (shooter.GetHeading() * proj_speed));

    if (shoot_direction.Dot(shooter.GetHeading()) < 0) {
      shoot_direction = shooter.GetHeading();
    }

    Vector2f extent(radius * 2, radius * 2);

    float shooter_radius = game.GetShipSettings(shooter.ship).GetRadius();
    Vector2f side = Perpendicular(shooter.GetHeading()) * shooter_radius *
                    kRadiusMultiplier;

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
        if (distance < 30) {
          Vector2f hit = shooter.position + shoot_direction * distance;

          *dodge = Normalize(side * side.Dot(Normalize(hit - target.position)));
          return true;
        }
#endif

        if (distance < 30) {
          *dodge = Perpendicular(direction);

          return true;
        }
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

    Path path =
        CreatePath(ctx, "path", from, to, game.GetShipSettings().GetRadius());

    ctx.blackboard.Set("path", path);

    return behavior::ExecuteResult::Success;
  }
};

struct WarpNode : public behavior::BehaviorNode {
  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) {
    auto& game = ctx.bot->GetGame();

    game.Warp();

    return behavior::ExecuteResult::Success;
  }
};

Bot::Bot(std::unique_ptr<GameProxy> game)
    : game_(std::move(game)), steering_(this) {
  auto processor = std::make_unique<path::NodeProcessor>(game_->GetMap());

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

  // Warp the player back to spawn when they leave the spawn area
  MapCoord spawn = game_->GetSettings().SpawnSettings[0].GetCoord();
  auto in_spawn_region = std::make_unique<InRegionNode>(spawn);
  auto invert_in_region =
      std::make_unique<behavior::InvertNode>(in_spawn_region.get());
  auto warp_node = std::make_unique<WarpNode>();
  auto warp_to_center = std::make_unique<behavior::SequenceNode>(
      invert_in_region.get(), warp_node.get());

  auto shoot_enemy = std::make_unique<ShootEnemyNode>();
  auto path_to_enemy = std::make_unique<PathToEnemyNode>();
  auto move_to_enemy = std::make_unique<MoveToEnemyNode>();
  auto follow_path = std::make_unique<behavior::FollowPathNode>();
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
      warp_to_center.get(), handle_enemy.get(), patrol_path_sequence.get());

  behavior_nodes_.push_back(std::move(in_spawn_region));
  behavior_nodes_.push_back(std::move(invert_in_region));
  behavior_nodes_.push_back(std::move(warp_node));
  behavior_nodes_.push_back(std::move(warp_to_center));

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

#if !DEBUG_USER_CONTROL
  if (game_->GetPlayer().ship > 7) {
    uint64_t timestamp = GetTime();

    if (timestamp - last_ship_change_ > 10000) {
      if (game_->SetShip(ship_)) {
        last_ship_change_ = timestamp;
      }
    }
    return;
  }
#endif

  steering_.Reset();

  behavior_ctx_.bot = this;
  behavior_ctx_.dt = dt;

  behavior_->Update(behavior_ctx_);

  Steer();
}

void Bot::Steer() {
  Vector2f center = GetWindowCenter();
  float debug_y = 100;

  Vector2f force = steering_.GetSteering();
  float rotation = steering_.GetRotation();

  Vector2f heading = game_->GetPlayer().GetHeading();
  // Start out by trying to move in the direction that the bot is facing.
  Vector2f steering_direction = heading;

  bool has_force = force.LengthSq() > 0.0f;

  // If the steering system calculated any movement force, then set the movement
  // direction to it.
  if (has_force) {
    steering_direction = marvin::Normalize(force);
  }

  // Rotate toward the movement direction.
  Vector2f rotate_target = steering_direction;

  // If the steering system calculated any rotation then rotate from the heading
  // to desired orientation.
  if (rotation != 0.0f) {
    rotate_target = Rotate(heading, -rotation);
  }

  if (!has_force) {
    steering_direction = rotate_target;
  }

  Vector2f perp = marvin::Perpendicular(heading);
  bool behind = force.Dot(heading) < 0;
  // This is whether or not the steering direction is pointing to the left of
  // the ship.
  bool leftside = steering_direction.Dot(perp) < 0;

  // Cap the steering direction so it's pointing toward the rotate target.
  if (steering_direction.Dot(rotate_target) < 0.75) {
    RenderText("adjusting", center - Vector2f(0, debug_y), RGB(100, 100, 100),
               RenderText_Centered);
    debug_y -= 20;
    float rotation = 0.1f;
    int sign = leftside ? 1 : -1;

    if (behind) sign *= -1;

    // Pick the side of the rotate target that is closest to the force
    // direction.
    steering_direction = Rotate(rotate_target, rotation * sign);

    leftside = steering_direction.Dot(perp) < 0;
  }

  bool clockwise = !leftside;

  if (has_force) {
    if (behind) {
      keys_.Press(VK_DOWN);
    } else {
      keys_.Press(VK_UP);
    }
  }

  if (heading.Dot(steering_direction) < 1.0f) {
    keys_.Set(VK_RIGHT, clockwise);
    keys_.Set(VK_LEFT, !clockwise);
  }

#if DEBUG_RENDER

  if (has_force) {
    Vector2f force_direction = Normalize(force);
    float force_percent =
        force.Length() /
        (GetGame().GetShipStatus().speed / 16.0f / 16.0f);
    RenderLine(center, center + (force_direction * 100 * force_percent),
               RGB(255, 255, 0));
  }

  RenderLine(center, center + (heading * 100), RGB(255, 0, 0));
  RenderLine(center, center + (perp * 100), RGB(100, 0, 100));
  RenderLine(center, center + (rotate_target * 85), RGB(0, 0, 255));
  RenderLine(center, center + (steering_direction * 75), RGB(0, 255, 0));

  if (rotation == 0.0f) {
    RenderText("no rotation", center - Vector2f(0, debug_y), RGB(100, 100, 100),
               RenderText_Centered);
    debug_y -= 20;
  } else {
    std::string text = "rotation: " + std::to_string(rotation);
    RenderText(text.c_str(), center - Vector2f(0, debug_y), RGB(100, 100, 100),
               RenderText_Centered);
    debug_y -= 20;
  }

  if (behind) {
    RenderText("behind", center - Vector2f(0, debug_y), RGB(100, 100, 100),
               RenderText_Centered);
    debug_y -= 20;
  }

  if (leftside) {
    RenderText("leftside", center - Vector2f(0, debug_y), RGB(100, 100, 100),
               RenderText_Centered);
    debug_y -= 20;
  }

  if (rotation != 0.0f) {
    RenderText("face-locked", center - Vector2f(0, debug_y), RGB(100, 100, 100),
               RenderText_Centered);
    debug_y -= 20;
  }
#endif
}

void Bot::Move(const Vector2f& target, float target_distance) {
  const Player& bot_player = game_->GetPlayer();
  float distance = bot_player.position.Distance(target);

  if (distance > target_distance) {
    steering_.Seek(target);
  } else {
    Vector2f to_target = target - bot_player.position;

    steering_.Seek(target - Normalize(to_target) * target_distance);
  }
}

uint64_t Bot::GetTime() const {
  return std::chrono::time_point_cast<std::chrono::milliseconds>(
             std::chrono::high_resolution_clock::now())
      .time_since_epoch()
      .count();
}

}  // namespace marvin
