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

Bot::Bot(std::unique_ptr<GameProxy> game) : game_(std::move(game)) {
  auto processor = std::make_unique<path::NodeProcessor>(game_->GetMap());

  pathfinder_ = std::make_unique<path::Pathfinder>(std::move(processor));
}

void Bot::Update(float dt) {
  keys_.ReleaseAll();
  game_->Update(dt);
  UpdateProofOfConcept();
}

Vector2f ClosestWall(const Map& map, Vector2f pos, int search) {
  double closest_dist = std::numeric_limits<double>::max();
  Vector2f closest;

  Vector2f base(std::floor(pos.x), std::floor(pos.y));
  for (int y = -search; y <= search; ++y) {
    for (int x = -search; x <= search; ++x) {
      Vector2f current = base + Vector2f(x, y);

      if (!map.IsSolid((unsigned short)current.x, (unsigned short)current.y)) {
        continue;
      }

      double dist = BoxPointDistance(current, Vector2f(1, 1), pos);

      if (dist < closest_dist) {
        closest_dist = dist;
        closest = current;
      }
    }
  }

  return closest;
}

bool IsPassablePath(const Map& map, Vector2f from, Vector2f to, double radius) {
  const Vector2f direction = Normalize(to - from);
  const Vector2f side = Perpendicular(direction) * (radius * 0.3);
  const double distance = from.Distance(to);

  CastResult cast_center = RayCast(map, from, direction, distance);
  //CastResult cast_side1 = RayCast(level, from + side, direction, distance);
  //CastResult cast_side2 = RayCast(level, from - side, direction, distance);

  return !cast_center.hit; //&& !cast_side1.hit && !cast_side2.hit;
}

void Bot::SmoothPath() {
  // How far away it should try to push the path from walls
  double push_distance = 2.1;

  int ship = game_->GetPlayer().ship;
  float radius = game_->GetSettings().ShipSettings[ship].Radius / 16.0f;

  std::vector<Vector2f> result;

  result.resize(path_.size());

  for (std::size_t i = 0; i < path_.size(); ++i) {
    Vector2f current = path_[i] + Vector2f(0.5, 0.5);
    Vector2f closest = ClosestWall(game_->GetMap(), current, (int)std::ceil(push_distance + 1));
    Vector2f new_pos = current;

#if 1
    if (closest != Vector2f(0, 0)) {
      // Attempt to push the path outward from the wall
      // TODO: iterative box penetration push

      Vector2f center = closest + Vector2f(0.5, 0.5);
      CastResult cast_result = RayCast(game_->GetMap(), current, center, push_distance + 1);
      Vector2f hit = cast_result.hit ? cast_result.position : center;

      //double dist = BoxPointDistance(closest, Vec2(1, 1), current);
      double dist = hit.Distance(current);
      double force = push_distance - dist;

      if (force < 0) force = 0;

      new_pos = current + Normalize(current - hit) * force;
      //new_pos = current + Vec2Normalize(current - (closest + Vec2(0.5, 0.5))) * force;

    }

    if (current != new_pos) {
      // Make sure the new node is in line of sight
#if 1
      if (!IsPassablePath(game_->GetMap(), current, new_pos, radius)) {
        new_pos = current;
      }
#else
      if (!Util::IsClearPath(current, new_pos, 1, level)) {
        new_pos = current;
      }
#endif
    }
#endif

    result[i] = new_pos;
  }

  if (result.size() <= 2) return;

#if 0
  for (int i = 0; i < 1; ++i) {
    result = PerformCulling(level, result, push_distance, radius);
  }
#endif

#if 1
  std::vector<Vector2f> minimum;
  minimum.reserve(result.size());

  minimum.push_back(result[0]);

  Vector2f prev = minimum[0];

  for (std::size_t i = 1; i < result.size(); ++i) {
    Vector2f curr = result[i];
    Vector2f direction = Normalize(curr - prev);
    Vector2f side = Perpendicular(direction) * radius;
    double dist = prev.Distance(curr);

    CastResult cast_center = RayCast(game_->GetMap(), prev, direction, dist);
    CastResult cast_side1 = RayCast(game_->GetMap(), prev + side, direction, dist);
    CastResult cast_side2 = RayCast(game_->GetMap(), prev - side, direction, dist);

    if (cast_center.hit || cast_side1.hit || cast_side2.hit) {
      if (minimum.size() > result.size()) {
        minimum = result;
        break;
      }

      if (result[i - 1] != minimum.back()) {
        minimum.push_back(result[i - 1]);
        prev = minimum.back();
        i--;
      } else {
        minimum.push_back(result[i]);
        prev = minimum.back();
      }
    }
  }

  minimum.push_back(result.back());

  result = minimum;
#endif
}

// Temporary code just to test
void Bot::PerformPathing(const Player& bot_player, const Player& target) {
  bool find_path = true;

#if 0
  if (!path_.empty()) {
    Vector2f goal = path_[path_.size() - 1];

    if (goal.DistanceSq(target.position) < 3) {
      find_path = false;
    }
  }
#endif

  if (find_path) {
    path_ = pathfinder_->FindPath(bot_player.position, target.position);
    SmoothPath();
  }

  if (path_.empty()) return;

  Vector2f current = path_.front();

  while (current.DistanceSq(bot_player.position) < 2 && !path_.empty()) {
    path_.erase(path_.begin());
    current = path_.front();
  }

  Move(current, 0.0f);
}

// Temporary code just to test
void Bot::AttackTarget(const Player& bot_player, const Player& target,
                       float distance) {
  const float kTargetDistance = 15.0f;
  float proj_speed =
      game_->GetSettings().ShipSettings[bot_player.ship].BulletSpeed / 10.0f /
      16.0f;

  Vector2f seek_position =
      CalculateShot(game_->GetPosition(), target.position, bot_player.velocity,
                    target.velocity, proj_speed);

  // marvin::Vector2f seek_position = target->position;// + (target->velocity
  // * prediction);
  Vector2f direction = Normalize(seek_position - bot_player.position);
  float dot = bot_player.GetHeading().Dot(direction);

  if (dot >= 0.95) {
    if (CanShoot(game_->GetMap(), bot_player, target)) {
      keys_.Press(VK_CONTROL);
    }
  }

  Move(seek_position, kTargetDistance);
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

void Bot::UpdateProofOfConcept() {
  // Simple proof of concept for targeting enemies and moving around
  auto& bot_player = game_->GetPlayer();

  const marvin::Player* target = nullptr;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < game_->GetPlayers().size(); ++i) {
    const marvin::Player& player = game_->GetPlayers()[i];

    if (player.id == game_->GetPlayer().id) continue;
    if (player.ship > 7) continue;
    if (player.frequency == bot_player.frequency) continue;
    if (game_->GetMap().GetTileId(player.position) == marvin::kSafeTileId) continue;
    if (!IsValidPosition(player.position)) continue;

    double dist_sq = bot_player.position.DistanceSq(player.position);

    if (dist_sq < closest_distance) {
      closest_distance = dist_sq;
      target = &game_->GetPlayers()[i];
    }
  }

  if (target) {
    const auto& ship_settings = game_->GetSettings().ShipSettings[target->ship];

    // Simple steering
    auto heading = bot_player.GetHeading();
    marvin::Vector2f trajectory = target->position - game_->GetPosition();
    float distance = trajectory.Length();
    float target_max_speed = ship_settings.MaximumSpeed / 10.0f / 16.0f;
    float prediction = distance / target_max_speed;

    CastResult cast = RayCast(game_->GetMap(), bot_player.position,
                              Normalize(trajectory), distance);

    if (cast.hit) {
      PerformPathing(bot_player, *target);
    } else {
      AttackTarget(bot_player, *target, distance);
    }
  }
}

}  // namespace marvin
