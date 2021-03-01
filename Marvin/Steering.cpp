#include "Steering.h"

#include <algorithm>
#include <cmath>

#include "Bot.h"
#include "Debug.h"
#include "Player.h"

namespace marvin {

float GetCurrentSpeed(Bot& bot) {
  return bot.GetGame().GetPlayer().velocity.Length();
}

// Context sensitive max speed to handle wormhole increases
float GetMaxSpeed(Bot& bot) {
  float speed = bot.GetGame().GetShipStatus().speed / 10.0f / 16.0f;

  if (GetCurrentSpeed(bot) > speed) {
    speed = std::abs(speed + bot.GetGame().GetShipSettings().GravityTopSpeed);
  }

  return speed;
}

SteeringBehavior::SteeringBehavior(Bot* bot) : bot_(bot), rotation_(0.0f) {}

Vector2f SteeringBehavior::GetSteering() {
  return force_;
}

float SteeringBehavior::GetRotation() {
  return rotation_;
}

void SteeringBehavior::Reset() {
  force_ = Vector2f();
  rotation_ = 0.0f;
}

void SteeringBehavior::Seek(Vector2f target, float multiplier) {
  float speed = GetMaxSpeed(*bot_);

  Vector2f desired = Normalize(target - bot_->GetGame().GetPosition()) * speed * multiplier;

  force_ += desired - bot_->GetGame().GetPlayer().velocity;
}

void SteeringBehavior::Flee(Vector2f target) {
  float speed = GetMaxSpeed(*bot_);

  Vector2f desired = Normalize(bot_->GetGame().GetPosition() - target) * speed;

  force_ += desired - bot_->GetGame().GetPlayer().velocity;
}

void SteeringBehavior::Arrive(Vector2f target, float deceleration) {
  float max_speed = GetMaxSpeed(*bot_);

  Vector2f to_target = target - bot_->GetGame().GetPosition();
  float distance = to_target.Length();

  if (distance > 0) {
    float speed = distance / deceleration;

    speed = std::min(speed, max_speed);

    Vector2f desired = to_target * (speed / distance);

    force_ += desired - bot_->GetGame().GetPlayer().velocity;
  }
}

void SteeringBehavior::Pursue(const Player& enemy) {
  const Player& player = bot_->GetGame().GetPlayer();
  float max_speed = GetMaxSpeed(*bot_);
  Vector2f to_enemy = enemy.position - bot_->GetGame().GetPosition();
  float dot = player.GetHeading().Dot(enemy.GetHeading());

  if (to_enemy.Dot(player.GetHeading()) > 0 && dot < -0.95f) {
    Seek(enemy.position);
  } else {
    float t = to_enemy.Length() / (max_speed + enemy.velocity.Length());

    Seek(enemy.position + enemy.velocity * t);
  }
}

void SteeringBehavior::Face(Vector2f target) {
  Vector2f to_target = target - bot_->GetGame().GetPosition();
  Vector2f heading = Rotate(bot_->GetGame().GetPlayer().GetHeading(), rotation_);

  float rotation = std::atan2(heading.y, heading.x) - std::atan2(to_target.y, to_target.x);

  rotation_ += WrapToPi(rotation);
}

void SteeringBehavior::AvoidWalls(float max_look_ahead) {
  constexpr float kDegToRad = 3.14159f / 180.0f;
  constexpr size_t kFeelerCount = 29;

  static_assert(kFeelerCount & 1, "Feeler count must be odd");

  auto& game = bot_->GetGame();
  Vector2f feelers[kFeelerCount];

  feelers[0] = Normalize(game.GetPlayer().velocity);

  for (size_t i = 1; i < kFeelerCount; i += 2) {
    feelers[i] = Rotate(feelers[0], kDegToRad * (90.0f / kFeelerCount) * i);
    feelers[i + 1] = Rotate(feelers[0], -kDegToRad * (90.0f / kFeelerCount) * i);
  }

  float speed = game.GetPlayer().velocity.Length();
  float max_speed = GetMaxSpeed(*bot_);
  float look_ahead = max_look_ahead * (speed / max_speed);

  size_t force_count = 0;
  Vector2f force;

  for (size_t i = 0; i < kFeelerCount; ++i) {
    float intensity = feelers[i].Dot(Normalize(game.GetPlayer().velocity));
    float check_distance = look_ahead * intensity;
    Vector2f check = feelers[i] * intensity;
    CastResult result = RayCast(game.GetMap(), game.GetPlayer().position, Normalize(feelers[i]), check_distance);
    COLORREF color = RGB(100, 0, 0);

    if (result.hit) {
      float multiplier = ((check_distance - result.distance) / check_distance) * 1.5f;

      force += Normalize(feelers[i]) * -Normalize(feelers[i]).Dot(feelers[0]) * multiplier * max_speed;

      ++force_count;
    } else {
      result.distance = check_distance;
      color = RGB(0, 100, 0);
    }

    RenderWorldLine(game.GetPosition(), game.GetPosition(),
                    game.GetPosition() + Normalize(feelers[i]) * result.distance, color);
  }

  if (force_count > 0) {
    force_ += force * (1.0f / force_count);
  }
}

} // namespace marvin
