#include "Steering.h"

#include <algorithm>
#include <cmath>

#include "Bot.h"
#include "Player.h"

namespace marvin {

SteeringBehavior::SteeringBehavior(Bot* bot) : bot_(bot), rotation_(0.0f) {}

Vector2f SteeringBehavior::GetSteering() { return force_; }

float SteeringBehavior::GetRotation() { return rotation_; }

void SteeringBehavior::Reset() {
  force_ = Vector2f();
  rotation_ = 0.0f;
}

void SteeringBehavior::Seek(Vector2f target, float multiplier) {
  float speed = bot_->GetGame().GetShipSettings().MaximumSpeed / 10.0f / 16.0f;
  Vector2f desired =
      Normalize(target - bot_->GetGame().GetPosition()) * speed * multiplier;

  force_ += desired - bot_->GetGame().GetPlayer().velocity;
}

void SteeringBehavior::Flee(Vector2f target) {
  float speed = bot_->GetGame().GetShipSettings().MaximumSpeed / 10.0f / 16.0f;
  Vector2f desired = Normalize(bot_->GetGame().GetPosition() - target) * speed;

  force_ += desired - bot_->GetGame().GetPlayer().velocity;
}

void SteeringBehavior::Arrive(Vector2f target, float deceleration) {
  float max_speed =
      bot_->GetGame().GetShipSettings().MaximumSpeed / 10.0f / 16.0f;

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
  float max_speed =
      bot_->GetGame().GetShipSettings().MaximumSpeed / 10.0f / 16.0f;
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
  Vector2f heading = bot_->GetGame().GetPlayer().GetHeading();

  float dot = heading.Dot(Normalize(to_target));

  rotation_ += std::acos(dot);
}

}  // namespace marvin
