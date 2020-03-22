#pragma once

#include "Vector2f.h"

namespace marvin {

class Bot;
struct Player;

class SteeringBehavior {
 public:
  SteeringBehavior(Bot* bot);

  Vector2f GetSteering();
  float GetRotation();

  void Reset();

  void Seek(Vector2f target, float multiplier = 1.0f);
  void Flee(Vector2f target);
  void Arrive(Vector2f target, float deceleration);
  void Pursue(const Player& enemy);
  void Face(Vector2f target);

 private:
  Bot* bot_;
  Vector2f force_;
  float rotation_;
};

}  // namespace marvin
