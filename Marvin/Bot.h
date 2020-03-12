#pragma once

#include <memory>

#include "KeyController.h"
#include "path/Pathfinder.h"

namespace marvin {

class GameProxy;
struct Player;

class Bot {
 public:
  Bot(std::unique_ptr<GameProxy> game);

  void Update(float dt);

  KeyController& GetKeys() { return keys_; }
  GameProxy& GetGame() { return *game_; }

 private:
  void UpdateProofOfConcept();
  void PerformPathing(const Player& bot_player, const Player& target);
  void AttackTarget(const Player& bot_player, const Player& target,
                    float distance);
  void Move(const Vector2f& target, float target_distance);

  void SmoothPath();

  std::unique_ptr<GameProxy> game_;
  std::unique_ptr<path::Pathfinder> pathfinder_;
  std::vector<Vector2f> path_;

  // TODO: Action-key map would be more versatile
  KeyController keys_;
};

}  // namespace marvin
