#pragma once

#include <fstream>
#include <memory>

#include "KeyController.h"
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

  const std::vector<Vector2f>& GetPath() const { return path_; }

 private:
  void UpdateProofOfConcept();
  void PerformPathing(const Player& bot_player, const Player& target);
  void AttackTarget(const Player& bot_player, const Player& target,
                    float distance);

  std::unique_ptr<GameProxy> game_;
  std::unique_ptr<path::Pathfinder> pathfinder_;
  std::vector<Vector2f> path_;
  std::unique_ptr<behavior::BehaviorEngine> behavior_;

  std::vector<std::unique_ptr<behavior::BehaviorNode>> behavior_nodes_;

  // TODO: Action-key map would be more versatile
  KeyController keys_;
};

}  // namespace marvin
