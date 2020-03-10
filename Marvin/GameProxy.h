#pragma once

#include <string>
#include <vector>

#include "ClientSettings.h"
#include "Player.h"
#include "Vector2f.h"

namespace marvin {

class Map;

class GameProxy {
 public:
  virtual ~GameProxy() {}

  virtual void Update(float dt) = 0;

  virtual std::string GetName() const = 0;
  virtual Vector2f GetPosition() const = 0;
  virtual const std::vector<Player>& GetPlayers() const = 0;
  virtual const ClientSettings& GetSettings() const = 0;
  virtual const Map& GetMap() const = 0;
  virtual const Player& GetPlayer() const = 0;
};

}  // namespace marvin
