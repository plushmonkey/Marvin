#pragma once

#include <string>
#include <vector>

#include "ClientSettings.h"
#include "KeyController.h"
#include "Player.h"
#include "Types.h"
#include "Vector2f.h"

namespace marvin {

class Map;

class Weapon {
 public:
  virtual u16 GetPlayerId() const = 0;
  virtual Vector2f GetPosition() const = 0;
  virtual Vector2f GetVelocity() const = 0;
  virtual u16 GetType() const = 0;
};

struct ShipStatus {
  u32 recharge;
  u32 thrust;
  u32 speed;
  u32 rotation;
  u32 shrapnel;
};

class GameProxy {
 public:
  virtual ~GameProxy() {}

  virtual void Update(float dt) = 0;

  virtual std::string GetName() const = 0;
  virtual int GetEnergy() const = 0;
  virtual Vector2f GetPosition() const = 0;
  virtual const std::vector<Player>& GetPlayers() const = 0;
  virtual const ClientSettings& GetSettings() const = 0;
  virtual const ShipSettings& GetShipSettings() const = 0;
  virtual const ShipSettings& GetShipSettings(int ship) const = 0;
  virtual const ShipStatus& GetShipStatus() const = 0;
  virtual const Map& GetMap() const = 0;

  virtual const Player& GetPlayer() const = 0;
  virtual const Player& GetSelectedPlayer() const = 0;
  virtual const Player* GetPlayerById(u16 id) const = 0;

  virtual std::vector<Weapon*> GetWeapons() = 0;

  // May need to be called more than once to transition the game menu
  // Returns true if it attempts to set the ship this call.
  virtual bool SetShip(int ship) = 0;
  virtual void Warp() = 0;
  virtual void Cloak(KeyController& keys) = 0;

  virtual void SendChatMessage(const std::string& mesg) const = 0;
};

}  // namespace marvin
