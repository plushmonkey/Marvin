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

enum class WeaponType : short { None, Bullet, BouncingBullet, Bomb, ProximityBomb, Repel, Decoy, Burst, Thor };

struct WeaponData {
  WeaponType type : 5;
  u16 level : 2;
  u16 shrapbouncing : 1;
  u16 shraplevel : 2;
  u16 shrap : 5;
  u16 alternate : 1;
};

class Weapon {
public:
  virtual u16 GetPlayerId() const = 0;
  virtual Vector2f GetPosition() const = 0;
  virtual Vector2f GetVelocity() const = 0;
  virtual WeaponData GetData() const = 0;
  virtual u32 GetAliveTicks() const = 0;
  virtual u32 GetRemainingTicks() const = 0;
  virtual s32 GetRemainingBounces() const = 0;

  inline bool IsMine() const {
    WeaponData data = GetData();

    return data.alternate && (data.type == WeaponType::Bomb || data.type == WeaponType::ProximityBomb);
  }
};

struct ShipStatus {
  u32 max_energy;
  u32 recharge;
  u32 thrust;
  u32 speed;
  u32 rotation;
  u32 shrapnel;
};

struct Flag {
  u32 id;
  u32 frequency;
  Vector2f position;

  Flag(u32 id, u32 freq, Vector2f pos) : id(id), frequency(freq), position(pos) {}

  bool IsNeutral() const {
    return frequency == 0xFFFFFFFF;
  }
};


struct Green {
  s32 prize_id;
  u32 x;
  u32 y;
  u32 remaining_ticks;
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
  virtual std::vector<Flag> GetDroppedFlags() = 0;
  virtual const std::vector<Green>& GetGreens() const = 0;

  // May need to be called more than once to transition the game menu
  // Returns true if it attempts to set the ship this call.
  virtual bool SetShip(int ship) = 0;
  virtual void Warp() = 0;
  virtual void Cloak(KeyController& keys) = 0;

  virtual void SendChatMessage(const std::string& mesg) const = 0;
};

} // namespace marvin
