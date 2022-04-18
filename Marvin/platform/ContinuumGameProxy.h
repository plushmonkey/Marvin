#pragma once

#include <cstddef>
#include <memory>

#include "../GameProxy.h"
#include "ExeProcess.h"

namespace marvin {

#pragma pack(push, 1)
struct WeaponMemory {
  u32 vtable; // 0x00

  u32 x; // 0x04
  u32 y; // 0x08

  u32 _unuseda;   // 0x0C
  i32 velocity_x; // 0x10
  i32 velocity_y; // 0x14

  u32 _unused2[29];

  s32 remaining_bounces; // 0x8C

  u32 _unused3[2];

  u32 pid; // 0x98

  char _unused4[11];

  WeaponData data; // a7-a9

  char _unused5[23];
  u32 alive_ticks;

  u32 _unused6[1];
};
#pragma pack(pop)

static_assert(sizeof(WeaponMemory) > 0x40);

// In memory weapon data
class ContinuumWeapon : public Weapon {
public:
  ContinuumWeapon(WeaponMemory* memory, u32 remaining_ticks) : weapon_(memory), remaining_ticks_(remaining_ticks) {}

  u16 GetPlayerId() const {
    return weapon_->pid;
  }

  Vector2f GetPosition() const {
    return Vector2f(weapon_->x / 1000.0f / 16.0f, weapon_->y / 1000.0f / 16.0f);
  }

  Vector2f GetVelocity() const {
    return Vector2f(weapon_->velocity_x / 10.0f / 16.0f, weapon_->velocity_y / 10.0f / 16.0f);
  }

  WeaponData GetData() const {
    return weapon_->data;
  }
  
  u32 GetAliveTicks() const {
    return weapon_->alive_ticks;
  }

  u32 GetRemainingTicks() const {
    return remaining_ticks_;
  }

  s32 GetRemainingBounces() const {
    return weapon_->remaining_bounces;
  }

private:
  WeaponMemory* weapon_;
  u32 remaining_ticks_;
};

class ContinuumGameProxy : public GameProxy {
public:
  ContinuumGameProxy(HWND hwnd);

  void Update(float dt) override;

  std::string GetName() const override;
  int GetEnergy() const override;
  Vector2f GetPosition() const override;
  const std::vector<Player>& GetPlayers() const override;
  const ClientSettings& GetSettings() const override;
  const ShipSettings& GetShipSettings() const override;
  const ShipSettings& GetShipSettings(int ship) const override;
  const ShipStatus& GetShipStatus() const override;
  std::string GetServerFolder() const;
  std::string GetMapName() const;
  const Map& GetMap() const override;

  const Player& GetPlayer() const override;
  const Player& GetSelectedPlayer() const override;
  const Player* GetPlayerById(u16 id) const override;

  std::vector<Weapon*> GetWeapons() override;
  std::vector<Flag> GetDroppedFlags() override;
  const std::vector<Green>& GetGreens() const override;

  bool SetShip(int ship) override;
  void Warp() override;
  void Cloak(KeyController& keys) override;

  void SendChatMessage(const std::string& mesg) const override;

  void SetWindowFocus();

  ExeProcess& GetProcess();

private:
  void SendKey(int vKey);

  void FetchPlayers();
  void FetchWeapons();
  void FetchGreens();

  ExeProcess process_;
  HWND hwnd_;
  std::size_t module_base_continuum_;
  std::size_t module_base_menu_;
  std::size_t game_addr_;
  uint32_t* position_data_;
  uint16_t player_id_;
  std::unique_ptr<Map> map_;
  std::string map_name_;
  Player* player_;
  std::vector<Player> players_;
  std::vector<ContinuumWeapon> weapons_;
  ShipStatus ship_status_;
  std::vector<Green> greens_;
};

} // namespace marvin
