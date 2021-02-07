#pragma once

#include <cstddef>
#include <memory>

#include "../GameProxy.h"
#include "ExeProcess.h"

namespace marvin {

#pragma pack(push, 1)
struct WeaponData {
  u32 _unused1;

  u32 x;  // 0x04
  u32 y;  // 0x08

  u32 _unuseda;
  i32 velocity_x;
  i32 velocity_y;
  u32 _unused2[32];

  u32 pid;  // 0x98

  char _unused3[11];

  u16 type;
};
#pragma pack(pop)

// In memory weapon data
class ContinuumWeapon : public Weapon {
 public:
  ContinuumWeapon(WeaponData* data) : weapon_(data) {}

  u16 GetPlayerId() const { return weapon_->pid; }

  Vector2f GetPosition() const {
    return Vector2f(weapon_->x / 1000.0f / 16.0f, weapon_->y / 1000.0f / 16.0f);
  }

  Vector2f GetVelocity() const {
    return Vector2f(weapon_->velocity_x / 1000.0f / 16.0f,
                    weapon_->velocity_y / 1000.0f / 16.0f);
  }

  u16 GetType() const { return weapon_->type; }

 private:
  WeaponData* weapon_;
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

  bool SetShip(int ship) override;
  void Warp() override;
  void Cloak(KeyController& keys) override;

  void SendChatMessage(const std::string& mesg) const override;

  void SetWindowFocus();

  ExeProcess& GetProcess();

 private:
  void SendKey(int vKey);
  void FetchPlayers();

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
};

}  // namespace marvin
