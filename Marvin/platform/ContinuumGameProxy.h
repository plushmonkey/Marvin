#pragma once

#include <cstddef>
#include <memory>

#include "../GameProxy.h"
#include "ExeProcess.h"

namespace marvin {

class ContinuumGameProxy : public GameProxy {
 public:
  ContinuumGameProxy();

  void Update(float dt) override;

  std::string GetName() const override;
  Vector2f GetPosition() const override;
  const std::vector<Player>& GetPlayers() const override;
  const ClientSettings& GetSettings() const override;
  const ShipSettings& GetShipSettings() const override;
  std::string GetServerFolder() const;
  const Map& GetMap() const override;
  const Player& GetPlayer() const override;

  bool SetShip(int ship) override;
  void Warp() override;

  void SetWindowFocus();
  void SetWindow(HWND hwnd);

  ExeProcess& GetProcess();

 private:
  void FetchPlayers();
  void SendKey(int vKey);

  ExeProcess process_;
  HWND hwnd_;
  std::size_t module_base_continuum_;
  std::size_t module_base_menu_;
  std::size_t game_addr_;
  uint32_t* position_data_;
  uint16_t player_id_;
  std::unique_ptr<Map> map_;
  Player* player_;
  std::vector<Player> players_;
};

}  // namespace marvin
