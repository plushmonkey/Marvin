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

  void SetWindowFocus();

  ExeProcess& GetProcess();

 private:
  void FetchPlayers();

  ExeProcess process_;
  std::size_t module_base_continuum_;
  std::size_t module_base_menu_;
  std::size_t game_addr_;
  uint32_t* position_data_;

  std::vector<Player> players_;
};

}  // namespace marvin
