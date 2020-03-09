#include "ContinuumGameProxy.h"

#include <vector>

namespace marvin {

ContinuumGameProxy::ContinuumGameProxy() {
  module_base_continuum_ = process_.GetModuleBase("Continuum.exe");
  module_base_menu_ = process_.GetModuleBase("menu040.dll");

  game_addr_ = process_.ReadU32(module_base_continuum_ + 0xC1AFC);

  position_data_ = (uint32_t*)(game_addr_ + 0x126BC);
}

void ContinuumGameProxy::Update(float dt) { FetchPlayers(); }

void ContinuumGameProxy::FetchPlayers() {
  const std::size_t kPosOffset = 0x04;
  const std::size_t kVelocityOffset = 0x10;
  const std::size_t kIdOffset = 0x18;
  const std::size_t kRotOffset = 0x3C;
  const std::size_t kShipOffset = 0x5C;
  const std::size_t kFreqOffset = 0x58;
  const std::size_t kStatusOffset = 0x60;
  const std::size_t kNameOffset = 0x6D;

  std::size_t base_addr = game_addr_ + 0x127EC;
  std::size_t players_addr = base_addr + 0x884;
  std::size_t count_addr = base_addr + 0x1884;

  std::size_t count = process_.ReadU32(count_addr) & 0xFFFF;

  players_.clear();

  for (std::size_t i = 0; i < count; ++i) {
    std::size_t player_addr = process_.ReadU32(players_addr + (i * 4));

    if (!player_addr) continue;

    Player player;

    player.position.x =
        process_.ReadU32(player_addr + kPosOffset) / 1000.0f / 16.0f;
    player.position.y =
        process_.ReadU32(player_addr + kPosOffset + 4) / 1000.0f / 16.0f;

    player.velocity.x =
        process_.ReadU32(player_addr + kVelocityOffset) / 10.0f / 16.0f;
    player.velocity.x =
        process_.ReadU32(player_addr + kVelocityOffset + 4) / 10.0f / 16.0f;

    player.id =
        static_cast<uint16_t>(process_.ReadU32(player_addr + kIdOffset));
    player.discrete_rotation = static_cast<uint16_t>(
        process_.ReadU32(player_addr + kRotOffset) / 1000);

    player.ship =
        static_cast<uint8_t>(process_.ReadU32(player_addr + kShipOffset));
    player.frequency =
        static_cast<uint16_t>(process_.ReadU32(player_addr + kFreqOffset));

    player.status =
        static_cast<uint8_t>(process_.ReadU32(player_addr + kStatusOffset));

    player.name = process_.ReadString(player_addr + kNameOffset, 23);

    players_.emplace_back(player);
  }
}

std::string ContinuumGameProxy::GetName() const {
  const std::size_t ProfileStructSize = 2860;

  uint16_t profile_index =
      process_.ReadU32(module_base_menu_ + 0x47FA0) & 0xFFFF;
  std::size_t addr = process_.ReadU32(module_base_menu_ + 0x47A38) + 0x15;

  if (addr == 0) {
    return "";
  }

  addr += profile_index * ProfileStructSize;

  std::string name = process_.ReadString(addr, 23);

  name = name.substr(0, strlen(name.c_str()));

  return name;
}

Vector2f ContinuumGameProxy::GetPosition() const {
  float x = (*position_data_) / 16.0f;
  float y = (*position_data_) / 16.0f;

  return Vector2f(x, y);
}

const std::vector<Player>& ContinuumGameProxy::GetPlayers() const {
  return players_;
}

void ContinuumGameProxy::SetWindowFocus() {
  std::size_t focus_addr = game_addr_ + 0x3039c;

  process_.WriteU32(focus_addr, 1);
}

ExeProcess& ContinuumGameProxy::GetProcess() { return process_; }

}  // namespace marvin
