#include "ContinuumGameProxy.h"

#include <thread>
#include <vector>

#include "../Bot.h"
#include "../Map.h"

namespace marvin {

ContinuumGameProxy::ContinuumGameProxy(HWND hwnd) {
  module_base_continuum_ = process_.GetModuleBase("Continuum.exe");
  module_base_menu_ = process_.GetModuleBase("menu040.dll");
  player_id_ = 0xFFFF;
  hwnd_ = hwnd;

  game_addr_ = process_.ReadU32(module_base_continuum_ + 0xC1AFC);

  position_data_ = (uint32_t*)(game_addr_ + 0x126BC);

  // TODO: Either find this from memory or pass it in through config
  std::string path = GetServerFolder() + "\\jun2018.lvl";

  map_ = Map::Load(path);

  FetchPlayers();

  for (auto& player : players_) {
    if (player.name == GetName()) {
      player_id_ = player.id;
      player_ = &player;
    }
  }
}

void ContinuumGameProxy::Update(float dt) {
  // Continuum stops processing input when it loses focus, so update the memory
  // to make it think it always has focus.
  SetWindowFocus();

  FetchPlayers();
}

void ContinuumGameProxy::FetchPlayers() {
  const std::size_t kPosOffset = 0x04;
  const std::size_t kVelocityOffset = 0x10;
  const std::size_t kIdOffset = 0x18;
  const std::size_t kRotOffset = 0x3C;
  const std::size_t kShipOffset = 0x5C;
  const std::size_t kFreqOffset = 0x58;
  const std::size_t kStatusOffset = 0x60;
  const std::size_t kNameOffset = 0x6D;
  const std::size_t kEnergyOffset1 = 0x208;
  const std::size_t kEnergyOffset2 = 0x20C;

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
        process_.ReadI32(player_addr + kVelocityOffset) / 10.0f / 16.0f;
    player.velocity.y =
        process_.ReadI32(player_addr + kVelocityOffset + 4) / 10.0f / 16.0f;

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

    // Energy calculation @4485FA
    u32 energy1 = process_.ReadU32(player_addr + kEnergyOffset1);
    u32 energy2 = process_.ReadU32(player_addr + kEnergyOffset2);

    u32 combined = energy1 + energy2;
    u64 energy = ((combined * (u64)0x10624DD3) >> 32) >> 6;

    player.energy = static_cast<uint16_t>(energy);

    players_.emplace_back(player);

    if (player.id == player_id_) {
      player_ = &players_.back();
    }
  }
}

const ClientSettings& ContinuumGameProxy::GetSettings() const {
  std::size_t addr = game_addr_ + 0x127EC + 0x1AE70;  // 0x2D65C

  return *reinterpret_cast<ClientSettings*>(addr);
}

const ShipSettings& ContinuumGameProxy::GetShipSettings() const {
  return GetSettings().ShipSettings[player_->ship];
}

const ShipSettings& ContinuumGameProxy::GetShipSettings(int ship) const {
  return GetSettings().ShipSettings[ship];
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

int ContinuumGameProxy::GetEnergy() const { return player_->energy; }

Vector2f ContinuumGameProxy::GetPosition() const {
  float x = (*position_data_) / 16.0f;
  float y = (*(position_data_ + 1)) / 16.0f;

  return Vector2f(x, y);
}

const std::vector<Player>& ContinuumGameProxy::GetPlayers() const {
  return players_;
}

const Map& ContinuumGameProxy::GetMap() const { return *map_; }
const Player& ContinuumGameProxy::GetPlayer() const { return *player_; }

// TODO: Find level data or level name in memory
std::string ContinuumGameProxy::GetServerFolder() const {
  std::size_t folder_addr = *(uint32_t*)(game_addr_ + 0x127ec + 0x5a3c) + 0x10D;
  std::string server_folder = process_.ReadString(folder_addr, 256);

  return server_folder;
}

bool ContinuumGameProxy::SetShip(int ship) {
  int* menu_open_addr = (int*)(game_addr_ + 0x12F39);

  bool menu_open = *menu_open_addr;

  if (!menu_open) {
    SendKey(VK_ESCAPE);
  } else {
    SendMessage(hwnd_, WM_CHAR, (WPARAM)('1' + ship), 0);
  }

  return menu_open;
}

void ContinuumGameProxy::Warp() { SendKey(VK_INSERT); }

void ContinuumGameProxy::Cloak(KeyController& keys) {
  keys.Press(VK_SHIFT);
  SendKey(VK_HOME);
}

void ContinuumGameProxy::SetWindowFocus() {
  std::size_t focus_addr = game_addr_ + 0x3039c;

  process_.WriteU32(focus_addr, 1);
}

ExeProcess& ContinuumGameProxy::GetProcess() { return process_; }

void ContinuumGameProxy::SendKey(int vKey) {
  SendMessage(hwnd_, WM_KEYDOWN, (WPARAM)vKey, 0);
  SendMessage(hwnd_, WM_KEYUP, (WPARAM)vKey, 0);
}

void ContinuumGameProxy::SendChatMessage(const std::string& mesg) const {
  typedef void(__fastcall* ChatSendFunction)(void* This, void* thiscall_garbage, char* msg, u32 length, u32* rv);

  if (mesg.empty()) return;

  // The address to the current text input buffer
  std::size_t chat_input_addr = game_addr_ + 0x2DD14;
  char* input = (char*)(chat_input_addr);

  memcpy(input, mesg.c_str(), mesg.length());
  input[mesg.length()] = 0;

  ChatSendFunction send_func = (ChatSendFunction)(*(u32*)(module_base_continuum_ + 0xAC30C));
  u32 rv = 0;

  void* This = (void*)(game_addr_ + 0x2DBF0);

  send_func(This, nullptr, input, mesg.length(), &rv);

  // Clear the text buffer after sending the message
  input[0] = 0;
}

}  // namespace marvin
