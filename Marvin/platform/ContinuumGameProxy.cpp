#include "ContinuumGameProxy.h"

#include <thread>
#include <vector>

#include "../Bot.h"
#include "../Debug.h"
#include "../Map.h"

namespace marvin {

ContinuumGameProxy::ContinuumGameProxy(HWND hwnd) {
  module_base_continuum_ = process_.GetModuleBase("Continuum.exe");
  module_base_menu_ = process_.GetModuleBase("menu040.dll");
  player_id_ = 0xFFFF;
  hwnd_ = hwnd;

  game_addr_ = process_.ReadU32(module_base_continuum_ + 0xC1AFC);

  position_data_ = (uint32_t*)(game_addr_ + 0x126BC);

  map_name_ = GetMapName();
  std::string path = GetServerFolder() + "\\" + map_name_;

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

  // Keep loaded map synchronized with the current arena map
  std::string new_map_name = GetMapName();
  if (new_map_name != this->map_name_) {
    std::string path = GetServerFolder() + "\\" + new_map_name;

    debug_log << "Loading new map " << path << std::endl;

    map_ = Map::Load(path);
    map_name_ = new_map_name;
  }

  FetchPlayers();
  FetchWeapons();
  FetchGreens();
}

void ContinuumGameProxy::FetchPlayers() {
  const std::size_t kPosOffset = 0x04;
  const std::size_t kVelocityOffset = 0x10;
  const std::size_t kIdOffset = 0x18;
  const std::size_t kBountyOffset1 = 0x20;
  const std::size_t kBountyOffset2 = 0x24;
  const std::size_t kFlagOffset1 = 0x30;
  const std::size_t kFlagOffset2 = 0x34;
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

    if (!player_addr)
      continue;

    Player player;

    player.position.x = process_.ReadU32(player_addr + kPosOffset) / 1000.0f / 16.0f;
    player.position.y = process_.ReadU32(player_addr + kPosOffset + 4) / 1000.0f / 16.0f;

    player.velocity.x = process_.ReadI32(player_addr + kVelocityOffset) / 10.0f / 16.0f;
    player.velocity.y = process_.ReadI32(player_addr + kVelocityOffset + 4) / 10.0f / 16.0f;

    player.id = static_cast<uint16_t>(process_.ReadU32(player_addr + kIdOffset));
    player.discrete_rotation = static_cast<uint16_t>(process_.ReadU32(player_addr + kRotOffset) / 1000);

    player.ship = static_cast<uint8_t>(process_.ReadU32(player_addr + kShipOffset));
    player.frequency = static_cast<uint16_t>(process_.ReadU32(player_addr + kFreqOffset));

    player.status = static_cast<uint8_t>(process_.ReadU32(player_addr + kStatusOffset));

    player.name = process_.ReadString(player_addr + kNameOffset, 23);

    player.bounty = *(u32*)(player_addr + kBountyOffset1) + *(u32*)(player_addr + kBountyOffset2);
    player.flags = *(u32*)(player_addr + kFlagOffset1) + *(u32*)(player_addr + kFlagOffset2);

    if (player.id == player_id_) {
      // Energy calculation @4485FA
      u32 energy1 = process_.ReadU32(player_addr + kEnergyOffset1);
      u32 energy2 = process_.ReadU32(player_addr + kEnergyOffset2);

      u32 combined = energy1 + energy2;
      u64 energy = ((combined * (u64)0x10624DD3) >> 32) >> 6;

      player.energy = static_cast<uint16_t>(energy);
    } else {
      u32 first = *(u32*)(player_addr + 0x150);
      u32 second = *(u32*)(player_addr + 0x154);

      player.energy = static_cast<uint16_t>(first + second);
    }

    players_.emplace_back(player);

    if (player.id == player_id_) {
      player_ = &players_.back();

      // @448D37
      ship_status_.rotation = *(u32*)(player_addr + 0x278) + *(u32*)(player_addr + 0x274);
      ship_status_.recharge = *(u32*)(player_addr + 0x1E8) + *(u32*)(player_addr + 0x1EC);
      ship_status_.shrapnel = *(u32*)(player_addr + 0x2A8) + *(u32*)(player_addr + 0x2AC);
      ship_status_.thrust = *(u32*)(player_addr + 0x244) + *(u32*)(player_addr + 0x248);
      ship_status_.speed = *(u32*)(player_addr + 0x350) + *(u32*)(player_addr + 0x354);
      ship_status_.max_energy = *(u32*)(player_addr + 0x1C8) + *(u32*)(player_addr + 0x1C4);
    }
  }
}

void ContinuumGameProxy::FetchWeapons() {
  // Grab the address to the main player structure
  u32 player_addr = *(u32*)(game_addr_ + 0x13070);

  // Follow a pointer that leads to weapon data
  u32 ptr = *(u32*)(player_addr + 0x0C);
  u32 weapon_count = *(u32*)(ptr + 0x1DD0) + *(u32*)(ptr + 0x1DD4);
  u32 weapon_ptrs = (ptr + 0x21F4);

  weapons_.clear();

  for (size_t i = 0; i < weapon_count; ++i) {
    u32 weapon_data = *(u32*)(weapon_ptrs + i * 4);

    WeaponMemory* data = (WeaponMemory*)(weapon_data);
    WeaponType type = data->data.type;

    u32 total_ticks = 0;
    
    switch (type) {
        case WeaponType::Bomb:
        case WeaponType::ProximityBomb:
        case WeaponType::Thor: {
          total_ticks = this->GetSettings().BombAliveTime;
          if (data->data.alternate) {
            total_ticks = this->GetSettings().MineAliveTime;
          }
        } break;
        case WeaponType::Burst:
        case WeaponType::Bullet:
        case WeaponType::BouncingBullet: {
          total_ticks = this->GetSettings().BulletAliveTime;
        } break;
        case WeaponType::Repel: {
          total_ticks = this->GetSettings().RepelTime;
        } break;
        case WeaponType::Decoy: {
          total_ticks = this->GetSettings().DecoyAliveTime;
        } break;
        default: {
          total_ticks = this->GetSettings().BulletAliveTime;
        } break;
    }

    u32 alive_ticks = data->alive_ticks;

    if (alive_ticks > total_ticks) {
      alive_ticks = total_ticks;
    }

    weapons_.emplace_back(data, total_ticks - alive_ticks);
  }
}

void ContinuumGameProxy::FetchGreens() {
  greens_.clear();

  u32 green_count = *(u32*)(game_addr_ + 0x2e350);
  Green* greens = (Green*)(game_addr_ + 0x2df50);

  for (size_t i = 0; i < green_count; ++i) {
    greens_.push_back(greens[i]);
  }
}

const ShipStatus& ContinuumGameProxy::GetShipStatus() const {
  return ship_status_;
}

std::vector<Weapon*> ContinuumGameProxy::GetWeapons() {
  std::vector<Weapon*> weapons;

  for (std::size_t i = 0; i < weapons_.size(); ++i) {
    weapons.push_back(&weapons_[i]);
  }

  return weapons;
}

std::vector<Flag> ContinuumGameProxy::GetDroppedFlags() {
  u32 flag_count = *(u32*)(game_addr_ + 0x127ec + 0x1d4c);
  u32** flag_ptrs = (u32**)(game_addr_ + 0x127ec + 0x188c);

  std::vector<Flag> flags;
  flags.reserve(flag_count);

  for (size_t i = 0; i < flag_count; ++i) {
    char* current = (char*)flag_ptrs[i];
    u32 flag_id = *(u32*)(current + 0x1C);
    u32 x = *(u32*)(current + 0x04);
    u32 y = *(u32*)(current + 0x08);
    u32 frequency = *(u32*)(current + 0x14);

    flags.emplace_back(flag_id, frequency, Vector2f(x / 16000.0f, y / 16000.0f));
  }

  return flags;
}

const std::vector<Green>& ContinuumGameProxy::GetGreens() const {
  return greens_;
}

const ClientSettings& ContinuumGameProxy::GetSettings() const {
  std::size_t addr = game_addr_ + 0x127EC + 0x1AE70; // 0x2D65C

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

  uint16_t profile_index = process_.ReadU32(module_base_menu_ + 0x47FA0) & 0xFFFF;
  std::size_t addr = process_.ReadU32(module_base_menu_ + 0x47A38) + 0x15;

  if (addr == 0) {
    return "";
  }

  addr += profile_index * ProfileStructSize;

  std::string name = process_.ReadString(addr, 23);

  name = name.substr(0, strlen(name.c_str()));

  return name;
}

int ContinuumGameProxy::GetEnergy() const {
  return player_->energy;
}

Vector2f ContinuumGameProxy::GetPosition() const {
  float x = (*position_data_) / 16.0f;
  float y = (*(position_data_ + 1)) / 16.0f;

  return Vector2f(x, y);
}

const std::vector<Player>& ContinuumGameProxy::GetPlayers() const {
  return players_;
}

const Map& ContinuumGameProxy::GetMap() const {
  return *map_;
}
const Player& ContinuumGameProxy::GetPlayer() const {
  return *player_;
}

const Player& ContinuumGameProxy::GetSelectedPlayer() const {
  u32 selected_index = *(u32*)(game_addr_ + 0x127EC + 0x1B758);

  return players_[selected_index];
}

const Player* ContinuumGameProxy::GetPlayerById(u16 id) const {
  for (std::size_t i = 0; i < players_.size(); ++i) {
    if (players_[i].id == id) {
      return &players_[i];
    }
  }

  return nullptr;
}

// TODO: Find level data in memory
std::string ContinuumGameProxy::GetServerFolder() const {
  std::size_t folder_addr = *(uint32_t*)(game_addr_ + 0x127ec + 0x5a3c) + 0x10D;
  std::string server_folder = process_.ReadString(folder_addr, 256);

  return server_folder;
}

std::string ContinuumGameProxy::GetMapName() const {
  return process_.ReadString((*(u32*)(game_addr_ + 0x127ec + 0x6C4)) + 0x01, 16);
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

void ContinuumGameProxy::Warp() {
  SendKey(VK_INSERT);
}

void ContinuumGameProxy::Cloak(KeyController& keys) {
  keys.Press(VK_SHIFT);
  SendKey(VK_HOME);
}

void ContinuumGameProxy::SetWindowFocus() {
  std::size_t focus_addr = game_addr_ + 0x3039c;

  process_.WriteU32(focus_addr, 1);
}

ExeProcess& ContinuumGameProxy::GetProcess() {
  return process_;
}

void ContinuumGameProxy::SendKey(int vKey) {
  SendMessage(hwnd_, WM_KEYDOWN, (WPARAM)vKey, 0);
  SendMessage(hwnd_, WM_KEYUP, (WPARAM)vKey, 0);
}

void ContinuumGameProxy::SendChatMessage(const std::string& mesg) const {
  typedef void(__fastcall * ChatSendFunction)(void* This, void* thiscall_garbage, char* msg, u32* unknown1,
                                              u32* unknown2);

  if (mesg.empty())
    return;

  // The address to the current text input buffer
  std::size_t chat_input_addr = game_addr_ + 0x2DD14;
  char* input = (char*)(chat_input_addr);

  memcpy(input, mesg.c_str(), mesg.length());
  input[mesg.length()] = 0;

  ChatSendFunction send_func = (ChatSendFunction)(*(u32*)(module_base_continuum_ + 0xAC30C));
  void* This = (void*)(game_addr_ + 0x2DBF0);

  // Some value that the client passes in for some reason
  u32 value = 0x4AC370;

  send_func(This, nullptr, input, &value, 0);

  // Clear the text buffer after sending the message
  input[0] = 0;
}

} // namespace marvin
