#include "platform/Platform.h"
//
#include <detours.h>

#include <fstream>
#include <iostream>
#include <limits>

#include "Map.h"
#include "platform/ContinuumGameProxy.h"
#include "platform/ExeProcess.h"

const char* kEnabledText = "Continuum (enabled)";
const char* kDisabledText = "Continuum (disabled)";

std::unique_ptr<marvin::ContinuumGameProxy> g_GameProxy;
static bool g_Enabled = true;
static HWND g_hWnd;
static std::ofstream flog;

static SHORT(WINAPI* RealGetAsyncKeyState)(int vKey) = GetAsyncKeyState;
static BOOL(WINAPI* RealPeekMessageA)(LPMSG lpMsg, HWND hWnd,
                                      UINT wMsgFilterMin, UINT wMsgFilterMax,
                                      UINT wRemoveMsg) = PeekMessageA;

static bool g_Keys[1024];

SHORT WINAPI OverrideGetAsyncKeyState(int vKey) {
  if (!g_Enabled) {
    return RealGetAsyncKeyState(vKey);
  }

  if (g_Keys[vKey]) {
    return (SHORT)0x8000;
  }

  return 0;
}

bool IsValidPosition(marvin::Vector2f position) {
  return position.x >= 0 && position.x < 1024 && position.y >= 0 &&
         position.y < 1024;
}

bool CanShoot(const marvin::Map& map, const marvin::Player& bot_player,
              const marvin::Player& target) {
  if (bot_player.position.DistanceSq(target.position) > 25 * 25) return false;
  if (map.GetTileId(bot_player.position) == marvin::kSafeTileId) return false;
  return true;
}

marvin::Vector2f CalculateShot(const marvin::Vector2f& pShooter, const marvin::Vector2f& pTarget, const marvin::Vector2f& vShooter, const marvin::Vector2f& vTarget, float sProjectile) {
  marvin::Vector2f totarget = pTarget - pShooter;
  marvin::Vector2f v = vTarget - vShooter;

  float a = v.Dot(v) - sProjectile * sProjectile;
  float b = 2 * v.Dot(totarget);
  float c = totarget.Dot(totarget);

  marvin::Vector2f solution;

  float disc = (b * b) - 4 * a * c;
  float t = -1.0;

  if (disc >= 0.0) {
    float t1 = (-b + std::sqrt(disc)) / (2 * a);
    float t2 = (-b - std::sqrt(disc)) / (2 * a);
    if (t1 < t2 && t1 >= 0)
      t = t1;
    else
      t = t2;
  }

  // Only use the calculated shot if its collision is within acceptable timeframe
  if (t < 0 || t > 5) {
    marvin::Vector2f hShooter = marvin::Normalize(vShooter);
    marvin::Vector2f hTarget = marvin::Normalize(vTarget);

    int sign = hShooter.Dot(hTarget) < 0.0 ? -1 : 1;

    float speed = vShooter.Length() + (sign * vTarget.Length()) + sProjectile;
    float look_ahead = totarget.Length() / speed;
    return marvin::Vector2f(pTarget + vTarget * look_ahead);
  }

  solution = pTarget + (v * t);

  return solution;
}

void UpdateProofOfConcept() {
  // Simple proof of concept for targeting enemies and moving around
  auto& bot_player = g_GameProxy->GetPlayer();

  const marvin::Player* target = nullptr;
  double closest_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < g_GameProxy->GetPlayers().size(); ++i) {
    const marvin::Player& player = g_GameProxy->GetPlayers()[i];

    if (player.id == g_GameProxy->GetPlayer().id) continue;
    if (player.ship > 7) continue;
    if (player.frequency == bot_player.frequency) continue;
    if (!IsValidPosition(player.position)) continue;

    double dist_sq = bot_player.position.DistanceSq(player.position);

    if (dist_sq < closest_distance) {
      closest_distance = dist_sq;
      target = &g_GameProxy->GetPlayers()[i];
    }
  }

  const float kTargetDistance = 15.0f;

  if (target) {
    // Simple steering
    auto heading = bot_player.GetHeading();
    marvin::Vector2f trajectory = target->position - g_GameProxy->GetPosition();
    float distance = trajectory.Length();
    float target_max_speed = g_GameProxy->GetSettings().ShipSettings[target->ship].MaximumSpeed / 10.0f / 16.0f;
    float target_max_thrust = g_GameProxy->GetSettings().ShipSettings[target->ship].MaximumThrust / 10.0f / 16.0f;
    float prediction = distance / target_max_speed;

    float proj_speed = g_GameProxy->GetSettings().ShipSettings[bot_player.ship].BulletSpeed / 10.0f / 16.0f;

    marvin::Vector2f seek_position = CalculateShot(g_GameProxy->GetPosition(), target->position, bot_player.velocity, target->velocity, proj_speed);

    //marvin::Vector2f seek_position = target->position;// + (target->velocity * prediction);
    marvin::Vector2f seek = seek_position - g_GameProxy->GetPosition();
    marvin::Vector2f direction = marvin::Normalize(seek);

    // Simple movement controller
    auto perp = marvin::Perpendicular(heading);
    float dot = heading.Dot(direction);
    bool clockwise = perp.Dot(direction) >= 0.0;

    bool forward = distance > kTargetDistance;
    float threshold = 0.05f;

    if (!forward) {
      threshold *= -1;
    }

    if (dot < -threshold) {
      g_Keys[VK_DOWN] = true;
    } else if (dot > threshold) {
      g_Keys[VK_UP] = true;
    }

    if (dot < 0.95f) {
      g_Keys[VK_RIGHT] = clockwise;
      g_Keys[VK_LEFT] = !clockwise;
    } else if (dot >= 0.9f) {
      if (CanShoot(g_GameProxy->GetMap(), bot_player, *target)) {
        g_Keys[VK_CONTROL] = true;
      }
    }
  }
}

// This is used to hook into the main update loop in Continuum so the bot can be
// updated.
BOOL WINAPI OverridePeekMessageA(LPMSG lpMsg, HWND hWnd, UINT wMsgFilterMin,
                                 UINT wMsgFilterMax, UINT wRemoveMsg) {
  // Check for key presses to enable/disable the bot.
  if (RealGetAsyncKeyState(VK_F10)) {
    g_Enabled = false;
    SetWindowText(g_hWnd, kDisabledText);
  } else if (RealGetAsyncKeyState(VK_F9)) {
    g_Enabled = true;
    SetWindowText(g_hWnd, kEnabledText);
  }

  // Continuum stops processing input when it loses focus, so update the memory
  // to make it think it always has focus.
  g_GameProxy->SetWindowFocus();

  g_GameProxy->Update(0.0f);

  memset(g_Keys, 0, sizeof(g_Keys));

  // TODO: Perform real update here

  UpdateProofOfConcept();

  return RealPeekMessageA(lpMsg, hWnd, wMsgFilterMin, wMsgFilterMax,
                          wRemoveMsg);
}

HWND GetMainWindow() {
  DWORD proc_id = GetCurrentProcessId();
  HWND hwnd = nullptr;

  char title[1024];

  do {
    hwnd = FindWindowEx(nullptr, hwnd, nullptr, nullptr);
    DWORD check_id;
    GetWindowThreadProcessId(hwnd, &check_id);

    if (check_id == proc_id) {
      GetWindowText(hwnd, title, 1024);

      if (strcmp(title, "Continuum") == 0) {
        return hwnd;
      }
    }
  } while (hwnd);

  return NULL;
}

BOOL WINAPI DllMain(HINSTANCE hInst, DWORD dwReason, LPVOID reserved) {
  if (DetourIsHelperProcess()) {
    return TRUE;
  }

  if (dwReason == DLL_PROCESS_ATTACH) {
    flog.open("marvin.log", std::ios::out | std::ios::app);

    flog << "Starting marvin.\n";

    g_GameProxy = std::make_unique<marvin::ContinuumGameProxy>();

    DetourRestoreAfterWith();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)RealGetAsyncKeyState, OverrideGetAsyncKeyState);
    DetourAttach(&(PVOID&)RealPeekMessageA, OverridePeekMessageA);
    DetourTransactionCommit();

    g_hWnd = GetMainWindow();

    SetWindowText(g_hWnd, kEnabledText);

  } else if (dwReason == DLL_PROCESS_DETACH) {
    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourDetach(&(PVOID&)RealGetAsyncKeyState, OverrideGetAsyncKeyState);
    DetourDetach(&(PVOID&)RealPeekMessageA, OverridePeekMessageA);
    DetourTransactionCommit();
  }

  return TRUE;
}
