#include "platform/Platform.h"
//
#include <detours.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include "Bot.h"
#include "Debug.h"
#include "Map.h"
#include "hyperspace/Hyperspace.h"
#include "path/Pathfinder.h"
#include "platform/ContinuumGameProxy.h"
#include "platform/ExeProcess.h"

#include <ddraw.h>

const char* kEnabledText = "Continuum (enabled)";
const char* kDisabledText = "Continuum (disabled)";

using time_clock = std::chrono::high_resolution_clock;
using time_point = time_clock::time_point;
using seconds = std::chrono::duration<float>;

std::unique_ptr<marvin::Bot> g_Bot;
static bool g_Enabled = true;
static bool g_Reload = false;
HWND g_hWnd = 0;
static time_point g_LastUpdateTime;

HWND GetMainWindow();
marvin::Bot& CreateBot();

static SHORT(WINAPI* RealGetAsyncKeyState)(int vKey) = GetAsyncKeyState;
static BOOL(WINAPI* RealPeekMessageA)(LPMSG lpMsg, HWND hWnd, UINT wMsgFilterMin, UINT wMsgFilterMax,
                                      UINT wRemoveMsg) = PeekMessageA;
static BOOL(WINAPI* RealGetMessageA)(LPMSG lpMsg, HWND hWnd, UINT wMsgFilterMin, UINT wMsgFilterMax) = GetMessageA;
static HRESULT(STDMETHODCALLTYPE* RealBlt)(LPDIRECTDRAWSURFACE, LPRECT, LPDIRECTDRAWSURFACE, LPRECT, DWORD, LPDDBLTFX);

HRESULT STDMETHODCALLTYPE OverrideBlt(LPDIRECTDRAWSURFACE surface, LPRECT dest_rect, LPDIRECTDRAWSURFACE next_surface,
                                      LPRECT src_rect, DWORD flags, LPDDBLTFX fx) {

  u32 graphics_addr = *(u32*)(0x4C1AFC) + 0x30;
  LPDIRECTDRAWSURFACE primary_surface = (LPDIRECTDRAWSURFACE) * (u32*)(graphics_addr + 0x40);
  LPDIRECTDRAWSURFACE back_surface = (LPDIRECTDRAWSURFACE) * (u32*)(graphics_addr + 0x44);

  // Check if flipping. I guess there's a full screen blit instead of flip when running without vsync?
  if (surface == primary_surface && next_surface == back_surface && fx == 0) {
    marvin::g_RenderState.Render();
  }

  return RealBlt(surface, dest_rect, next_surface, src_rect, flags, fx);
}

SHORT WINAPI OverrideGetAsyncKeyState(int vKey) {
#if DEBUG_USER_CONTROL
  if (1) {
#else
  if (!g_Enabled) {
#endif
    if (GetActiveWindow() == g_hWnd) {
      return RealGetAsyncKeyState(vKey);
    }

    return 0;
  }

  if (g_Bot->GetKeys().IsPressed(vKey)) {
    return (SHORT)0x8000;
  }

  return 0;
}

bool IsGameLoaded() {
  u32 game_addr = *(u32*)0x4C1AFC;

  if (game_addr != 0) {
    // Wait for map to load
    return *(u32*)(game_addr + 0x127ec + 0x6C4) != 0;
  }

  return false;
}

BOOL WINAPI OverrideGetMessageA(LPMSG lpMsg, HWND hWnd, UINT wMsgFilterMin, UINT wMsgFilterMax) {
  if (!IsGameLoaded()) {
    g_Reload = true;
  }

  return RealGetMessageA(lpMsg, hWnd, wMsgFilterMin, wMsgFilterMax);
}

// This is used to hook into the main update loop in Continuum so the bot can be
// updated.
BOOL WINAPI OverridePeekMessageA(LPMSG lpMsg, HWND hWnd, UINT wMsgFilterMin, UINT wMsgFilterMax, UINT wRemoveMsg) {
  // Check for key presses to enable/disable the bot.
  if (GetActiveWindow() == g_hWnd) {
    if (RealGetAsyncKeyState(VK_F10)) {
      g_Enabled = false;
      SetWindowText(g_hWnd, kDisabledText);
    } else if (RealGetAsyncKeyState(VK_F9)) {
      g_Enabled = true;
      SetWindowText(g_hWnd, kEnabledText);
    }
  }

  if (g_Reload) {
    g_Enabled = false;

    if (IsGameLoaded()) {
      CreateBot();
      g_Enabled = true;
      g_Reload = false;
      g_hWnd = GetMainWindow();
      SetWindowText(g_hWnd, kEnabledText);
    }
  }

  time_point now = time_clock::now();
  seconds dt = now - g_LastUpdateTime;

  if (g_Enabled) {
    if (dt.count() >= 1.0f / 60.0f) {
#if DEBUG_RENDER
      marvin::g_RenderState.renderable_texts.clear();
      marvin::g_RenderState.renderable_lines.clear();
#endif

      g_Bot->Update(dt.count());
      g_LastUpdateTime = now;
    }
  }

  return RealPeekMessageA(lpMsg, hWnd, wMsgFilterMin, wMsgFilterMax, wRemoveMsg);
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

marvin::Bot& CreateBot() {
  auto proxy = std::make_unique<marvin::ContinuumGameProxy>(g_hWnd);

  g_Bot = std::make_unique<marvin::Bot>(std::move(proxy));

#if 0
  marvin::hs::SetHyperspaceBehavior(*g_Bot);
#endif

  return *g_Bot;
}

extern "C" __declspec(dllexport) void InitializeMarvin() {
  g_hWnd = GetMainWindow();

  marvin::debug_log.open("marvin.log", std::ios::out | std::ios::app);

  marvin::debug_log << "Starting Marvin.\n";

  try {
    CreateBot();
  } catch (std::exception& e) {
    MessageBox(NULL, e.what(), "A", MB_OK);
  }

  u32 graphics_addr = *(u32*)(0x4C1AFC) + 0x30;
  LPDIRECTDRAWSURFACE surface = (LPDIRECTDRAWSURFACE) * (u32*)(graphics_addr + 0x44);
  void** vtable = (*(void***)surface);
  RealBlt = (HRESULT(STDMETHODCALLTYPE*)(LPDIRECTDRAWSURFACE surface, LPRECT, LPDIRECTDRAWSURFACE, LPRECT, DWORD,
                                         LPDDBLTFX))vtable[5];

  DetourRestoreAfterWith();

  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourAttach(&(PVOID&)RealGetAsyncKeyState, OverrideGetAsyncKeyState);
  DetourAttach(&(PVOID&)RealPeekMessageA, OverridePeekMessageA);
  DetourAttach(&(PVOID&)RealGetMessageA, OverrideGetMessageA);
#if DEBUG_RENDER
  DetourAttach(&(PVOID&)RealBlt, OverrideBlt);
#endif
  DetourTransactionCommit();

  SetWindowText(g_hWnd, kEnabledText);

  marvin::debug_log << "Marvin started successfully." << std::endl;
}

extern "C" __declspec(dllexport) void CleanupMarvin() {
  DetourTransactionBegin();
  DetourUpdateThread(GetCurrentThread());
  DetourDetach(&(PVOID&)RealGetAsyncKeyState, OverrideGetAsyncKeyState);
  DetourDetach(&(PVOID&)RealPeekMessageA, OverridePeekMessageA);
  DetourDetach(&(PVOID&)RealGetMessageA, OverrideGetMessageA);
#if DEBUG_RENDER
  DetourDetach(&(PVOID&)RealBlt, OverrideBlt);
#endif
  DetourTransactionCommit();

  SetWindowText(g_hWnd, "Continuum");

  marvin::debug_log << "Shutting down Marvin." << std::endl;

  g_Bot = nullptr;

  marvin::debug_log.close();
}

BOOL WINAPI DllMain(HINSTANCE hInst, DWORD dwReason, LPVOID reserved) {
  return TRUE;
}
