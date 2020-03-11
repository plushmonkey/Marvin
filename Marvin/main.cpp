#include "platform/Platform.h"
//
#include <detours.h>

#include <chrono>
#include <fstream>
#include <iostream>

#include "Bot.h"
#include "Map.h"
#include "platform/ContinuumGameProxy.h"
#include "platform/ExeProcess.h"

#include "path/Pathfinder.h"

const char* kEnabledText = "Continuum (enabled)";
const char* kDisabledText = "Continuum (disabled)";

using time_clock = std::chrono::high_resolution_clock;
using time_point = time_clock::time_point;
using seconds = std::chrono::duration<float>;

std::unique_ptr<marvin::Bot> g_Bot;
static bool g_Enabled = true;
static HWND g_hWnd;
static time_point g_LastUpdateTime;
static std::ofstream flog;

static SHORT(WINAPI* RealGetAsyncKeyState)(int vKey) = GetAsyncKeyState;
static BOOL(WINAPI* RealPeekMessageA)(LPMSG lpMsg, HWND hWnd,
                                      UINT wMsgFilterMin, UINT wMsgFilterMax,
                                      UINT wRemoveMsg) = PeekMessageA;

SHORT WINAPI OverrideGetAsyncKeyState(int vKey) {
  if (!g_Enabled) {
    return RealGetAsyncKeyState(vKey);
  }

  if (g_Bot->GetKeys().IsPressed(vKey)) {
    return (SHORT)0x8000;
  }

  return 0;
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

  time_point now = time_clock::now();
  seconds dt = now - g_LastUpdateTime;

  g_Bot->Update(dt.count());

  g_LastUpdateTime = now;

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

marvin::Bot& CreateBot() {
  auto proxy = std::make_unique<marvin::ContinuumGameProxy>();
  g_Bot = std::make_unique<marvin::Bot>(std::move(proxy));

  return *g_Bot;
}

BOOL WINAPI DllMain(HINSTANCE hInst, DWORD dwReason, LPVOID reserved) {
  if (DetourIsHelperProcess()) {
    return TRUE;
  }

  if (dwReason == DLL_PROCESS_ATTACH) {
    flog.open("marvin.log", std::ios::out | std::ios::app);

    flog << "Starting marvin.\n";

    CreateBot();

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
