#include "platform/Platform.h"
//
#include <detours.h>

#include <iostream>

#include "platform/ContinuumGameProxy.h"
#include "platform/ExeProcess.h"

std::unique_ptr<marvin::ContinuumGameProxy> g_GameProxy;
static bool g_Enabled = true;

static SHORT(WINAPI* RealGetAsyncKeyState)(int vKey) = GetAsyncKeyState;
static BOOL(WINAPI* RealPeekMessageA)(LPMSG lpMsg, HWND hWnd,
                                      UINT wMsgFilterMin, UINT wMsgFilterMax,
                                      UINT wRemoveMsg) = PeekMessageA;

SHORT WINAPI OverrideGetAsyncKeyState(int vKey) {
  if (!g_Enabled) {
    return RealGetAsyncKeyState(vKey);
  }

  // TODO: Check a key map
  if (vKey == VK_LEFT) {
    return (SHORT)0x8000;
  }

  if (vKey == VK_UP) {
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
  } else if (RealGetAsyncKeyState(VK_F9)) {
    g_Enabled = true;
  }

  // Continuum stops processing input when it loses focus, so update the memory
  // to make it think it always has focus.
  g_GameProxy->SetWindowFocus();

  g_GameProxy->Update(0.0f);

  // TODO: Perform update here

  return RealPeekMessageA(lpMsg, hWnd, wMsgFilterMin, wMsgFilterMax,
                          wRemoveMsg);
}

BOOL WINAPI DllMain(HINSTANCE hInst, DWORD dwReason, LPVOID reserved) {
  if (DetourIsHelperProcess()) {
    return TRUE;
  }

  if (dwReason == DLL_PROCESS_ATTACH) {
    g_GameProxy = std::make_unique<marvin::ContinuumGameProxy>();

    DetourRestoreAfterWith();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)RealGetAsyncKeyState, OverrideGetAsyncKeyState);
    DetourAttach(&(PVOID&)RealPeekMessageA, OverridePeekMessageA);
    DetourTransactionCommit();
  } else if (dwReason == DLL_PROCESS_DETACH) {
    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourDetach(&(PVOID&)RealGetAsyncKeyState, OverrideGetAsyncKeyState);
    DetourDetach(&(PVOID&)RealPeekMessageA, OverridePeekMessageA);
    DetourTransactionCommit();
  }

  return TRUE;
}
