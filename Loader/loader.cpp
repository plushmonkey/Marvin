#ifdef UNICODE
#undef UNICODE
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <TlHelp32.h>
#include <psapi.h>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

typedef void(*InitFunc)();
typedef void(*CleanupFunc)();

#define DEV_SWAP

static HMODULE hModule = NULL;
static std::string g_MarvinPath;
static std::string g_MarvinDll;
static std::string g_MarvinLoadedPath;
static FILETIME g_LastTime;
static std::thread g_MonitorThread;

std::string GetFolderFilePath(HMODULE hModule, const char* filename) {
  std::string result;
  DWORD size = 0;

  do {
    result.resize(result.size() + MAX_PATH);

    // Who designed this god damn api to not make it return real length with
    // nullptr?
    size = GetModuleFileNameA(hModule, &result[0], result.size());
  } while (size >= result.size());

  result.resize(size);

  auto pos = result.find_last_of('\\');
  result = result.substr(0, pos);

  return result + "\\" + filename;
}

bool GetLastWriteTime(const char* filename, FILETIME* ft) {
  WIN32_FILE_ATTRIBUTE_DATA data;

  if (GetFileAttributesExA(g_MarvinPath.c_str(), GetFileExInfoStandard, &data)) {
    *ft = data.ftLastWriteTime;
    return true;
  }

  return false;
}

bool WaitForUnload() {
  DWORD pid = GetCurrentProcessId();
  HANDLE hProcess = GetCurrentProcess();
  HMODULE hMods[1024];
  DWORD cbNeeded;

  bool loaded = true;

  int loops = 0;

  while (loaded) {
    loaded = false;

    ++loops;

    if (EnumProcessModules(hProcess, hMods, sizeof(hMods), &cbNeeded)) {
      for (std::size_t i = 0; i < (cbNeeded / sizeof(HMODULE)); ++i) {
        std::string module;

        module.resize(MAX_PATH);

        if (GetModuleFileNameEx(hProcess, hMods[i], &module[0], MAX_PATH)) {
          if (module.find(g_MarvinLoadedPath) != std::string::npos) {
            loaded = true;
          }
        }
      }
    }
  }

#if 0
  if (loops > 1) {
    std::string str = "Loops: " + std::to_string(loops);

    MessageBox(NULL, str.c_str(), "A", MB_OK);
  }
#endif

  return true;
}

void PerformReload() {
  if (hModule) {
    CleanupFunc cleanup = (CleanupFunc)GetProcAddress(hModule, "CleanupMarvin");

    if (cleanup) {
      cleanup();
    }

    FreeLibrary(hModule);

    WaitForUnload();
  }

  hModule = NULL;

  bool copied = false;
  for (int tries = 0; tries < 5; ++tries) {
    if (CopyFile(g_MarvinPath.c_str(), g_MarvinLoadedPath.c_str(), FALSE) != 0) {
      copied = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (!copied) {
    MessageBox(NULL, "Failed to hot swap after 5 tries. Reinjecting old dll.", "Error", MB_OK);
  }

  hModule = LoadLibrary(g_MarvinLoadedPath.c_str());

  if (hModule) {
    InitFunc init = (InitFunc)GetProcAddress(hModule, "InitializeMarvin");

    if (init) {
      init();
    }
  }
}

void MonitorDevFile() {
  FILETIME time;

  while (true) {
    if (GetLastWriteTime(g_MarvinPath.c_str(), &time)) {
      if (CompareFileTime(&time, &g_LastTime) > 0) {
        PerformReload();

        GetLastWriteTime(g_MarvinLoadedPath.c_str(), &time);
        g_LastTime = time;
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

BOOL WINAPI DllMain(HINSTANCE hInst, DWORD dwReason, LPVOID reserved) {
  if (dwReason == DLL_PROCESS_ATTACH) {
    hModule = NULL;

    g_MarvinPath = GetFolderFilePath(hInst, "Marvin.dll");

    // Copy Marvin and load the temporary dll
#ifdef DEV_SWAP
    DWORD pid = GetCurrentProcessId();
    g_MarvinDll = "Marvin-" + std::to_string(pid) + ".dll";

    g_MarvinLoadedPath = GetFolderFilePath(hInst, g_MarvinDll.c_str());

    PerformReload();
    GetLastWriteTime(g_MarvinLoadedPath.c_str(), &g_LastTime);

    g_MonitorThread = std::thread(MonitorDevFile);
#else
    hModule = LoadLibrary(g_MarvinLoadedPath.c_str());
#endif
  } else if (dwReason == DLL_PROCESS_DETACH) {
    if (hModule) {
      FreeLibrary(hModule);
    }
  }

  return TRUE;
}
