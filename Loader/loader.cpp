#ifdef UNICODE
#undef UNICODE
#endif
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <iostream>
#include <string>
#include <thread>
#include <chrono>

typedef void(*InitFunc)();
typedef void(*CleanupFunc)();

#define DEV_SWAP

static HMODULE hModule = NULL;
static std::string g_MarvinPath;
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

void PerformReload() {
  if (hModule) {
    CleanupFunc cleanup = (CleanupFunc)GetProcAddress(hModule, "CleanupMarvin");

    if (cleanup) {
      cleanup();
    }

    FreeLibrary(hModule);
  }

  hModule = NULL;

  CopyFile(g_MarvinPath.c_str(), g_MarvinLoadedPath.c_str(), FALSE);

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
    std::string tempname = "Marvin-" + std::to_string(pid) + ".dll";

    g_MarvinLoadedPath = GetFolderFilePath(hInst, tempname.c_str());

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
