#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

/*
  Injector injects the Loader.dll into the Continuum process.
  The loader is the one that loads the Marvin.dll so it can hot swap it.
*/

#ifdef UNICODE
#undef UNICODE
#endif
#define WIN32_LEAN_AND_MEAN

#include <Windows.h>
//
#include <TlHelp32.h>

#define INJECT_MODULE_NAME "Loader.dll"

namespace marvin {
namespace memory {

uint32_t ReadU32(HANDLE handle, size_t address) {
  uint32_t value = 0;
  SIZE_T num_read;

  if (ReadProcessMemory(handle, (LPVOID)address, &value, sizeof(uint32_t),
                        &num_read)) {
    return value;
  }

  return 0;
}

std::string ReadString(HANDLE handle, size_t address, size_t len) {
  std::string value;
  SIZE_T read;

  value.resize(len);

  if (ReadProcessMemory(handle, (LPVOID)address, &value[0], len, &read)) {
    return value;
  }

  return "";
}

}  // namespace memory

class Process {
 public:
  Process(DWORD process_id)
      : process_handle_(nullptr), process_id_(process_id) {}
  ~Process() {
    if (process_handle_) {
      CloseHandle(process_handle_);
    }
  }

  Process(const Process& other) = delete;
  Process& operator=(const Process& other) = delete;

  std::size_t GetModuleBase(const char* module_name) {
    HANDLE hSnapshot = CreateToolhelp32Snapshot(
        TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, process_id_);
    MODULEENTRY32 me = {0};

    me.dwSize = sizeof(me);

    if (hSnapshot == INVALID_HANDLE_VALUE) {
      return 0;
    }

    std::size_t module_base = 0;
    BOOL bModule = Module32First(hSnapshot, &me);
    while (bModule) {
      if (strcmp(module_name, me.szModule) == 0) {
        module_base = reinterpret_cast<std::size_t>(me.modBaseAddr);
        break;
      }

      bModule = Module32Next(hSnapshot, &me);
    }

    CloseHandle(hSnapshot);
    return module_base;
  }

  bool HasModule(const char* module_name) {
    HANDLE hSnapshot = CreateToolhelp32Snapshot(
        TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, process_id_);
    MODULEENTRY32 me = {0};

    me.dwSize = sizeof(me);

    if (hSnapshot == INVALID_HANDLE_VALUE) {
      return false;
    }

    bool has_module = false;
    BOOL bModule = Module32First(hSnapshot, &me);

    while (bModule) {
      if (strcmp(module_name, me.szModule) == 0) {
        has_module = true;
        break;
      }

      bModule = Module32Next(hSnapshot, &me);
    }

    CloseHandle(hSnapshot);

    return has_module;
  }

  bool InjectModule(const std::string& module_path) {
    bool injected = false;
    HMODULE hModule = GetModuleHandleA("kernel32.dll");
    HANDLE hProcess = GetHandle();

    if (!hModule) {
      std::cout << "hModule null\n";
    }

    if (!hProcess) {
      std::cout << "Failed to get handle\n";
    }

    if (hProcess && hModule) {
      // Get the address to LoadLibrary in kernel32.dll
      LPVOID load_addr = (LPVOID)GetProcAddress(hModule, "LoadLibraryA");
      // Allocate some memory in Continuum's process to store the path to the
      // DLL to load.
      LPVOID path_addr =
          VirtualAllocEx(hProcess, NULL, module_path.size(),
                         MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);

      if (!load_addr) {
        std::cout << "bad load addr\n";
      }

      if (!path_addr) {
        std::cout << "Bad allocation\n";
      }

      if (load_addr && path_addr) {
        // Write the path to the DLL to load into the recently allocated area.
        if (WriteProcessMemory(hProcess, path_addr, module_path.data(),
                               module_path.size(), NULL)) {
          // Start a remote thread in the Continuum process that immediately
          // kicks off the load.
          injected = CreateRemoteThread(hProcess, NULL, NULL,
                                        (LPTHREAD_START_ROUTINE)load_addr,
                                        path_addr, 0, NULL) != NULL;
        } else {
          std::cout << "Bad WriteProcessMemory\n";
        }
      }
    }

    return injected;
  }

  HANDLE GetHandle() {
    if (process_handle_) {
      return process_handle_;
    }

#if 0
    const DWORD desired_access =
        PROCESS_CREATE_THREAD | PROCESS_QUERY_INFORMATION |
        PROCESS_VM_OPERATION | PROCESS_VM_WRITE | PROCESS_VM_READ;
#endif
    const DWORD desired_access = PROCESS_ALL_ACCESS;

    process_handle_ = OpenProcess(desired_access, FALSE, process_id_);

    return process_handle_;
  }

  DWORD GetId() { return process_id_; }

 private:
  HANDLE process_handle_;
  DWORD process_id_;
};

bool GetDebugPrivileges() {
  HANDLE token = nullptr;
  bool success = false;

  if (OpenProcessToken(GetCurrentProcess(), TOKEN_ADJUST_PRIVILEGES, &token)) {
    TOKEN_PRIVILEGES privileges;

    LookupPrivilegeValue(NULL, SE_DEBUG_NAME, &privileges.Privileges[0].Luid);
    privileges.PrivilegeCount = 1;
    privileges.Privileges[0].Attributes = SE_PRIVILEGE_ENABLED;

    if (AdjustTokenPrivileges(token, FALSE, &privileges,
                              sizeof(TOKEN_PRIVILEGES), 0, 0))
      success = true;

    CloseHandle(token);
  }

  return success;
}

std::string GetWorkingDirectory() {
  std::string directory;

  directory.resize(GetCurrentDirectory(0, NULL));

  GetCurrentDirectory(directory.size(), &directory[0]);

  return directory.substr(0, directory.size() - 1);
}

std::vector<DWORD> GetProcessIds(const char* exe_file) {
  std::vector<DWORD> pids;
  PROCESSENTRY32 pe32 = {};

  pe32.dwSize = sizeof(PROCESSENTRY32);

  HANDLE hTool32 = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, NULL);
  BOOL bProcess = Process32First(hTool32, &pe32);

  while (bProcess) {
    if (strcmp(pe32.szExeFile, "Continuum.exe") == 0) {
      pids.push_back(pe32.th32ProcessID);
    }

    bProcess = Process32Next(hTool32, &pe32);
  }

  CloseHandle(hTool32);

  return pids;
}

class GameProxy {
 public:
  virtual ~GameProxy() {}
  virtual std::string GetName() const = 0;
};

class ContinuumGameProxy : public GameProxy {
 public:
  ContinuumGameProxy(std::unique_ptr<Process> process)
      : process_(std::move(process)) {}

  std::string GetName() const override {
    const size_t ProfileStructSize = 2860;
    DWORD pid = process_->GetId();
    HANDLE handle = process_->GetHandle();
    size_t menu_base = process_->GetModuleBase("menu040.dll");

    if (menu_base == 0) {
      return "";
    }

    uint16_t profile_index =
        memory::ReadU32(handle, menu_base + 0x47FA0) & 0xFFFF;
    size_t addr = memory::ReadU32(handle, menu_base + 0x47A38) + 0x15;

    if (addr == 0) {
      return "";
    }

    addr += profile_index * ProfileStructSize;

    std::string name = memory::ReadString(handle, addr, 23);

    name = name.substr(0, strlen(name.c_str()));

    return name;
  }

  Process& GetProcess() { return *process_; }

 private:
  std::unique_ptr<Process> process_;
};

}  // namespace marvin

DWORD SelectPid(const std::vector<DWORD>& pids, std::string target_player) {
  std::transform(target_player.begin(), target_player.end(),
                 target_player.begin(), tolower);

  for (std::size_t i = 0; i < pids.size(); ++i) {
    auto pid = pids[i];
    auto game =
        marvin::ContinuumGameProxy(std::make_unique<marvin::Process>(pid));

    std::string name = game.GetName();

    std::transform(name.begin(), name.end(), name.begin(), tolower);

    if (name == target_player) {
      return pid;
    }
  }

  std::cerr << "Failed to find process with that name." << std::endl;

  return 0;
}

DWORD SelectPid(const std::vector<DWORD>& pids) {
  for (std::size_t i = 0; i < pids.size(); ++i) {
    auto pid = pids[i];
    auto game =
        marvin::ContinuumGameProxy(std::make_unique<marvin::Process>(pid));

    std::string name = game.GetName();

    std::cout << (i + 1) << ": " << pid << " (" << name << ")";

    auto& process = game.GetProcess();

    if (process.HasModule(INJECT_MODULE_NAME)) {
      std::cout << " - Already loaded." << std::endl;
    } else {
      std::cout << std::endl;
    }
  }

  std::cout << "> ";

  std::string input;
  std::cin >> input;

  auto selection = strtol(input.c_str(), nullptr, 10);

  if (selection < 1 || selection > (long)pids.size()) {
    std::cerr << "Invalid selection." << std::endl;
    return 0;
  }

  return pids[selection - 1];
}

int main(int argc, char* argv[]) {
  if (!marvin::GetDebugPrivileges()) {
    std::cerr << "Failed to get debug privileges. Try running as Administrator."
              << std::endl;
    return EXIT_FAILURE;
  }

  auto continuum_pids = marvin::GetProcessIds("Continuum.exe");

  if (continuum_pids.empty()) {
    std::cout << "No Continuum.exe processes found." << std::endl;
    return EXIT_FAILURE;
  }

  std::string inject_path =
      marvin::GetWorkingDirectory() + "\\" + INJECT_MODULE_NAME;

  DWORD pid = 0;

  if (argc > 1) {
    std::string target_player;

    for (int i = 1; i < argc; ++i) {
      if (i != 1) {
        target_player += " ";
      }

      target_player += argv[i];
    }

    pid = SelectPid(continuum_pids, target_player);
  } else {
    pid = SelectPid(continuum_pids);
  }

  if (pid == 0) {
    return EXIT_FAILURE;
  }

  auto process = std::make_unique<marvin::Process>(pid);

  if (process->HasModule(INJECT_MODULE_NAME)) {
    std::cerr << "Invalid selection. " << pid << " already has Marvin loaded."
              << std::endl;
    return EXIT_FAILURE;
  }

  if (process->InjectModule(inject_path)) {
    std::cout << "Successfully loaded." << std::endl;
  } else {
    std::cerr << "Failed to load" << std::endl;
  }

  return EXIT_SUCCESS;
}
