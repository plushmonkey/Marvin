#include "ExeProcess.h"

#include <TlHelp32.h>

namespace marvin {

ExeProcess::ExeProcess() {
  process_handle_ = GetModuleHandle(NULL);
  process_id_ = GetCurrentProcessId();
}

ExeProcess::~ExeProcess() {}

uint32_t ExeProcess::ReadU32(std::size_t address) const {
  return *(uint32_t*)address;
}

bool ExeProcess::WriteU32(std::size_t address, uint32_t value) {
  uint32_t* data = (uint32_t*)address;
  *data = value;

  return true;
}

std::string ExeProcess::ReadString(std::size_t address,
                                   std::size_t length) const {
  std::string value;
  char* data = (char*)address;

  value.resize(length);

  for (std::size_t i = 0; i < length; ++i) {
    value[i] = data[i];
  }

  return value;
}

std::size_t ExeProcess::GetModuleBase(const char* module_name) {
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

HANDLE ExeProcess::GetHandle() { return process_handle_; }

DWORD ExeProcess::GetId() { return process_id_; }

}  // namespace marvin
