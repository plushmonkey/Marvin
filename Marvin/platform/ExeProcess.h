#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "Platform.h"

namespace marvin {

class ExeProcess {
 public:
  ExeProcess();
  ~ExeProcess();

  ExeProcess(const ExeProcess& other) = delete;
  ExeProcess& operator=(const ExeProcess& other) = delete;

  uint32_t ReadU32(std::size_t address) const;
  std::string ReadString(std::size_t address, std::size_t length) const;

  bool WriteU32(std::size_t address, uint32_t value);

  std::size_t GetModuleBase(const char* module_name);

  HANDLE GetHandle();
  DWORD GetId();

 private:
  HANDLE process_handle_;
  DWORD process_id_;
};

}  // namespace marvin
