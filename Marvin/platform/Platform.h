#pragma once

#ifdef UNICODE
#undef UNICODE
#endif

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

#include <string>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
#ifdef GetMessage
#undef GetMessage
#endif

inline std::string GetWorkingDirectory() {
  std::string directory;

  directory.resize(GetCurrentDirectory(0, NULL));

  GetCurrentDirectory(directory.size(), &directory[0]);

  return directory.substr(0, directory.size() - 1);
}
