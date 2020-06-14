#pragma once

#include <fstream>

#include "Vector2f.h"
#include "platform/Platform.h"

#define DEBUG_RENDER 0
#define DEBUG_USER_CONTROL 0

extern HWND g_hWnd;

namespace marvin {

extern std::ofstream debug_log;

enum RenderTextFlags {
  RenderText_Centered = (1 << 1),
};

void RenderLine(Vector2f from, Vector2f to, COLORREF color);
void RenderText(const char* text, Vector2f at, COLORREF color, int flags = 0);
void WaitForSync();

Vector2f GetWindowCenter();

}  // namespace marvin
