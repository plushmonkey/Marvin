#pragma once

#include <fstream>
#include <vector>

#include "Vector2f.h"
#include "platform/Platform.h"

#define DEBUG_RENDER 0
#define DEBUG_USER_CONTROL 0

extern HWND g_hWnd;

namespace marvin {

extern std::ofstream debug_log;

enum class TextColor { White, Green, Blue, Red, Yellow, Fuchsia, DarkRed, Pink };

enum RenderTextFlags {
  RenderText_Centered = (1 << 1),
};

struct RenderableText {
  std::string text;
  Vector2f at;
  TextColor color;
  int flags;
};

struct RenderableLine {
  Vector2f from;
  Vector2f to;
  COLORREF color;
};

struct RenderState {
  std::vector<RenderableText> renderable_texts;
  std::vector<RenderableLine> renderable_lines;

  void Render();
};

extern RenderState g_RenderState;

void RenderLine(Vector2f from, Vector2f to, COLORREF color);
void RenderWorldLine(Vector2f screenCenterWorldPosition, Vector2f from, Vector2f to, COLORREF color);
void RenderText(const char* text, Vector2f at, TextColor color, int flags = 0);

Vector2f GetWindowCenter();

} // namespace marvin
