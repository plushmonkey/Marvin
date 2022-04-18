#include "Debug.h"

#include "Types.h"
#include <ddraw.h>

#pragma comment(lib, "ddraw.lib")
#pragma comment(lib, "dxguid.lib")

namespace marvin {

RenderState g_RenderState;

const bool RenderState::kDisplayDebugText = true;

std::ofstream debug_log;

#if DEBUG_RENDER

void RenderState::Render() {
  u32 graphics_addr = *(u32*)(0x4C1AFC) + 0x30;
  LPDIRECTDRAWSURFACE back_surface = (LPDIRECTDRAWSURFACE) * (u32*)(graphics_addr + 0x44);

  typedef void(__fastcall * RenderTextFunc)(void* This, void* thiscall_garbage, int x, int y, const char* text,
                                            int zero, int length, u8 alpha);

  RenderTextFunc render_text = (RenderTextFunc)(0x442FE0);
  void* This = (void*)(graphics_addr);

  HDC hdc;
  back_surface->GetDC(&hdc);

  HGDIOBJ obj = SelectObject(hdc, GetStockObject(DC_PEN));

  for (RenderableLine& renderable : renderable_lines) {
    SetDCPenColor(hdc, renderable.color);
    MoveToEx(hdc, (int)renderable.from.x, (int)renderable.from.y, NULL);
    LineTo(hdc, (int)renderable.to.x, (int)renderable.to.y);
  }

  back_surface->ReleaseDC(hdc);

  for (RenderableText& renderable : renderable_texts) {
    u32 x = (u32)renderable.at.x;
    u32 y = (u32)renderable.at.y;

    if (renderable.flags & RenderText_Centered) {
      x -= (u32)((renderable.text.length() / 2.0f) * 8.0f);
    }

    render_text(This, 0, x, y, renderable.text.c_str(), (int)renderable.color, -1, 1);
  }
}

void RenderState::RenderDebugText(const char* fmt, ...) {
  if (!kDisplayDebugText)
    return;

  char buffer[2048];

  va_list args;

  va_start(args, fmt);
  vsprintf_s(buffer, fmt, args);
  va_end(args);

  RenderText(std::string(buffer), Vector2f(GetWindowCenter().x + 150.0f, debug_y), TextColor::Pink, 0);
  debug_y += 12.0f;
}

void RenderLine(Vector2f from, Vector2f to, COLORREF color) {
  RenderableLine renderable;

  renderable.from = from;
  renderable.to = to;
  renderable.color = color;

  g_RenderState.renderable_lines.push_back(renderable);
}

void RenderWorldLine(Vector2f screenCenterWorldPosition, Vector2f from, Vector2f to, COLORREF color) {
  Vector2f center = GetWindowCenter();

  Vector2f diff = to - from;
  from = (from - screenCenterWorldPosition) * 16.0f;
  to = from + (diff * 16.0f);

  RenderLine(center + from, center + to, color);
}

void RenderText(const std::string& text, Vector2f at, TextColor color, int flags) {
  RenderableText renderable;

  renderable.text = text;
  renderable.at = at;
  renderable.color = color;
  renderable.flags = flags;

  g_RenderState.renderable_texts.push_back(std::move(renderable));
}

#else

void RenderState::Render() {}

void RenderLine(Vector2f from, Vector2f to, COLORREF color) {}
void RenderWorldLine(Vector2f screenCenterWorldPosition, Vector2f from, Vector2f to, COLORREF color) {}
void RenderText(const char* text, Vector2f at, TextColor color, int flags) {}

#endif

Vector2f GetWindowCenter() {
  RECT rect;

  GetClientRect(g_hWnd, &rect);

  return Vector2f((float)rect.right / 2.0f, (float)rect.bottom / 2.0f);
}

} // namespace marvin
