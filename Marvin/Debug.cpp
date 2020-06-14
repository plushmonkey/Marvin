#include "Debug.h"

namespace marvin {

std::ofstream debug_log;

#if DEBUG_RENDER

#include <dwmapi.h>
#pragma comment(lib, "dwmapi.lib")

void RenderLine(Vector2f from, Vector2f to, COLORREF color) {
  HDC hdc = GetDC(g_hWnd);

  HGDIOBJ obj = SelectObject(hdc, GetStockObject(DC_PEN));

  SetDCPenColor(hdc, color);

  MoveToEx(hdc, (int)from.x, (int)from.y, NULL);
  LineTo(hdc, (int)to.x, (int)to.y);

  ReleaseDC(g_hWnd, hdc);
}

void RenderText(const char* text, Vector2f at, COLORREF color, int flags) {
  HDC hdc = GetDC(g_hWnd);

  HGDIOBJ obj = SelectObject(hdc, GetStockObject(DC_BRUSH));

  SetDCBrushColor(hdc, color);

  SetBkColor(hdc, RGB(0, 0, 0));
  SetTextColor(hdc, RGB(255, 255, 255));
  if (flags & RenderText_Centered) {
    SetTextAlign(hdc, TA_CENTER);
  }
  TextOutA(hdc, (int)at.x, (int)at.y, text, strlen(text));

  ReleaseDC(g_hWnd, hdc);
}

void WaitForSync() { DwmFlush(); }

#else

void RenderLine(Vector2f from, Vector2f to, COLORREF color) {}

void RenderText(const char* text, Vector2f at, COLORREF color, int flags) {}

void WaitForSync() {}
#endif

Vector2f GetWindowCenter() {
  RECT rect;

  GetClientRect(g_hWnd, &rect);

  return Vector2f((float)rect.right / 2.0f, (float)rect.bottom / 2.0f);
}

}  // namespace marvin
