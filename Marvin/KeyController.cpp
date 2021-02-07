#include "KeyController.h"

#include <cstring>

namespace marvin {

KeyController::KeyController() {
  ReleaseAll();
}

void KeyController::Set(KeyId key, bool down) {
  if (key >= kKeyCount)
    return;

  keys_[key] = down;
}

void KeyController::Press(KeyId key) {
  if (key >= kKeyCount)
    return;

  keys_[key] = true;
}

bool KeyController::IsPressed(KeyId key) {
  if (key >= kKeyCount)
    return false;

  return keys_[key];
}

void KeyController::Release(KeyId key) {
  if (key >= kKeyCount)
    return;

  keys_[key] = false;
}

void KeyController::ReleaseAll() {
  memset(keys_, 0, sizeof(keys_));
}

} // namespace marvin
