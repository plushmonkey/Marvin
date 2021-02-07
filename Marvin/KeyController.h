#pragma once

namespace marvin {

using KeyId = int;
constexpr KeyId kKeyCount = 256;

class KeyController {
public:
  KeyController();

  void Set(KeyId key, bool down);
  void Press(KeyId key);
  void Release(KeyId key);
  void ReleaseAll();

  bool IsPressed(KeyId key);

private:
  bool keys_[kKeyCount];
};

} // namespace marvin
