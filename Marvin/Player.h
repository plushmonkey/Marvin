#pragma once

#define _USE_MATH_DEFINES
#include <math.h>

#include <cstdint>
#include <string>

#include "Vector2f.h"

namespace marvin {

struct Player {
  std::string name;

  uint16_t id;
  uint16_t ship;

  Vector2f position;
  Vector2f velocity;

  uint16_t frequency;
  uint16_t discrete_rotation;

  int32_t energy;
  uint8_t status;

  Vector2f GetHeading() const {
    const float kToRads = (static_cast<float>(M_PI) / 180.0f);
    float rads = (((40 - (discrete_rotation + 30)) % 40) * 9.0f) * kToRads;
    float x = std::cos(rads);
    float y = -std::sin(rads);

    return Vector2f(x, y);
  }
};

}  // namespace marvin
