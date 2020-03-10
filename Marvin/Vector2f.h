#pragma once

#include <cmath>
#include <iostream>
#include <limits>

namespace marvin {

class Vector2f {
 public:
  union {
    struct {
      float x;
      float y;
    };

    float values[2];
  };

  Vector2f() : x(0), y(0) {}
  Vector2f(float x, float y) : x(x), y(y) {}

  Vector2f(const Vector2f& other) : x(other.x), y(other.y) {}

  Vector2f& operator=(const Vector2f& other) {
    x = other.x;
    y = other.y;
    return *this;
  }

  bool operator==(const Vector2f& other) const noexcept {
    constexpr float epsilon = 0.000001f;
    return std::fabs(x - other.x) < epsilon && std::fabs(y - other.y) < epsilon;
  }

  bool operator!=(const Vector2f& other) const noexcept {
    return !(*this == other);
  }

  float operator[](std::size_t index) { return values[index]; }

  Vector2f operator-() const { return Vector2f(-x, -y); }

  inline Vector2f& operator+=(float v) noexcept {
    x += v;
    return *this;
  }

  inline Vector2f& operator-=(float v) noexcept {
    x -= v;
    return *this;
  }

  inline Vector2f& operator+=(const Vector2f& other) noexcept {
    x += other.x;
    y += other.y;
    return *this;
  }

  inline Vector2f& operator-=(const Vector2f& other) noexcept {
    x -= other.x;
    y -= other.y;
    return *this;
  }

  inline Vector2f& operator*=(float v) noexcept {
    x *= v;
    y *= v;
    return *this;
  }

  inline Vector2f& operator/=(float v) {
    x /= v;
    y /= v;
    return *this;
  }

  inline Vector2f operator+(const Vector2f& other) const {
    return Vector2f(x + other.x, y + other.y);
  }

  inline Vector2f operator-(const Vector2f& other) const {
    return Vector2f(x - other.x, y - other.y);
  }

  inline Vector2f operator+(float v) const { return Vector2f(x + v, y + v); }

  inline Vector2f operator-(float v) const { return Vector2f(x - v, y - v); }

  inline Vector2f operator*(float v) const { return Vector2f(x * v, y * v); }

  inline Vector2f operator/(float v) const { return Vector2f(x / v, y / v); }

  inline float operator*(const Vector2f& other) const { return Dot(other); }

  inline float Length() const { return std::sqrt(x * x + y * y); }

  inline float LengthSq() const { return x * x + y * y; }

  inline float Distance(const Vector2f& other) const {
    Vector2f to_other = other - *this;
    return to_other.Length();
  }

  inline float DistanceSq(const Vector2f& other) const {
    Vector2f to_other = other - *this;
    return to_other.LengthSq();
  }

  inline void Normalize() {
    float len = Length();

    if (len > std::numeric_limits<float>::epsilon() * 2) {
      x /= len;
      y /= len;
    }
  }

  inline float Dot(const Vector2f& other) const noexcept {
    return x * other.x + y * other.y;
  }

  inline Vector2f& Truncate(float len) {
    if (LengthSq() > len * len) {
      Normalize();
      *this *= len;
    }

    return *this;
  }
};

inline float DotProduct(const Vector2f& v1, const Vector2f& v2) noexcept {
  return v1.x * v2.x + v1.y * v2.y;
}

inline Vector2f Normalize(const Vector2f& v) {
  Vector2f result = v;

  result.Normalize();

  return result;
}

inline Vector2f Perpendicular(const Vector2f& v) { return Vector2f(-v.y, v.x); }

inline bool operator<(const Vector2f& lhs, const Vector2f& rhs) noexcept {
  if (lhs.y < rhs.y) {
    return true;
  } else if (lhs.y > rhs.y) {
    return false;
  }

  return lhs.x < rhs.x;
}

}  // namespace marvin

inline std::ostream& operator<<(std::ostream& out, const marvin::Vector2f& v) {
  return out << "(" << v.x << ", " << v.y << ")";
}
