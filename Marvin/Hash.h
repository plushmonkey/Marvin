#pragma once

#include <cstdlib>
#include <functional>

// Simple hasher for making structs able to be keys in unordered_maps

inline void hash_combine(std::size_t& seed) {}

template <typename T, typename... Rest>
inline void hash_combine(std::size_t& seed, const T& v, Rest... rest) {
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  hash_combine(seed, rest...);
}

// The MAKE_HASHABLE call must be done outside of any namespaces
#define MAKE_HASHABLE(T, ...)                                                                                          \
  namespace std {                                                                                                      \
  template <>                                                                                                          \
  struct hash<T> {                                                                                                     \
    std::size_t operator()(const T& t) const {                                                                         \
      std::size_t seed = 0;                                                                                            \
      hash_combine(seed, __VA_ARGS__);                                                                                 \
      return seed;                                                                                                     \
    }                                                                                                                  \
  };                                                                                                                   \
  }
