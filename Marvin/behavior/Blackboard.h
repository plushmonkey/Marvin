#pragma once

#include <any>
#include <optional>
#include <string>
#include <unordered_map>

namespace marvin {
namespace behavior {

class Blackboard {
 public:
  bool Has(const std::string& key) { return data_.find(key) != data_.end(); }

  template <typename T>
  void Set(const std::string& key, const T& value) {
    data_[key] = std::any(value);
  }

  template <typename T>
  std::optional<T> Value(const std::string& key) {
    auto iter = data_.find(key);

    if (iter == data_.end()) {
      return std::nullopt;
    }

    auto& any = iter->second;

    try {
      return std::any_cast<T>(any);
    } catch (const std::bad_any_cast&) {
      return std::nullopt;
    }
  }

  template <typename T>
  T ValueOr(const std::string& key, const T& or_result) {
    return Value<T>(key).value_or(or_result);
  }

  void Clear() { data_.clear(); }

 private:
  std::unordered_map<std::string, std::any> data_;
};

}  // namespace behavior
}  // namespace marvin
