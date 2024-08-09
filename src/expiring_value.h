#ifndef HALMET_SRC_EXPIRING_VALUE_H_
#define HALMET_SRC_EXPIRING_VALUE_H_

#include <Arduino.h>

#include <cstdint>

namespace sensesp {

/**
 * @brief Keep value from update till expiration duration else return expired
 * value.
 *
 * @tparam T
 */
template <typename T>
class ExpiringValue {
 public:
  ExpiringValue()
      : value_{},
        expiration_duration_{1000},
        last_update_{0},
        expired_value_{T{}} {}

  ExpiringValue(T value, uint64_t expiration_duration, T expired_value)
      : value_{value},
        expiration_duration_{expiration_duration},
        expired_value_{expired_value},
        last_update_{millis()} {}

  void update(T value) {
    value_ = value;
    last_update_ = millis();
  }

  T get() const {
    if (!is_expired()) {
      return value_;
    } else {
      return expired_value_;
    }
  }

  bool is_expired() const {
    return millis() - last_update_ > expiration_duration_;
  }

 private:
  T value_;
  T expired_value_;
  uint64_t expiration_duration_;
  uint64_t last_update_;
};

}  // namespace sensesp

#endif
