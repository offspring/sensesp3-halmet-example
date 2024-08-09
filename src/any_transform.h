#ifndef HALMET_SRC_ANY_TRANSFORM_H_
#define HALMET_SRC_ANY_TRANSFORM_H_

#include <elapsedMillis.h>
#include <sensesp/transforms/transform.h>

#include <array>

namespace sensesp {

/**
 * @brief A transform that returns true if any of the input values are true.
 *
 * If any of the input values are older than the expiration duration, no
 * value is returned. Otherwise a value is emitted if any of the input values
 * is updated.
 *
 * @tparam _Nm
 */

template <std::size_t _Nm>
class AnyTransform : public sensesp::Transform<bool, bool> {
 public:
  AnyTransform(const String& config_path = "")
      : Transform<bool, bool>(config_path), value_({}) {}

  virtual void set_input(bool input, uint8_t input_channel) {
    value_[input_channel] = input;
    for (const auto& item : value_) {
      if (item) {
        this->emit(true);
        return;
      }
    }
    this->emit(false);
  }

 private:
  std::array<bool, _Nm> value_;
};

}  // namespace sensesp

#endif
