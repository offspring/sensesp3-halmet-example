#ifndef HALMET_SRC_HALMET_CONST_H_
#define HALMET_SRC_HALMET_CONST_H_

#include <Arduino.h>

namespace halmet {

// I2C pins on HALMET.
constexpr gpio_num_t kSDAPin = GPIO_NUM_21;
constexpr gpio_num_t kSCLPin = GPIO_NUM_22;

// ADS1115 I2C address
constexpr int kADS1115Address = 0x4b;

// CAN bus (NMEA 2000) pins on HALMET
constexpr gpio_num_t kCANRxPin = GPIO_NUM_18;
constexpr gpio_num_t kCANTxPin = GPIO_NUM_19;

// HALMET digital input pins
constexpr gpio_num_t kDigitalInputPin1 = GPIO_NUM_23;
constexpr gpio_num_t kDigitalInputPin2 = GPIO_NUM_25;
constexpr gpio_num_t kDigitalInputPin3 = GPIO_NUM_27;
constexpr gpio_num_t kDigitalInputPin4 = GPIO_NUM_26;

// One Wire
constexpr gpio_num_t kOneWirePin = GPIO_NUM_4;

}  // namespace halmet

#endif /* HALMET_SRC_HALMET_CONST_H_ */
