#include "halmet_analog.h"

#include <WString.h>

#include <Adafruit_ADS1X15.h>

#include <sensesp/sensors/sensor.h>
#include <sensesp/system/valueproducer.h>

namespace {

// ADS1115 input hardware scale factor (input voltage vs voltage at ADS1115)
constexpr float kAnalogInputScale = 29. / 2.048;

// HALMET constant measurement current (A)
constexpr float kMeasurementCurrent = 0.01;

}  // namespace

sensesp::FloatProducer* AnalogResistanceSender(Adafruit_ADS1115* ads1115,
                                               int channel,
                                               const String& name) {
  String config_path;

  config_path = "/Analog " + name + "/Resistance";
  auto* analog_sender =
      new sensesp::RepeatSensor<float>(500, [ads1115, channel]() {
        const int16_t adc_output = ads1115->readADC_SingleEnded(channel);
        const float adc_output_volts = ads1115->computeVolts(adc_output);
#if 0        
        if (channel == 0) {
          debugD("a%d_adc_output_volts: %f", channel, adc_output_volts);
        }
#endif
        const auto value =
            kAnalogInputScale * adc_output_volts / kMeasurementCurrent;
#if 0                
        if (channel == 0) {
          debugD("a%d_adc_resistance: %f", channel, value);
        }
#endif
        return value;
      });
  return analog_sender;
}
