#include "halmet_digital.h"

#include "rate_limiter.h"

#include <Arduino.h>
#include <WString.h>

#include <sensesp/sensors/digital_input.h>
#include <sensesp/system/local_debug.h>
#include <sensesp/system/valueproducer.h>
#include <sensesp/transforms/frequency.h>

#ifdef ENABLE_SIGNALK
#include <sensesp/signalk/signalk_output.h>
#endif

//#include <cstdio>

namespace {

// Default RPM count scale factor, corresponds to 100 pulses per revolution.
// This is rarely, if ever correct.
constexpr float kDefaultFrequencyScale = 1 / 100.;

}  // namespace

sensesp::FloatProducer* TachoDigitalSender(int pin, const String& path_prefix,
                                           const String& sk_name,
                                           int sort_order_base) {
  String config_path;
#ifdef ENABLE_SIGNALK
  String sk_path;
#endif

  auto* tacho_input = new sensesp::DigitalInputCounter(pin, INPUT, RISING, 500);

  config_path = "/" + path_prefix + "/Revolution Multiplier";

  auto* tacho_frequency =
      new sensesp::Frequency(kDefaultFrequencyScale, config_path);

  tacho_frequency->set_description(
      "The ratio of pulses to revolutions <em>per second</em>.");
  tacho_frequency->set_sort_order(sort_order_base + 100);

  tacho_input->connect_to(tacho_frequency);

#ifdef ENABLE_SIGNALK
  config_path = "/" + path_prefix + "/Revolutions SK Path";
  sk_path = "propulsion." + sk_name + ".revolutions";

  auto* tacho_frequency_sk_output =
      new sensesp::SKOutputFloat(sk_path, config_path);

  tacho_frequency_sk_output->set_description(
      "Signal K path of the RPM output.");
  tacho_frequency_sk_output->set_sort_order(sort_order_base + 200);

  tacho_frequency->connect_to(tacho_frequency_sk_output);
#endif

#if 0
  tacho_input->attach([path_prefix, tacho_input]() {
    debugD("Input %s counter: %d", path_prefix.c_str(), tacho_input->get());
  });
#endif

  return tacho_frequency;
}

sensesp::BoolProducer* AlarmDigitalSender(int pin, const String& name,
                                          int sort_order_base) {
#ifdef ENABLE_SIGNALK
  String config_path;
  String sk_path;
#endif

  auto* alarm_input = new sensesp::DigitalInputState(pin, INPUT, 100);

  alarm_input->connect_to(new sensesp::RateLimiter<bool>(1000));

#ifdef ENABLE_SIGNALK
  config_path = "/Alarm " + name + "/SK Path";
  sk_path = "alarm." + name;

  auto* alarm_sk_output = new sensesp::SKOutputBool(sk_path, config_path);

  alarm_sk_output->set_description("Signal K path of the alarm output.");
  alarm_sk_output->set_sort_order(sort_order_base + 100);

  alarm_input->connect_to(alarm_sk_output);
#endif

  return alarm_input;
}
